/*
 * RespRateEstimator.h
 * ---------------------------------------------------------------------------
 * ESTIMACIÓN (no medición directa) de la frecuencia respiratoria (FR) a partir
 * de ECG + PPG de un canal cada uno.  Núcleo de procesamiento portable: NO
 * depende de Arduino ni de la biblioteca AFE4950, de modo que se puede probar
 * en un PC (ver host_test/) y luego usar tal cual en el ESP32.
 *
 * Método (resumen; ver informe):
 *   1. ECG  -> detección de QRS (banda 5-15 Hz) -> serie RR (RSA) y amplitud
 *              pico a pico del QRS (EDR).
 *   2. PPG  -> detección de pulsos (banda 0.5-5 Hz) -> línea base por latido
 *              (RIIV), amplitud del pulso (RIAV) y intervalo entre pulsos.
 *   3. Cada serie latido a latido (5 en total) se edita (latidos ectópicos /
 *      falsos), se remuestrea a 4 Hz, se le quita la tendencia, se aplica
 *      ventana de Hann y se busca el pico espectral (Goertzel) en la banda
 *      respiratoria permitida.
 *   4. Fusión: la FR se acepta solo si >= 2 características coinciden
 *      (+-3 lpm).  Si coinciden una de ECG y una de PPG -> confianza ALTA.
 *   5. Suavizado temporal con mediana de 3 + EMA y límite de saltos.
 *
 * Unidades: las muestras se aceptan en "cuentas" del ADC (el algoritmo es
 * invariante a la escala salvo los umbrales de Config marcados AJUSTAR).
 *
 * Flujos de entrada (v2): ECG a fsEcg (500 Hz) con pushEcg(), PPG a fsEcg/ecgPerPpg
 * (250 Hz) con pushPpg().  Esa es la configuración del manual del AFE4950 (modo MIX:
 * ECG 500 Hz, PPG 500 Hz con decimación 2, FIFO = ECG, ECG, PPG).  La base de tiempo
 * del algoritmo es el contador de muestras de ECG.
 * ---------------------------------------------------------------------------
 */
#ifndef RESP_RATE_ESTIMATOR_H
#define RESP_RATE_ESTIMATOR_H

#include <stdint.h>
#include <math.h>
#include <string.h>

namespace RespRate {

static const float kPi = 3.14159265358979f;

// ---- Dimensionamiento estático (memoria fija, sin malloc) -----------------
static const int kWindowS     = 32;                    // ventana de análisis [s]
static const int kGridHz      = 4;                     // remuestreo de series respiratorias [Hz]
static const int kGridMax     = kWindowS * kGridHz + 4;
static const int kMaxBeats    = 160;                   // latidos por canal (32 s a 200 lpm = 107)
static const int kRingLen     = 256;                   // buffers circulares ECG (potencia de 2)
static const int kRingMask    = kRingLen - 1;
static const int kMaMax       = 128;                   // longitud máx. de la media móvil de energía
static const int kNumFeatures = 5;
static const int kMaxFreqBins = 160;

// Características respiratorias (índice en los vectores de resultados)
enum Feature { F_PPG_RIIV = 0,   // línea base del PPG (intensidad)
               F_PPG_RIAV = 1,   // amplitud del pulso PPG
               F_PPG_RIFV = 2,   // intervalo entre pulsos PPG (modulación de frecuencia)
               F_ECG_RIFV = 3,   // intervalo RR del ECG (arritmia sinusal respiratoria)
               F_ECG_EDR  = 4 }; // amplitud pico a pico del QRS (EDR)

enum Status : uint8_t {
  ST_WARMUP = 0,      // reuniendo la primera ventana completa
  ST_OK_HIGH,         // estimación aceptada, coinciden ECG y PPG
  ST_OK_MEDIUM,       // estimación aceptada, coinciden >=2 características de un solo sensor
  ST_HELD,            // ventana actual no fiable: se muestra el último valor aceptado (con edad)
  ST_UNRELIABLE,      // sin valor que mostrar
  ST_NO_SIGNAL        // ni ECG ni PPG presentes
};

// Motivos (máscara de bits) por los que la ventana no produjo estimación
enum Reason : uint16_t {
  R_NO_PPG        = 1,
  R_NO_ECG        = 2,
  R_FEW_BEATS     = 4,
  R_ARTIFACT      = 8,     // latidos rechazados / amplitud errática / huecos largos
  R_SATURATED     = 16,
  R_NO_AGREEMENT  = 32,
  R_LOW_QUALITY   = 64,
  R_NYQUIST       = 128,   // FC demasiado baja para resolver la banda respiratoria
  R_UNCONFIRMED   = 256    // candidata pendiente de confirmación en la ventana siguiente
};

struct Config {
  // Banda respiratoria admitida [respiraciones/min] (adultos)
  float rrMinBpm      = 6.0f;
  float rrMaxBpm      = 40.0f;
  float nyquistFrac   = 0.40f;   // FR_max <= 0.40*FC: la serie latido a latido muestrea a la FC

  // --- AJUSTAR con señales reales de vuestro hardware (cuentas del ADC) ---
  float ppgMinP2P     = 300.0f;      // pico a pico mínimo del PPG filtrado 0.5-5 Hz (cuentas)
  float ecgMinP2P     = 1500.0f;     // pico a pico mínimo del QRS (cuentas; ruido p-p del AFE ~100 cuentas con INA x21)
  float satCounts     = 2000000.0f;  // |valor| >= esto => saturación (el ADC satura en +-2^21 = 2 097 152)

  // Edición de latidos
  float minBeatInterval = 0.30f;     // s  (200 lpm)
  float maxBeatInterval = 2.00f;     // s  (30 lpm)
  float intervalTol     = 0.30f;     // desviación máx. respecto de la mediana
  float minValidFrac    = 0.70f;     // fracción mínima de latidos válidos en la ventana
  float maxAmpCv        = 0.60f;     // CV máx. de la amplitud (artefacto de movimiento)
  int   minBeatsWindow  = 20;        // >= 20 latidos en 32 s (FC >= ~38 lpm)

  // Estimación espectral y fusión
  float peakHalfWidthHz = 0.05f;     // ancho a cada lado del pico para la métrica de calidad
  float minPeakQuality  = 0.50f;     // fracción mínima de potencia en el pico (AJUSTAR con datos reales)
  float minQualitySingle= 0.65f;     // exigencia mayor si solo coinciden características de UN sensor
  float edgeMarginHz    = 0.02f;     // picos a < 0.02 Hz del borde de banda se descartan (deriva / aliasing)
  float clusterTolBpm   = 2.0f;      // tolerancia de coincidencia entre características

  // Suavizado / pantalla
  float hopS            = 5.0f;      // periodo de actualización de la estimación
  float maxJumpBpm      = 8.0f;      // salto máx. admitido sin confirmación
  float emaAlpha        = 0.4f;
  float holdS           = 30.0f;     // tiempo máx. que se retiene el último valor
};

struct Result {
  Status   status;
  float    rrBpm;         // FR suavizada para mostrar (válida solo si hasValue)
  bool     hasValue;
  float    ageS;          // segundos desde la última estimación aceptada
  float    rrRaw;         // estimación bruta de la última ventana (NAN si no hubo)
  float    conf;          // 0..1, índice HEURÍSTICO de coherencia (no es una probabilidad calibrada)
  uint8_t  nAgree;        // características que coinciden
  uint16_t reasons;       // máscara Reason
  float    hrEcg, hrPpg;  // FC derivada de los detectores (lpm), 0 si no disponible
  float    featBpm[kNumFeatures];
  float    featQ[kNumFeatures];
  float    warmFrac;      // progreso de la primera ventana 0..1
};

// ---------------------------------------------------------------------------
// Filtro bicuadrático (Butterworth 2.º orden, receta RBJ), forma II transpuesta
// ---------------------------------------------------------------------------
struct Biquad {
  float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0, z1 = 0, z2 = 0;
  void lowpass(float fs, float fc)  { design(fs, fc, false); }
  void highpass(float fs, float fc) { design(fs, fc, true);  }
  void design(float fs, float fc, bool hp) {
    const float q = 0.70710678f;
    float w0 = 2.0f * kPi * fc / fs, cw = cosf(w0), sw = sinf(w0), al = sw / (2.0f * q);
    float a0 = 1.0f + al;
    if (hp) { b0 = (1 + cw) / 2 / a0; b1 = -(1 + cw) / a0; b2 = b0; }
    else    { b0 = (1 - cw) / 2 / a0; b1 =  (1 - cw) / a0; b2 = b0; }
    a1 = -2.0f * cw / a0; a2 = (1.0f - al) / a0;
  }
  inline float process(float x) {
    float y = b0 * x + z1;
    z1 = b1 * x - a1 * y + z2;
    z2 = b2 * x - a2 * y;
    return y;
  }
  void reset() { z1 = z2 = 0; }
};

// ---------------------------------------------------------------------------
class Estimator {
 public:
  explicit Estimator(const Config& c = Config()) : cfg(c) { memset(&res, 0, sizeof(res)); }

  // Diseña filtros para la tasa de ECG fsEcg [Hz] (medida, no nominal) y reinicia.
  void begin(float fsEcg, int ecgSamplesPerPpg = 2) {
    fs = fsEcg; fsTime = fsEcg;
    ecgPerPpg = (ecgSamplesPerPpg < 1) ? 1 : ecgSamplesPerPpg;
    float fsPpgNative = fs / ecgPerPpg;             // 250 Hz en la configuración de la clase
    decim   = (int)(fsPpgNative / 100.0f); if (decim < 1) decim = 1;
    fsPpg   = fsPpgNative / decim;                  // 125 Hz tras decimar
    secCycles = (uint32_t)(fs + 0.5f);
    hopCycles = (uint32_t)(cfg.hopS * fs + 0.5f);
    maLen   = (int)(0.08f * fs + 0.5f); if (maLen < 3) maLen = 3; if (maLen > kMaMax) maLen = kMaMax;
    winN    = (int)(0.12f * fs + 0.5f); if (winN > kRingLen - 8) winN = kRingLen - 8;
    refracEcg = (uint32_t)(0.25f * fs);
    refracPpgK = (uint32_t)(0.30f * fsPpg);
    ecgHP.highpass(fs, 5.0f);  ecgLP.lowpass(fs, 15.0f);
    ppgLP.lowpass(fsPpgNative, 10.0f);              // anti-alias previo a la decimación
    ppgHP.highpass(fsPpg, 0.5f); ppgBL.lowpass(fsPpg, 5.0f);
    reset();
  }

  // Corrige solo la conversión ciclos->segundos (deriva lenta del reloj del AFE).
  void setTimeBase(float fsMeasured) {
    if (fsMeasured > 0.8f * fs && fsMeasured < 1.25f * fs) fsTime = fsMeasured;
  }

  void reset() {
    cyc = 0; ecgOffSet = false; ppgOffSet = false; kPpg = 0; decCount = 0;
    ecgHP.reset(); ecgLP.reset(); ppgLP.reset(); ppgHP.reset(); ppgBL.reset();
    memset(rawRing, 0, sizeof(rawRing)); memset(zRing, 0, sizeof(zRing));
    memset(maRing, 0, sizeof(maRing)); maPos = 0; maSum = 0; m1 = m2 = 0;
    learnMaxE = 0; learnedE = false; ecgSpk = 0; haveQrs = false; lastQrsCyc = 0;
    zMin = zMax = 0; zMinPrev = zMaxPrev = 0;
    ppgSign = 1.0f; py1 = py2 = 0; yPrevRaw = 0; havePeak = false; lastPeakK = 0; ppgPk = 0;
    learnMaxY = 0; learnedP = false; learnUntilK = (uint32_t)(2.0f * fsPpg);
    sumX = 0; cntX = 0; minY = 0; yMin = yMax = 0; yMinPrev = yMaxPrev = 0;
    sumD2 = sumD3 = 0; nD = 0; tickCount = 0;
    ppgPresent = ecgPresent = false; lastSatCyc = 0; everSat = false; lastSatEcgCyc = 0; everSatEcg = false; noSignalSecs = 0;
    clearBeats();
    dataStartCyc = (uint32_t)(3.0f * fs);          // 3 s de asentamiento de filtros
    nextSecondCyc = secCycles; nextHopCyc = hopCycles;
    sm.have = false; sm.nh = 0; sm.pendCnt = 0;
    noSignalNow = false;
    memset(&res, 0, sizeof(res)); res.status = ST_WARMUP; res.rrRaw = NAN;
  }

  // Muestra de ECG (500 Hz). 'sat' = la palabra del AFE venía marcada como saturada.
  // Devuelve true cuando se terminó de evaluar una nueva ventana (cada hopS).
  bool pushEcg(float x, bool sat = false) {
    if (!ecgOffSet) { ecgOff = x; ecgOffSet = true; }
    if (sat || fabsf(x) >= cfg.satCounts) { lastSatEcgCyc = cyc; everSatEcg = true; }
    processEcg(x - ecgOff);
    cyc++;
    if (cyc >= nextSecondCyc) { oneSecondTick(); nextSecondCyc += secCycles; }
    if (cyc >= nextHopCyc)    { nextHopCyc += hopCycles; computeWindow(); return true; }
    return false;
  }

  // Muestra de PPG (250 Hz). Llamar en el orden del FIFO (ECG, ECG, PPG).
  void pushPpg(float x, bool sat = false) {
    if (!ppgOffSet) { ppgOff = x; ppgOffSet = true; }
    if (sat || fabsf(x) >= cfg.satCounts) { lastSatCyc = cyc; everSat = true; }
    processPpg(x - ppgOff);
  }

  // Resultado con la edad actualizada (llamar cada ~1 s para refrescar pantalla).
  Result snapshot() const {
    Result r = res;
    if (noSignalNow) { r.status = ST_NO_SIGNAL; r.hasValue = false; return r; }
    if (r.status == ST_WARMUP) return r;
    r.ageS = sm.have ? (float)(cyc - sm.lastAcceptCyc) / fsTime : 1e9f;
    r.hasValue = sm.have && r.ageS <= cfg.holdS;
    r.rrBpm = r.hasValue ? sm.out : 0.0f;
    if (!r.hasValue) { if (r.status == ST_HELD || r.status == ST_OK_HIGH || r.status == ST_OK_MEDIUM) r.status = ST_UNRELIABLE; }
    else if (r.ageS > 1.5f * cfg.hopS && (r.status == ST_OK_HIGH || r.status == ST_OK_MEDIUM)) r.status = ST_HELD;
    return r;
  }
  const Result& lastWindow() const { return res; }
  float sampleRate() const { return fsTime; }

 private:
  Config cfg;
  Result res;

  // ---- parámetros derivados ----
  float fs = 500, fsTime = 500, fsPpg = 100;
  int decim = 5, maLen = 40, winN = 60;
  uint32_t secCycles = 500, hopCycles = 2500, refracEcg = 125, refracPpgK = 30;

  // ---- estado general ----
  uint32_t cyc = 0, dataStartCyc = 0, nextSecondCyc = 0, nextHopCyc = 0;
  bool ecgOffSet = false, ppgOffSet = false; float ecgOff = 0, ppgOff = 0;
  int ecgPerPpg = 2;
  int tickCount = 0, noSignalSecs = 0; bool noSignalNow = false;
  bool ppgPresent = false, ecgPresent = false, everSat = false; uint32_t lastSatCyc = 0;
  bool everSatEcg = false; uint32_t lastSatEcgCyc = 0;

  // ---- ECG ----
  Biquad ecgHP, ecgLP;
  float rawRing[kRingLen], zRing[kRingLen];
  float maRing[kMaMax]; int maPos = 0; double maSum = 0;
  float m1 = 0, m2 = 0, learnMaxE = 0, ecgSpk = 0; bool learnedE = false;
  bool haveQrs = false; uint32_t lastQrsCyc = 0;
  float zMin = 0, zMax = 0, zMinPrev = 0, zMaxPrev = 0;

  // ---- PPG ----
  Biquad ppgLP, ppgHP, ppgBL;
  int decCount = 0; uint32_t kPpg = 0, lastPeakK = 0, learnUntilK = 0;
  float ppgSign = 1, py1 = 0, py2 = 0, yPrevRaw = 0, ppgPk = 0, learnMaxY = 0; bool learnedP = false, havePeak = false;
  double sumX = 0; int cntX = 0; float minY = 0;
  float yMin = 0, yMax = 0, yMinPrev = 0, yMaxPrev = 0;
  double sumD2 = 0, sumD3 = 0; int nD = 0;

  // ---- latidos ----
  struct PpgBeat { uint32_t cyc; float frac, base, amp; };
  struct EcgBeat { uint32_t cyc; float frac, amp; };
  PpgBeat ppgB[kMaxBeats]; int ppgHead = 0, ppgCount = 0;
  EcgBeat ecgB[kMaxBeats]; int ecgHead = 0, ecgCount = 0;
  void clearBeats() { ppgHead = ppgCount = ecgHead = ecgCount = 0; }

  // ---- espacio de trabajo de ventana ----
  float wt[kMaxBeats], wint[kMaxBeats], wbase[kMaxBeats], wamp[kMaxBeats], tmpA[kMaxBeats];
  bool  wok[kMaxBeats];
  float ft[kMaxBeats], fv[kMaxBeats], gx[kGridMax], pw[kMaxFreqBins];

  // ---- suavizado ----
  struct Smoother {
    float hist[3]; int nh = 0; float out = 0; bool have = false; uint32_t lastAcceptCyc = 0;
    float pendVal = 0; int pendCnt = 0;
  } sm;

  // =========================================================================
  //                               ECG
  // =========================================================================
  void processEcg(float x) {
    uint32_t n = cyc;
    rawRing[n & kRingMask] = x;
    float z = ecgLP.process(ecgHP.process(x));
    zRing[n & kRingMask] = z;
    if (z < zMin) zMin = z;
    if (z > zMax) zMax = z;

    float e = z * z;
    maSum += (double)e - (double)maRing[maPos];
    maRing[maPos] = e; if (++maPos >= maLen) maPos = 0;
    float m = (float)(maSum / maLen); if (m < 0) m = 0;

    if (!learnedE) {                                  // primeros 2 s: aprender nivel de QRS
      if (m > learnMaxE) learnMaxE = m;
      if (n >= (uint32_t)(2.0f * fs)) { ecgSpk = 0.5f * learnMaxE; learnedE = true; }
    } else {
      float thr = 0.3f * ecgSpk;
      if (m1 > m2 && m1 >= m && m1 > thr && n >= 2 &&
          (!haveQrs || (n - 1) - lastQrsCyc >= refracEcg)) {
        detectQrs(n - 1, m1);
      }
    }
    m2 = m1; m1 = m;
  }

  void detectQrs(uint32_t endIdx, float mPeak) {
    if (endIdx < (uint32_t)winN + 2) return;
    uint32_t start = endIdx - winN + 1;
    float amax = -1, rmin = 1e30f, rmax = -1e30f; uint32_t imax = start;
    for (int j = 0; j < winN; j++) {
      uint32_t idx = start + j;
      float zz = fabsf(zRing[idx & kRingMask]);
      if (zz > amax) { amax = zz; imax = idx; }
      float r = rawRing[idx & kRingMask];
      if (r < rmin) rmin = r;
      if (r > rmax) rmax = r;
    }
    float p2p = rmax - rmin;
    if (p2p < cfg.ecgMinP2P) return;                  // ruido / electrodo suelto
    float ym = fabsf(zRing[(imax - 1) & kRingMask]), yp = fabsf(zRing[(imax + 1) & kRingMask]);
    float den = ym - 2 * amax + yp, frac = (den != 0.0f) ? 0.5f * (ym - yp) / den : 0.0f;
    frac = fmaxf(-0.5f, fminf(0.5f, frac));
    EcgBeat& b = ecgB[ecgHead];
    b.cyc = imax; b.frac = frac; b.amp = p2p;
    ecgHead = (ecgHead + 1) % kMaxBeats; if (ecgCount < kMaxBeats) ecgCount++;
    haveQrs = true; lastQrsCyc = imax;
    ecgSpk = 0.875f * ecgSpk + 0.125f * mPeak;
  }

  // =========================================================================
  //                               PPG
  // =========================================================================
  void processPpg(float x) {
    float xl = ppgLP.process(x);
    if (++decCount < decim) return;
    decCount = 0;
    ppgSample(xl);
  }

  void ppgSample(float x100) {
    uint32_t k = kPpg++;
    float yraw = ppgBL.process(ppgHP.process(x100));
    float d = yraw - yPrevRaw; yPrevRaw = yraw;
    if (k > 0) { sumD2 += (double)d * d; sumD3 += (double)d * d * d; nD++; }
    if (yraw < yMin) yMin = yraw;
    if (yraw > yMax) yMax = yraw;
    float y = ppgSign * yraw;

    if (!learnedP) {                                  // aprender amplitud de pulso
      if (y > learnMaxY) learnMaxY = y;
      if (k >= learnUntilK) { ppgPk = 0.7f * learnMaxY; learnedP = true; havePeak = false; }
      py2 = py1; py1 = y; return;
    }
    float thr = fmaxf(0.25f * cfg.ppgMinP2P, 0.4f * ppgPk);
    bool isPeak = (k >= 2) && py1 > py2 && py1 >= y && py1 > thr &&
                  (!havePeak || (k - 1) - lastPeakK >= refracPpgK);
    if (isPeak) {
      if (havePeak && cntX > 0) {
        float den = py2 - 2 * py1 + y, frac = (den != 0.0f) ? 0.5f * (py2 - y) / den : 0.0f;
        frac = fmaxf(-0.5f, fminf(0.5f, frac));
        PpgBeat& b = ppgB[ppgHead];
        b.cyc  = ((k - 1) * (uint32_t)decim + (uint32_t)(decim - 1)) * (uint32_t)ecgPerPpg;   // en muestras de ECG
        b.frac = frac * decim * ecgPerPpg;
        b.base = (float)(sumX / cntX);
        b.amp  = py1 - minY;
        ppgHead = (ppgHead + 1) % kMaxBeats; if (ppgCount < kMaxBeats) ppgCount++;
      }
      ppgPk = (ppgPk > 0) ? 0.8f * ppgPk + 0.2f * py1 : py1;
      havePeak = true; lastPeakK = k - 1;
      sumX = x100; cntX = 1; minY = y;
    } else {
      sumX += x100; cntX++; if (y < minY || cntX == 1) minY = y;
    }
    py2 = py1; py1 = y;
  }

  // =========================================================================
  //                       Tarea de 1 s (presencia, decaimientos)
  // =========================================================================
  void oneSecondTick() {
    tickCount++;
    float pMin = fminf(yMin, yMinPrev), pMax = fmaxf(yMax, yMaxPrev);
    bool sat = everSat && (cyc - lastSatCyc) < (uint32_t)(2.0f * fs);
    ppgPresent = ((pMax - pMin) >= cfg.ppgMinP2P) && !sat;
    float qMin = fminf(zMin, zMinPrev), qMax = fmaxf(zMax, zMaxPrev);
    bool satE = everSatEcg && (cyc - lastSatEcgCyc) < (uint32_t)(2.0f * fs);
    ecgPresent = ((qMax - qMin) >= cfg.ecgMinP2P) && !satE;
    yMinPrev = yMin; yMaxPrev = yMax; yMin = 1e30f; yMax = -1e30f;
    zMinPrev = zMin; zMaxPrev = zMax; zMin = 1e30f; zMax = -1e30f;

    if (havePeak && (kPpg - lastPeakK) > (uint32_t)(2.5f * fsPpg)) ppgPk *= 0.7f;   // recuperar umbral
    if (haveQrs && (cyc - lastQrsCyc) > (uint32_t)(2.5f * fs))     ecgSpk *= 0.8f;

    if (!ppgPresent && !ecgPresent) noSignalSecs++; else noSignalSecs = 0;
    noSignalNow = (noSignalSecs >= 5);
    if (noSignalSecs == 10) {                                // pérdida prolongada: reiniciar ventana
      clearBeats(); dataStartCyc = cyc; sm.have = false; sm.nh = 0; sm.pendCnt = 0;
    }

    if (tickCount % 5 == 0 && ppgPresent && nD > 100) {      // polaridad del PPG (asimetría de la derivada)
      double v2 = sumD2 / nD; double sk = (v2 > 0) ? (sumD3 / nD) / pow(v2, 1.5) : 0;
      float newSign = ppgSign;
      if (sk > 0.25) newSign = 1.0f; else if (sk < -0.25) newSign = -1.0f;
      if (newSign != ppgSign) {                              // invertido: reaprender
        ppgSign = newSign; learnedP = false; learnMaxY = 0; learnUntilK = kPpg + (uint32_t)(2.0f * fsPpg);
        havePeak = false; ppgPk = 0; py1 = py2 = 0; ppgCount = ppgHead = 0;
      }
      sumD2 = sumD3 = 0; nD = 0;
    }
  }

  // =========================================================================
  //                      Utilidades numéricas pequeñas
  // =========================================================================
  float medianOf(const float* a, int n, float* tmp) {
    for (int i = 0; i < n; i++) tmp[i] = a[i];
    for (int i = 1; i < n; i++) { float v = tmp[i]; int j = i - 1; while (j >= 0 && tmp[j] > v) { tmp[j + 1] = tmp[j]; j--; } tmp[j + 1] = v; }
    return (n & 1) ? tmp[n / 2] : 0.5f * (tmp[n / 2 - 1] + tmp[n / 2]);
  }

  // =========================================================================
  //        Análisis de una modalidad (ECG o PPG) en la ventana actual
  // =========================================================================
  struct ModStat { int nBeats = 0, nValid = 0; float hrBpm = 0, hrHz = 0, ampCv = 0; bool ok = false; uint16_t reasons = 0; };

  ModStat analyzeModality(bool isPpg, uint32_t tStart) {
    ModStat ms; int n = 0;
    int cnt = isPpg ? ppgCount : ecgCount, head = isPpg ? ppgHead : ecgHead;
    for (int i = 0; i < cnt; i++) {
      int idx = (head - cnt + i + 2 * kMaxBeats) % kMaxBeats;
      uint32_t c = isPpg ? ppgB[idx].cyc : ecgB[idx].cyc;
      if (c < tStart) continue;
      float fr = isPpg ? ppgB[idx].frac : ecgB[idx].frac;
      wt[n] = ((float)(int32_t)(c - tStart) + fr) / fsTime;
      wbase[n] = isPpg ? ppgB[idx].base : 0.0f;
      wamp[n]  = isPpg ? ppgB[idx].amp  : ecgB[idx].amp;
      if (++n >= kMaxBeats) break;
    }
    ms.nBeats = n;
    if (n < cfg.minBeatsWindow) { ms.reasons |= R_FEW_BEATS; return ms; }

    int nInt = 0; wok[0] = false; wint[0] = 0;
    for (int i = 1; i < n; i++) {
      wint[i] = wt[i] - wt[i - 1];
      if (wint[i] >= cfg.minBeatInterval && wint[i] <= cfg.maxBeatInterval) tmpA[nInt++] = wint[i];
    }
    if (nInt < 8) { ms.reasons |= R_FEW_BEATS; return ms; }
    float tmp2[kMaxBeats];
    float med = medianOf(tmpA, nInt, tmp2);
    for (int i = 1; i < n; i++)
      wok[i] = (wint[i] >= cfg.minBeatInterval && wint[i] <= cfg.maxBeatInterval &&
                fabsf(wint[i] - med) <= cfg.intervalTol * med);
    // amplitud: rechazo de latidos con amplitud anómala (movimiento)
    int na = 0; for (int i = 1; i < n; i++) if (wok[i]) tmpA[na++] = wamp[i];
    if (na < 8) { ms.reasons |= R_ARTIFACT; return ms; }
    float medA = medianOf(tmpA, na, tmp2);
    for (int i = 1; i < n; i++) if (wok[i] && (wamp[i] > 3.0f * medA || wamp[i] < medA / 3.0f)) wok[i] = false;
    double s = 0, s2 = 0; int nv = 0;
    for (int i = 1; i < n; i++) if (wok[i]) { s += wamp[i]; s2 += (double)wamp[i] * wamp[i]; nv++; }
    ms.nValid = nv; ms.hrBpm = 60.0f / med; ms.hrHz = 1.0f / med;
    if (nv > 1 && s > 0) { double mean = s / nv, var = s2 / nv - mean * mean; ms.ampCv = (float)(sqrt(var > 0 ? var : 0) / mean); }
    float validFrac = (float)nv / (float)(n - 1);
    if (validFrac < cfg.minValidFrac || ms.ampCv > cfg.maxAmpCv) { ms.reasons |= R_ARTIFACT; return ms; }
    ms.ok = true;
    return ms;
  }

  // =========================================================================
  //   Estimación espectral de una característica (Goertzel, 4 Hz, Hann)
  //   Devuelve false si la serie no es utilizable. bpm = FR del pico; q = pureza.
  // =========================================================================
  bool estimateFeature(int m, float hrHz, float& bpm, float& q, uint16_t& why) {
    if (m < 12) { why |= R_FEW_BEATS; return false; }
    float t0 = ft[0], span = ft[m - 1] - ft[0];
    if (span < 24.0f) { why |= R_ARTIFACT; return false; }
    for (int i = 1; i < m; i++) if (ft[i] - ft[i - 1] > 4.0f) { why |= R_ARTIFACT; return false; }  // hueco > 4 s
    int N = (int)(span * kGridHz) + 1; if (N > kGridMax) N = kGridMax;
    int j = 0;
    for (int i = 0; i < N; i++) {                                   // interpolación lineal a 4 Hz
      float tg = t0 + (float)i / kGridHz;
      while (j < m - 2 && ft[j + 1] < tg) j++;
      float dt = ft[j + 1] - ft[j], u = (dt > 0) ? (tg - ft[j]) / dt : 0; if (u < 0) u = 0; if (u > 1) u = 1;
      gx[i] = fv[j] + u * (fv[j + 1] - fv[j]);
    }
    double sx = 0, sy = 0, sxx = 0, sxy = 0;                        // quitar tendencia lineal
    for (int i = 0; i < N; i++) { sx += i; sy += gx[i]; sxx += (double)i * i; sxy += (double)i * gx[i]; }
    double dd = N * sxx - sx * sx, bb = (dd != 0) ? (N * sxy - sx * sy) / dd : 0, aa = (sy - bb * sx) / N;
    for (int i = 0; i < N; i++) {
      float w = 0.5f * (1.0f - cosf(2.0f * kPi * i / (N - 1)));     // Hann
      gx[i] = (float)(gx[i] - (aa + bb * i)) * w;
    }
    float fLow = cfg.rrMinBpm / 60.0f, fHigh = fminf(cfg.rrMaxBpm / 60.0f, cfg.nyquistFrac * hrHz);
    if (fHigh - fLow < 0.12f) { why |= R_NYQUIST; return false; }
    const float df = 0.005f; int nF = (int)((fHigh - fLow) / df) + 1; if (nF > kMaxFreqBins) nF = kMaxFreqBins;
    double tot = 0; int kp = 0; float pmax = -1;
    for (int k = 0; k < nF; k++) {
      float f = fLow + k * df, coef = 2.0f * cosf(2.0f * kPi * f / kGridHz), s1 = 0, s2 = 0;
      for (int i = 0; i < N; i++) { float s0 = gx[i] + coef * s1 - s2; s2 = s1; s1 = s0; }
      float p = s1 * s1 + s2 * s2 - coef * s1 * s2; if (p < 0) p = 0;
      pw[k] = p; tot += p; if (p > pmax) { pmax = p; kp = k; }
    }
    if (tot <= 0) { why |= R_LOW_QUALITY; return false; }
    float fk = kp;                                                  // refinamiento parabólico
    if (kp > 0 && kp < nF - 1) { float a = pw[kp - 1], b = pw[kp], c = pw[kp + 1], dn = a - 2 * b + c; if (dn != 0) fk = kp + 0.5f * (a - c) / dn; }
    float fpk = fLow + fk * df;
    double nearSum = 0; for (int k = 0; k < nF; k++) if (fabsf((fLow + k * df) - fpk) <= cfg.peakHalfWidthHz) nearSum += pw[k];
    q = (float)(nearSum / tot);
    if (fpk < fLow + cfg.edgeMarginHz || fpk > fHigh - 0.01f) q = 0.0f;  // pico pegado al borde de banda: deriva/aliasing, no es respiración
    bpm = 60.0f * fpk;
    return true;
  }

  // =========================================================================
  //                 Evaluación completa de la ventana (cada hopS)
  // =========================================================================
  void computeWindow() {
    for (int f = 0; f < kNumFeatures; f++) { res.featBpm[f] = 0; res.featQ[f] = 0; }
    res.reasons = 0; res.nAgree = 0; res.conf = 0; res.rrRaw = NAN; res.hrEcg = res.hrPpg = 0;
    uint32_t winCycles = (uint32_t)(kWindowS * fsTime + 0.5f);
    if (cyc < winCycles || (cyc - winCycles) < dataStartCyc) {
      res.status = ST_WARMUP;
      uint32_t have = (cyc > dataStartCyc) ? (cyc - dataStartCyc) : 0;
      res.warmFrac = fminf(1.0f, (float)have / (float)winCycles);
      return;
    }
    res.warmFrac = 1.0f;
    uint32_t tStart = cyc - winCycles;
    if (noSignalNow) { res.status = ST_NO_SIGNAL; res.reasons = R_NO_PPG | R_NO_ECG; return; }
    bool sat = everSat && (cyc - lastSatCyc) < winCycles;
    bool satE = everSatEcg && (cyc - lastSatEcgCyc) < winCycles;

    bool feat_ok[kNumFeatures] = {false, false, false, false, false};
    // ---------- PPG ----------
    ModStat mp = analyzeModality(true, tStart);
    res.reasons |= mp.reasons; if (sat) { res.reasons |= R_SATURATED; mp.ok = false; }
    if (mp.ok) {
      res.hrPpg = mp.hrBpm;
      for (int f = F_PPG_RIIV; f <= F_PPG_RIFV; f++) {
        int m = 0;
        for (int i = 1; i < mp.nBeats; i++) if (wok[i]) {
          ft[m] = wt[i];
          fv[m] = (f == F_PPG_RIIV) ? wbase[i] : (f == F_PPG_RIAV) ? wamp[i] : wint[i];
          m++;
        }
        feat_ok[f] = estimateFeature(m, mp.hrHz, res.featBpm[f], res.featQ[f], res.reasons);
      }
    } else if (!(res.reasons & (R_FEW_BEATS | R_ARTIFACT | R_SATURATED))) res.reasons |= R_NO_PPG;
    // ---------- ECG ----------
    ModStat me = analyzeModality(false, tStart);
    res.reasons |= me.reasons; if (satE) { res.reasons |= R_SATURATED; me.ok = false; }
    if (me.ok) {
      res.hrEcg = me.hrBpm;
      for (int f = F_ECG_RIFV; f <= F_ECG_EDR; f++) {
        int m = 0;
        for (int i = 1; i < me.nBeats; i++) if (wok[i]) { ft[m] = wt[i]; fv[m] = (f == F_ECG_RIFV) ? wint[i] : wamp[i]; m++; }
        feat_ok[f] = estimateFeature(m, me.hrHz, res.featBpm[f], res.featQ[f], res.reasons);
      }
    } else if (!(me.reasons & (R_FEW_BEATS | R_ARTIFACT))) res.reasons |= R_NO_ECG;

    // ---------- Fusión ----------
    int cand[kNumFeatures], nc = 0;
    for (int f = 0; f < kNumFeatures; f++) if (feat_ok[f] && res.featQ[f] >= cfg.minPeakQuality) cand[nc++] = f;
    float rr = 0, meanQ = 0; int bestCnt = 0; bool hasP = false, hasE = false;
    if (nc >= 2) {
      float bestSumQ = -1; int bestMem[kNumFeatures];
      for (int a = 0; a < nc; a++) {
        int mem[kNumFeatures], c = 0; float sq = 0;
        for (int b = 0; b < nc; b++) if (fabsf(res.featBpm[cand[b]] - res.featBpm[cand[a]]) <= cfg.clusterTolBpm) { mem[c++] = cand[b]; sq += res.featQ[cand[b]]; }
        if (c > bestCnt || (c == bestCnt && sq > bestSumQ)) { bestCnt = c; bestSumQ = sq; memcpy(bestMem, mem, sizeof(int) * c); }
      }
      if (bestCnt >= 2) {
        double sw = 0, swb = 0;
        for (int i = 0; i < bestCnt; i++) { int f = bestMem[i]; sw += res.featQ[f]; swb += res.featQ[f] * res.featBpm[f]; if (f <= F_PPG_RIFV) hasP = true; else hasE = true; }
        rr = (float)(swb / sw); meanQ = (float)(sw / bestCnt);
        if (!(hasP && hasE)) {                                   // un solo sensor: exigir más calidad
          for (int i = 0; i < bestCnt; i++) if (res.featQ[bestMem[i]] < cfg.minQualitySingle) { bestCnt = 0; break; }
        }
      }
    }
    if (bestCnt >= 2) {
      res.rrRaw = rr; res.nAgree = (uint8_t)bestCnt;
      res.conf = fminf(1.0f, meanQ * (0.5f + 0.1f * bestCnt));
      bool fresh = acceptEstimate(rr);
      if (fresh) res.status = (hasP && hasE) ? ST_OK_HIGH : ST_OK_MEDIUM;
      else { res.reasons |= R_UNCONFIRMED; res.status = (sm.have && (float)(cyc - sm.lastAcceptCyc) / fsTime <= cfg.holdS) ? ST_HELD : ST_UNRELIABLE; }
    } else {
      res.reasons |= (nc == 0) ? R_LOW_QUALITY : R_NO_AGREEMENT;
      if (!sm.have) sm.pendCnt = 0;                                // la confirmación exige ventanas consecutivas
      res.status = (sm.have && (float)(cyc - sm.lastAcceptCyc) / fsTime <= cfg.holdS) ? ST_HELD : ST_UNRELIABLE;
    }
  }

  // Suavizado: mediana de 3 + EMA, con límite de salto que exige confirmación.
  // Devuelve true si el valor mostrado se actualizó con esta ventana.
  bool acceptEstimate(float rr) {
    if (sm.nh < 3) sm.hist[sm.nh++] = rr; else { sm.hist[0] = sm.hist[1]; sm.hist[1] = sm.hist[2]; sm.hist[2] = rr; }
    float med;
    if (sm.nh == 1) med = sm.hist[0]; else if (sm.nh == 2) med = 0.5f * (sm.hist[0] + sm.hist[1]);
    else { float a = sm.hist[0], b = sm.hist[1], c = sm.hist[2]; med = fmaxf(fminf(a, b), fminf(fmaxf(a, b), c)); }
    if (!sm.have) {                                              // primera FR: exige 2 ventanas consecutivas coincidentes
      if (sm.pendCnt >= 1 && fabsf(rr - sm.pendVal) <= cfg.clusterTolBpm) {
        sm.out = 0.5f * (rr + sm.pendVal); sm.have = true; sm.lastAcceptCyc = cyc; sm.pendCnt = 0; sm.hist[0] = sm.out; sm.nh = 1;
        return true;
      }
      sm.pendVal = rr; sm.pendCnt = 1; sm.nh = 0; return false;
    }
    if (fabsf(med - sm.out) > cfg.maxJumpBpm) {
      if (sm.pendCnt >= 1 && fabsf(rr - sm.pendVal) <= 3.0f) {      // 2 estimaciones consecutivas coinciden: aceptar salto
        sm.out = rr; sm.hist[0] = rr; sm.nh = 1; sm.pendCnt = 0; sm.lastAcceptCyc = cyc;
        return true;
      } else { sm.pendVal = rr; sm.pendCnt = 1; return false; }    // esperar confirmación (la edad sigue creciendo)
    }
    sm.out += cfg.emaAlpha * (med - sm.out); sm.pendCnt = 0; sm.lastAcceptCyc = cyc;
    return true;
  }
};

}  // namespace RespRate
#endif  // RESP_RATE_ESTIMATOR_H
