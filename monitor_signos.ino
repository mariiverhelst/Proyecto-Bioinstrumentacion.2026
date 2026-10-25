/*
 * AFE4950 + ESP32 -> NEXTION  (versión FreeRTOS)
 * ECG + PPG + HR + PR + SpO2 + PRESIÓN ARTERIAL (PTT + PIR) + BUZZER
 * + FRECUENCIA RESPIRATORIA ESTIMADA (ECG + PPG)
 * + ALARMAS NEWS2 + ALARMAS TÉCNICAS
 *
 * Núcleo 1 (tiempo real):
 *   T_adq    prio 5 : única dueña del AFE4950. Lee el FIFO, decodifica y encola.
 *   T_proc   prio 3 : filtros, Pan-Tompkins, FC/PR, lead-off, PA (PTT+PIR), FR, gráfica.
 *   T_sonido prio 2 : única dueña del buzzer.
 * Núcleo 0 (comunicaciones):
 *   T_display prio 2: única dueña de Serial (= Nextion). Lee "CAL <SBP> <DBP>".
 *   T_alarma  prio 1: NEWS2 + alarmas técnicas cada 1 s con persistencia.
 *
 * Por Serial solo se envían comandos Nextion (no usar Serial.print).
 */

#include <Arduino.h>
#include <AFE4950.h>
#include "biosignals.h"
#include "RespRateEstimator.h"   // [FR]

// ---------------- PINES AFE4950 ----------------
#define SCLK      18
#define MISO      19
#define MOSI      23
#define CS         5
#define AFE_RST    4
#define FIFO_RDY  16

// ---------------- BUZZER PASIVO ----------------
#define PIN_BUZZER          25
#define BUZZER_DURACION_MS  40
#define BUZZER_FREQ_HZ      2000   // bip de pulso (no es alarma)

// ---------------- ALARMAS ----------------
#define ALARMA_FREQ_HZ      540    // fundamental 150-1000 Hz; armónicos 1620, 2700, 3780
#define ESCALAR_SEG         5      // segundos sostenidos para SUBIR de nivel
#define DESESCALAR_SEG      10     // segundos sostenidos para BAJAR de nivel
#define TECNICA_SEG         3      // segundos sostenidos para alarma técnica

#define SIMULAR_NEWS2       0      // 1 = valores simulados que recorren todos los niveles
#define USAR_PAS_EN_NEWS2   0      // 1 = incluir la PAS por PTT en el NEWS2
#define USAR_FR_EN_NEWS2    0      // [FR] 1 = incluir la FR estimada en el NEWS2 (validar antes)

// ---------------- FC / PR ----------------
#define TIMEOUT_LATIDO_MS   4000   // sin latidos en este tiempo -> FC/PR = 0

// ---------------- LEAD-OFF (electrodo desconectado, por software) ----------------
#define LOFF_HABILITADO     0      // 1 cuando los umbrales estén calibrados
#define CALIBRAR_LOFF       0      // 1 = PR muestra pico a pico (µV), SpO2 muestra promedio (mV)
#define LOFF_PP_MIN_V       0.00001f  // AJUSTAR: menos que esto = señal plana
#define LOFF_PP_MAX_V       0.5f      // AJUSTAR: más que esto = ruido de electrodo suelto
#define LOFF_SAT_V          1.0f      // AJUSTAR: |promedio| mayor = saturado

// ---------------- [FR] FRECUENCIA RESPIRATORIA ESTIMADA ----------------
#define NEX_OBJ_FR           "nFR"     // Number en la Nextion (0 = sin valor)
#define NEX_OBJ_FR_TXT       "tFRest"  // Text opcional con el estado de la estimación
#define USAR_TEXTO_ESTADO_FR 0         // 1 si agregas el Text tFRest en la Nextion
#define FR_CAL_MS            5000UL    // se mide la tasa real de ECG durante 5 s
#define DEBUG_FR_SERIAL1     0         // 1 = detalle de cada ventana por Serial1 (TX en GPIO33)
#define DEBUG_TX_PIN         33

#define FR_EST_CAL_RELOJ     10        // códigos extra de estado (además de RespRate::Status)
#define FR_EST_CONFIG        11

// ---------------- PRESIÓN ARTERIAL (PTT + PIR) ----------------
#define FS_PA        500      // Hz (sincronizado con el ECG a 500 Hz)
#define USAR_PIR     1        // 1 = usa PTT + PIR, 0 = usa solo PTT
#define SBP_DEF      120.0f   // Sistólica base de calibración (mmHg)
#define DBP_DEF      80.0f    // Diastólica base de calibración (mmHg)

#define N_BUF        2048     // ~4 s de historia a 500 Hz (potencia de 2)
#define MS(x)        ((uint32_t)((x) * FS_PA / 1000))
#define B(buf, i)    buf[(i) % N_BUF]

#define NPROM        8        // latidos a promediar
#define N_CAL        5        // latidos de calibración

// ---------------- NEXTION ----------------
#define NEX_BAUD    115200
#define ID_WAVE_ECG 1
#define ID_WAVE_PPG 2
#define CANAL       0
#define ALTO_WAVE   199

#define INVERTIR_ECG true
#define INVERTIR_PPG false

// ---------------- FRECUENCIAS DE MUESTREO ----------------
#define FS_ECG_HZ 500.0f
#define FS_PPG_HZ 250.0f

static const uint32_t PERIODO_ECG_US = 2000;
static const uint32_t PERIODO_PPG_US = 4000;

static const float FACTOR_VOLT = 1.2f / 2097151.0f;

static const uint32_t PERIODO_GRAFICA_MS = 10;
static const uint32_t PERIODO_INFO_MS    = 1000;

// Decimación para la gráfica (100 puntos/s por onda)
#define DECIM_ECG     5        // 500 Hz -> 100 Hz
#define PPG_PASO      2        // 250 Hz * 2/5 = 100 Hz
#define PPG_DIVISOR   5

// Filtros visuales
#define ECG_FC_HZ    40.0f
#define ECG_HP_FC_HZ 0.5f
#define ECG_Q        0.7071f

#define PPG_FC_HZ    8.0f
#define PPG_HP_FC_HZ 0.5f
#define PPG_Q        0.7071f

// Pan-Tompkins PPG
#define PT_HP_FC_PPG 0.7f
#define PT_LP_FC_PPG 5.0f
#define PT_Q_PPG     0.7071f

#define PT_CUENTAS_POR_V_PPG 20000.0f
#define PT_DC_FC_HZ          0.3f
#define PT_CUENTAS_POR_V_ECG 100000.0f

static const float SPAN_MIN_ECG_V = 0.0002f;
static const float SPAN_MIN_PPG_V = 0.0020f;


// ============================================================
// TIPOS (TODOS aquí arriba, antes de cualquier función)
// ============================================================

enum TipoMsg : uint8_t {
  MSG_ECG = 0,        // v = voltios, a = 1 si la muestra está saturada  [FR]
  MSG_PPG,            // v = voltios, a = 1 si la muestra está saturada  [FR]
  MSG_ESTADO_SPO2,    // a = nuevo estado del sensor (0 = sin dedo)
  MSG_SPO2,           // a = SpO2 calculada (%)
  MSG_PIR             // v = PIR del procesador SpO2 (0 = no válido)
};

struct Msg {                 // T_adq -> T_proc
  TipoMsg tipo;
  uint8_t a;
  float   v;
};

struct PuntoGraf {           // T_proc -> T_display
  uint8_t id;
  float   v;
};

struct CalPA {               // T_display -> T_proc
  float sbp;
  float dbp;
};

struct Signos {              // T_proc -> T_display / T_sonido / T_alarma (mailbox)
  int     fc;                // lpm ECG (0 = sin dato)
  int     pr;                // lpm PPG (0 = sin dato)
  int     spo2;              // %
  uint8_t estadoSpo2;        // 0 = sin dedo, 1 = estabilizando, 2 = válida
  int     fr;                // rpm   (-1 = no disponible)
  float   temp;              // °C    (-1 = no disponible todavía)
  int     pas;               // mmHg para NEWS2 (-1 = no disponible)
  int     sbp;               // mmHg para pantalla (0 = sin dato)
  int     dbp;               // mmHg para pantalla (0 = sin dato)
  bool    ecgDesconectado;   // lead-off por software
  int     calPpUv;           // calibración lead-off: pico a pico (µV)
  int     calMediaMv;        // calibración lead-off: promedio (mV)
  uint8_t frEstado;          // [FR] RespRate::Status o FR_EST_CAL_RELOJ / FR_EST_CONFIG
};

struct EscalaAutomatica {
  float maximo;
  float bloqueMaximo;
  uint16_t cantidad;
  bool inicializada;
};

enum NivelAlarma : uint8_t { ALARMA_BAJA = 0, ALARMA_MEDIA = 1, ALARMA_ALTA = 2 };

struct EntradaNews2 {        // -1 = parámetro no disponible (no se puntúa)
  int   fr;
  int   spo2;
  bool  o2Suplementario;
  int   pas;
  int   pulso;
  bool  alerta;
  float temp;
  bool  o2Conocido;
  bool  concienciaConocida;
};

struct EstadoAlarma {        // T_alarma -> T_sonido / T_display (mailbox)
  uint8_t     news2;
  NivelAlarma nivel;         // filtrado por persistencia
  bool        algunTres;
  uint8_t     parametros;    // cuántos de los 7 se puntuaron
  bool        tecEcg;        // alarma técnica: electrodo desconectado
  bool        tecSpo2;       // alarma técnica: sensor SpO2 retirado
};

enum ModoSonido : uint8_t { SON_PULSO = 0, SON_TECNICA, SON_MEDIA, SON_ALTA };

struct Paso {
  uint16_t freq;             // Hz (0 = silencio)
  uint16_t ms;
};


// ============================================================
// OBJETOS FreeRTOS
// ============================================================

QueueHandle_t colaMsg;
QueueHandle_t colaGraf;
QueueHandle_t colaCal;
QueueHandle_t buzonSignos;
QueueHandle_t buzonAlarma;

portMUX_TYPE muxCont = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t perdidas = 0;

AFE4950 afe;


// ============================================================
// ESTADO DE PRESIÓN ARTERIAL  (SOLO lo toca T_proc)
// Globales para no gastar ~24 KB del stack de la tarea.
// ============================================================

static float    ecgBuf[N_BUF];
static float    ppgBuf[N_BUF];
static float    dcBuf[N_BUF];
static uint32_t nMuestrasPA = 0;

static uint32_t Rprev = 0;
static float hPtt[NPROM], hPir[NPROM];
static int   nProm = 0, iProm = 0;
static float pttProm = 0.0f, pirProm = 1.02f;

static bool  calibrando = true;
static bool  calibrado  = false;
static float SBP0 = SBP_DEF, DBP0 = DBP_DEF;
static float PTT0 = 260.0f, PIR0 = 1.02f;   // semillas para respuesta inmediata
static float sumPtt = 0.0f, sumPir = 0.0f;
static int   nCal = 0;

static int sbpEstable = 0;
static int dbpEstable = 0;


// ============================================================
// [FR] ESTADO DE FRECUENCIA RESPIRATORIA  (SOLO lo toca T_proc)
// Global: el estimador ocupa ~13 KB, no cabe en el stack de la tarea.
// ============================================================

static RespRate::Estimator frEst;
static bool     frCalibrando    = true;
static bool     frDeshabilitado = false;
static uint32_t frCalInicioUs = 0, frCalMuestras = 0;
static uint32_t frTbInicioUs  = 0, frTbMuestras  = 0;


// ============================================================
// PATRONES SONOROS
// ============================================================

const Paso PATRON_ALTA[] = {          // 3 + 2 pulsos rápidos, 2 s de silencio
  {ALARMA_FREQ_HZ, 150}, {0, 100}, {ALARMA_FREQ_HZ, 150}, {0, 100}, {ALARMA_FREQ_HZ, 150}, {0, 350},
  {ALARMA_FREQ_HZ, 150}, {0, 100}, {ALARMA_FREQ_HZ, 150}, {0, 2000}
};

const Paso PATRON_MEDIA[] = {         // 3 pulsos, 5 s de silencio
  {ALARMA_FREQ_HZ, 200}, {0, 200}, {ALARMA_FREQ_HZ, 200}, {0, 200}, {ALARMA_FREQ_HZ, 200}, {0, 5000}
};

const Paso PATRON_TECNICA[] = {       // 2 pulsos cortos, 10 s de silencio
  {ALARMA_FREQ_HZ, 100}, {0, 150}, {ALARMA_FREQ_HZ, 100}, {0, 10000}
};


// ============================================================
// UTILIDADES DE FILTROS
// ============================================================

void biquadPrime(Biquad* filtro, float x0) {
  filtro->z2 = (filtro->b2 - filtro->a2) * x0;
  filtro->z1 = (filtro->b1 - filtro->a1 + filtro->b2 - filtro->a2) * x0;
}

void biquadPrimeHP(Biquad* filtro, float x0) {
  filtro->z2 = filtro->b2 * x0;
  filtro->z1 = (filtro->b1 + filtro->b2) * x0;
}


// ============================================================
// NEXTION
// ============================================================

void nexEnviar(char* buffer, int longitud) {
  buffer[longitud++] = 0xFF;
  buffer[longitud++] = 0xFF;
  buffer[longitud++] = 0xFF;
  Serial.write((const uint8_t*)buffer, longitud);
}

void nexComando(const char* comando) {
  char buffer[48];
  int longitud = snprintf(buffer, sizeof(buffer) - 3, "%s", comando);
  nexEnviar(buffer, longitud);
}

void nexAdd(uint8_t id, uint8_t valor) {
  char buffer[32];
  int longitud = snprintf(buffer, sizeof(buffer) - 3, "add %u,%u,%u", id, CANAL, valor);
  nexEnviar(buffer, longitud);
}

void nexNumero(const char* objeto, int valor) {
  char buffer[40];
  int longitud = snprintf(buffer, sizeof(buffer) - 3, "%s.val=%d", objeto, valor);
  nexEnviar(buffer, longitud);
}


// ============================================================
// AUTOESCALADO (solo visual)
// ============================================================

uint8_t escalarSenal(EscalaAutomatica& escala, float valor, float spanMinimo, bool invertir) {
  float a = fabsf(valor);

  if (a > escala.bloqueMaximo) escala.bloqueMaximo = a;
  escala.cantidad++;

  if (!escala.inicializada) {
    if (escala.cantidad >= 100) {
      escala.maximo = escala.bloqueMaximo;
      escala.inicializada = true;
      escala.bloqueMaximo = 0.0f;
      escala.cantidad = 0;
    }
    return ALTO_WAVE / 2;
  }

  if (a > escala.maximo) escala.maximo = a;

  if (escala.cantidad >= 300) {
    escala.maximo += 0.35f * (escala.bloqueMaximo - escala.maximo);
    escala.bloqueMaximo = 0.0f;
    escala.cantidad = 0;
  }

  float amp = escala.maximo;
  if (amp < 0.5f * spanMinimo) amp = 0.5f * spanMinimo;
  amp *= 1.1f;

  int salida = (int)(ALTO_WAVE * 0.5f + (valor / amp) * ALTO_WAVE * 0.5f + 0.5f);
  salida = constrain(salida, 0, ALTO_WAVE);
  if (invertir) salida = ALTO_WAVE - salida;

  return (uint8_t)salida;
}


// ============================================================
// PRESIÓN ARTERIAL (pasos 4 a 8)  — llamadas solo desde T_proc
// ============================================================

void revisarYPromediar(float ptt, float pir);
void acumularCalibracion(float ptt, float pir);
void calcularPA(float ptt, float pir);

// ---------- PASO 7: calibración ----------
void iniciarCalibracion(float sbp, float dbp) {
  SBP0 = sbp;
  DBP0 = dbp;
  sumPtt = 0.0f;
  sumPir = 0.0f;
  nCal = 0;
  calibrado = false;
  calibrando = true;
}

void acumularCalibracion(float ptt, float pir) {
  sumPtt += ptt;
  sumPir += pir;
  nCal++;
  PTT0 = sumPtt / nCal;      // se actualiza progresivamente
  PIR0 = sumPir / nCal;
  if (nCal >= N_CAL) {
    calibrando = false;
    calibrado = true;
  }
}

// ---------- PASO 5: PTT y PIR de un latido ----------
void procesarLatido(uint32_t R, uint32_t Rsig, float ppgDC, float pirSpo2) {
  if (Rsig <= R || (Rsig - R) >= N_BUF - 10) return;

  float rr = (Rsig - R) * 1000.0f / FS_PA;
  if (rr < 300.0f || rr > 2200.0f) return;

  // 5a. Frente más pronunciado del PPG en [R + 80 ms, R + 600 ms]
  uint32_t iniBusqueda = R + MS(80);
  uint32_t fin = R + MS(600);
  if (fin > Rsig) fin = Rsig;
  if (iniBusqueda + 6 >= fin) return;

  uint32_t kSubida = iniBusqueda, kBajada = iniBusqueda;
  float mejorSubida = -1e9f, mejorBajada = -1e9f;

  for (uint32_t i = iniBusqueda; i + 5 < fin; i++) {
    float pend = B(ppgBuf, i + 5) - B(ppgBuf, i - 5);
    if (pend > mejorSubida)  { mejorSubida = pend;  kSubida = i; }
    if (-pend > mejorBajada) { mejorBajada = -pend; kBajada = i; }
  }

  uint32_t k  = (mejorSubida >= mejorBajada) ? kSubida : kBajada;
  float mejor = (mejorSubida >= mejorBajada) ? mejorSubida : mejorBajada;
  if (mejor <= 1e-7f) return;

  // 5b. PTT en ms
  float ptt = (k - R) * 1000.0f / FS_PA;

  // 5c. PIR
  float pir = 1.02f;
#if USAR_PIR
  if (pirSpo2 > 0.0f) {
    pir = pirSpo2;                         // calculado en T_adq con acIr / dcIr
  } else {
    float maxPpg = -1e9f, minPpg = 1e9f;
    uint32_t finL = k + MS(300);
    if (finL > Rsig) finL = Rsig;
    for (uint32_t i = R; i <= finL; i++) {
      float v = B(dcBuf, i);
      if (v > maxPpg) maxPpg = v;
      if (v < minPpg) minPpg = v;
    }
    float ampPpg = maxPpg - minPpg;
    float baseDC = fabsf(ppgDC) + 0.05f;
    pir = 1.0f + constrain(ampPpg / baseDC, 0.005f, 0.15f);
  }
  pir = constrain(pir, 1.002f, 1.25f);
#endif

  revisarYPromediar(ptt, pir);
}

// ---------- PASO 4: ubicar el pico R ----------
void nuevoLatidoPA(uint32_t m, float ppgDC, float pirSpo2) {
  if (m < MS(500)) return;
  uint32_t ini = m - MS(400);
  if (Rprev != 0 && ini < Rprev + MS(250)) ini = Rprev + MS(250);

  uint32_t R = ini;
  for (uint32_t i = ini; i <= m; i++) {
    if (B(ecgBuf, i) > B(ecgBuf, R)) R = i;
  }

  if (Rprev != 0) procesarLatido(Rprev, R, ppgDC, pirSpo2);
  Rprev = R;
}

// ---------- PASO 6: revisar y promediar ----------
void revisarYPromediar(float ptt, float pir) {
  if (ptt < 80.0f || ptt > 550.0f) return;
  if (nProm == NPROM && fabsf(ptt - pttProm) > 0.35f * pttProm) return;

  hPtt[iProm] = ptt;
  hPir[iProm] = pir;
  iProm = (iProm + 1) % NPROM;
  if (nProm < NPROM) nProm++;

  float sP = 0.0f, sI = 0.0f;
  for (int j = 0; j < nProm; j++) { sP += hPtt[j]; sI += hPir[j]; }
  pttProm = sP / nProm;
  pirProm = sI / nProm;

  if (calibrando) acumularCalibracion(pttProm, pirProm);

  calcularPA(pttProm, pirProm);
}

// ---------- PASO 8: modelo de presión (Ding et al., 2017) ----------
void calcularPA(float ptt, float pir) {
  if (ptt <= 1.0f || pir <= 0.0f || PTT0 <= 1.0f || PIR0 <= 0.0f) return;

  float PP0  = SBP0 - DBP0;
  float MBP0 = DBP0 + PP0 / 3.0f;

  float r   = constrain(PTT0 / ptt, 0.65f, 1.55f);
  float mbp = MBP0 * (PIR0 / pir);
  float pp  = PP0 * (pir / PIR0) * r * r;

  float sbp = mbp + 2.0f * pp / 3.0f;
  float dbp = mbp - pp / 3.0f;

  sbp = constrain(sbp, 75.0f, 210.0f);
  dbp = constrain(dbp, 45.0f, 140.0f);

  sbpEstable = (int)(sbp + 0.5f);
  dbpEstable = (int)(dbp + 0.5f);
}


// ============================================================
// [FR] FRECUENCIA RESPIRATORIA — llamadas solo desde T_proc
// Se alimenta con la muestra CRUDA (cuentas del ADC), antes de los
// filtros visuales: el pasa-altos de 0.5 Hz de la gráfica no la afecta.
// ============================================================

// Mide la tasa real de ECG (el AFE usa reloj interno) y luego alimenta el estimador.
void frAlimentarEcg(float cuentas, bool saturada) {
  if (frDeshabilitado) return;

  if (frCalibrando) {
    if (frCalMuestras == 0) frCalInicioUs = micros();
    frCalMuestras++;
    uint32_t dt = micros() - frCalInicioUs;
    if (dt >= FR_CAL_MS * 1000UL) {
      float fsMedida = (float)frCalMuestras * 1.0e6f / (float)dt;
      if (fsMedida < 0.8f * FS_ECG_HZ || fsMedida > 1.25f * FS_ECG_HZ) {
        frDeshabilitado = true;     // el AFE no está en 500 Hz / ECG,ECG,PPG
        return;
      }
      frEst.begin(fsMedida, 2);     // 2 muestras de ECG por cada muestra de PPG
      frCalibrando = false;
      frTbInicioUs = micros();
      frTbMuestras = 0;
    }
    return;
  }

  frTbMuestras++;
  bool nuevaVentana = frEst.pushEcg(cuentas, saturada);

  // Cada 20 s se corrige la deriva lenta del reloj del AFE
  uint32_t dt = micros() - frTbInicioUs;
  if (dt >= 20000000UL) {
    frEst.setTimeBase((float)frTbMuestras * 1.0e6f / (float)dt);
    frTbInicioUs = micros();
    frTbMuestras = 0;
  }

#if DEBUG_FR_SERIAL1
  if (nuevaVentana) {
    const RespRate::Result& w = frEst.lastWindow();
    char b[140];
    snprintf(b, sizeof(b),
             "FR st=%d raw=%.1f f=[%.1f %.1f %.1f %.1f %.1f] q=[%.2f %.2f %.2f %.2f %.2f] mot=0x%X\r\n",
             (int)w.status, w.rrRaw,
             w.featBpm[0], w.featBpm[1], w.featBpm[2], w.featBpm[3], w.featBpm[4],
             w.featQ[0], w.featQ[1], w.featQ[2], w.featQ[3], w.featQ[4], (unsigned)w.reasons);
    Serial1.print(b);
  }
#else
  (void)nuevaVentana;
#endif
}

void frAlimentarPpg(float cuentas, bool saturada) {
  if (!frCalibrando && !frDeshabilitado) frEst.pushPpg(cuentas, saturada);
}

// Llena s.fr (-1 = sin valor) y s.frEstado. Llamar una vez por segundo.
void frActualizarSignos(Signos& s) {
  if (frDeshabilitado) { s.fr = -1; s.frEstado = FR_EST_CONFIG;    return; }
  if (frCalibrando)    { s.fr = -1; s.frEstado = FR_EST_CAL_RELOJ; return; }

  RespRate::Result r = frEst.snapshot();
  s.frEstado = (uint8_t)r.status;
  s.fr = r.hasValue ? (int)(r.rrBpm + 0.5f) : -1;
}

const char* textoEstadoFR(uint8_t e) {
  switch (e) {
    case FR_EST_CAL_RELOJ:        return "calentando";
    case FR_EST_CONFIG:           return "config?";
    case RespRate::ST_WARMUP:     return "calentando";
    case RespRate::ST_OK_HIGH:    return "estimada";     // ECG y PPG coinciden
    case RespRate::ST_OK_MEDIUM:  return "est 1 sens";   // coincide un solo sensor
    case RespRate::ST_HELD:       return "retenida";     // ventana actual no fiable
    case RespRate::ST_UNRELIABLE: return "no fiable";
    default:                      return "sin senal";
  }
}


// ============================================================
// NEWS2 (escala 1 de SpO2). Cada función devuelve 0-3, o -1 si no hay dato.
// ============================================================

int puntajeFR(int fr) {
  if (fr < 0)   return -1;
  if (fr <= 8)  return 3;
  if (fr <= 11) return 1;
  if (fr <= 20) return 0;
  if (fr <= 24) return 2;
  return 3;
}

int puntajeSpO2(int spo2) {
  if (spo2 <= 0)  return -1;
  if (spo2 <= 91) return 3;
  if (spo2 <= 93) return 2;
  if (spo2 <= 95) return 1;
  return 0;
}

int puntajePAS(int pas) {
  if (pas <= 0)   return -1;
  if (pas <= 90)  return 3;
  if (pas <= 100) return 2;
  if (pas <= 110) return 1;
  if (pas <= 219) return 0;
  return 3;
}

int puntajePulso(int p) {
  if (p <= 0)   return -1;
  if (p <= 40)  return 3;
  if (p <= 50)  return 1;
  if (p <= 90)  return 0;
  if (p <= 110) return 1;
  if (p <= 130) return 2;
  return 3;
}

int puntajeTemp(float t) {
  if (t < 0.0f)   return -1;
  if (t <= 35.0f) return 3;
  if (t <= 36.0f) return 1;
  if (t <= 38.0f) return 0;
  if (t <= 39.0f) return 1;
  return 2;
}

EstadoAlarma calcularNews2(const EntradaNews2& e) {
  int p[7] = {
    puntajeFR(e.fr),
    puntajeSpO2(e.spo2),
    e.o2Conocido ? (e.o2Suplementario ? 2 : 0) : -1,
    puntajePAS(e.pas),
    puntajePulso(e.pulso),
    e.concienciaConocida ? (e.alerta ? 0 : 3) : -1,
    puntajeTemp(e.temp)
  };

  EstadoAlarma a = {0, ALARMA_BAJA, false, 0, false, false};
  for (int i = 0; i < 7; i++) {
    if (p[i] < 0) continue;
    a.news2 += p[i];
    a.parametros++;
    if (p[i] == 3) a.algunTres = true;
  }

  if (a.news2 >= 7)                     a.nivel = ALARMA_ALTA;
  else if (a.news2 >= 5 || a.algunTres) a.nivel = ALARMA_MEDIA;
  else                                  a.nivel = ALARMA_BAJA;
  return a;
}


// ============================================================
// T_adq  (núcleo 1, prio 5)
// ============================================================

void encolar(TipoMsg tipo, uint8_t a, float v) {
  Msg m = {tipo, a, v};
  if (xQueueSendToBack(colaMsg, &m, 0) != pdPASS) {
    portENTER_CRITICAL(&muxCont); perdidas++; portEXIT_CRITICAL(&muxCont);
  }
}

void tareaAdq(void* arg) {
  // Ciclo continuo ECG, ECG, PPG (igual que _fifoSlot de la librería)
  uint8_t  cicloIdx   = 0;
  uint8_t  spo2Estado = 0;
  uint32_t acIr = 0, dcIr = 0, acRed = 0, dcRed = 0;

  for (;;) {
    if (!afe.ready()) {
      vTaskDelay(1);
      continue;
    }

    const uint8_t* datos = afe.getData();
    size_t longitud = afe.getBytesToSend();

    for (size_t i = 0; i + 4 <= longitud; i += 4) {
      uint32_t valor24 = (uint32_t)datos[i] |
                         ((uint32_t)datos[i + 1] << 8) |
                         ((uint32_t)datos[i + 2] << 16);
      uint8_t tag = datos[i + 3];

      switch (tag) {
        case 0x02:
        case 0x03:
          break;

        case 0x05: {
          int32_t raw = (int32_t)valor24;
          if (raw & 0x00800000L) raw -= 0x01000000L;
          float voltios = raw * FACTOR_VOLT;

          // [FR] Saturación según el manual del AFE: |raw| fuera de +-2^21
          uint8_t sat = ((raw >= 2097152L) || (raw < -2097152L)) ? 1 : 0;

          if (cicloIdx == 2) encolar(MSG_PPG, sat, voltios);
          else               encolar(MSG_ECG, sat, voltios);

          cicloIdx = (cicloIdx + 1) % 3;
          break;
        }

        case 0x06: {
          uint8_t nuevo = datos[i];
          if (nuevo != spo2Estado) encolar(MSG_ESTADO_SPO2, nuevo, 0.0f);
          spo2Estado = nuevo;
          break;
        }

        case 0x07: acIr  = valor24; break;
        case 0x08: dcIr  = valor24; break;
        case 0x09: acRed = valor24; break;
        case 0x0A: {
          dcRed = valor24;

          // PIR físico: I_max / I_min = (DC + AC/2) / (DC - AC/2)
          float pir = 0.0f;
          if (dcIr > 100 && acIr > 0 && acIr < dcIr) {
            pir = ((float)dcIr + 0.5f * (float)acIr) /
                  ((float)dcIr - 0.5f * (float)acIr);
          }
          encolar(MSG_PIR, 0, pir);

          int spo2 = 0;
          if (spo2Estado == 2 && dcRed && dcIr && acIr) {
            float R = ((float)acRed / dcRed) / ((float)acIr / dcIr);
            float s = 110.0f - 25.0f * R;
            if (s < 0.0f)   s = 0.0f;
            if (s > 100.0f) s = 100.0f;
            spo2 = (int)(s + 0.5f);
          }
          encolar(MSG_SPO2, (uint8_t)spo2, 0.0f);
          break;
        }
      }
    }
  }
}


// ============================================================
// T_proc  (núcleo 1, prio 3)
// ============================================================

void tareaProc(void* arg) {
  PanTompkins ptEcg, ptPpg;
  Biquad ecgLP1, ecgLP2, ecgHP;
  Biquad ppgLP1, ppgLP2, ppgHP;

  biquad_lowpass (&ecgLP1, FS_ECG_HZ, ECG_FC_HZ,    ECG_Q);
  biquad_lowpass (&ecgLP2, FS_ECG_HZ, ECG_FC_HZ,    ECG_Q);
  biquad_highpass(&ecgHP,  FS_ECG_HZ, ECG_HP_FC_HZ, ECG_Q);

  biquad_lowpass (&ppgLP1, FS_PPG_HZ, PPG_FC_HZ,    PPG_Q);
  biquad_lowpass (&ppgLP2, FS_PPG_HZ, PPG_FC_HZ,    PPG_Q);
  biquad_highpass(&ppgHP,  FS_PPG_HZ, PPG_HP_FC_HZ, PPG_Q);

  const float kDC = 2.0f * PI * PT_DC_FC_HZ / FS_PPG_HZ;
  float ppgDC = 0.0f;

  auto iniciarPTEcg = [&]() { pt_init(&ptEcg, FS_ECG_HZ); };
  auto iniciarPTPpg = [&]() {
    pt_init(&ptPpg, FS_PPG_HZ);
    biquad_highpass(&ptPpg.hp, FS_PPG_HZ, PT_HP_FC_PPG, PT_Q_PPG);
    biquad_lowpass (&ptPpg.lp, FS_PPG_HZ, PT_LP_FC_PPG, PT_Q_PPG);
  };
  iniciarPTEcg();
  iniciarPTPpg();

  uint32_t tiempoEcgUs = 0, tiempoPpgUs = 0;
  bool tEcgIni = false, tPpgIni = false;

  bool hayEcg = false, hayPpg = false;
  uint8_t decimEcg = 0;
  uint8_t acumPpg  = 0;

  // Últimos valores de PPG (se guardan junto a cada muestra ECG para la PA)
  float ppgGraficaUlt  = 0.0f;
  float ppgFiltradaUlt = 0.0f;
  float pirSpo2        = 0.0f;   // PIR desde acIr/dcIr (0 = no válido)

  // Último latido detectado (para que FC/PR caigan a 0)
  TickType_t tUltLatEcg = 0;
  TickType_t tUltLatPpg = 0;

  // Ventana de 1 s de la ECG cruda (lead-off por software)
  float ecgMin = 1e9f, ecgMax = -1e9f, ecgSuma = 0.0f;
  uint16_t ecgN = 0;

  auto enviarPunto = [&](uint8_t id, float v) {
    PuntoGraf p = {id, v};
    xQueueSendToBack(colaGraf, &p, 0);
  };

  Signos s = {0, 0, 0, 0, -1, -1.0f, -1, 0, 0, false, 0, 0, FR_EST_CAL_RELOJ};
  TickType_t tInfo = xTaskGetTickCount();
  Msg m;

  for (;;) {
    xQueueReceive(colaMsg, &m, portMAX_DELAY);

    switch (m.tipo) {

      case MSG_ECG: {
        float voltios = m.v;

        // [FR] muestra cruda en cuentas (antes de cualquier filtro visual)
        frAlimentarEcg(voltios / FACTOR_VOLT, m.a != 0);

        if (!hayEcg) {
          biquadPrime(&ecgLP1, voltios);
          biquadPrime(&ecgLP2, voltios);
          biquadPrimeHP(&ecgHP, voltios);
          hayEcg = true;
        }
        float filtrada = biquad_process(&ecgLP2, biquad_process(&ecgLP1, voltios));
        float grafica  = biquad_process(&ecgHP, filtrada);

        if (++decimEcg >= DECIM_ECG) {
          decimEcg = 0;
          enviarPunto(ID_WAVE_ECG, grafica);
        }

        if (!tEcgIni) { tiempoEcgUs = micros(); tEcgIni = true; }
        else          { tiempoEcgUs += PERIODO_ECG_US; }

        // ---- Buffers circulares de PA a 500 Hz (paso 2) ----
        uint32_t idx = nMuestrasPA;
        B(ecgBuf, idx) = INVERTIR_ECG ? -grafica       : grafica;
        B(ppgBuf, idx) = INVERTIR_PPG ? -ppgGraficaUlt : ppgGraficaUlt;
        B(dcBuf,  idx) = ppgFiltradaUlt;
        nMuestrasPA = idx + 1;

        PT_Result r = pt_process_t(&ptEcg, voltios * PT_CUENTAS_POR_V_ECG, tiempoEcgUs);
        if (r.beat) {
          tUltLatEcg = xTaskGetTickCount();
          if (!s.ecgDesconectado && s.estadoSpo2 != 0) {
            nuevoLatidoPA(idx, ppgDC, pirSpo2);
          }
        }

        // Ventana de lead-off (1 s = 500 muestras)
        if (voltios < ecgMin) ecgMin = voltios;
        if (voltios > ecgMax) ecgMax = voltios;
        ecgSuma += voltios;
        if (++ecgN >= (uint16_t)FS_ECG_HZ) {
          float pp    = ecgMax - ecgMin;
          float media = ecgSuma / ecgN;
          s.calPpUv    = (int)(pp * 1e6f);
          s.calMediaMv = (int)(media * 1e3f);
#if LOFF_HABILITADO
          s.ecgDesconectado = (pp < LOFF_PP_MIN_V) || (pp > LOFF_PP_MAX_V) ||
                              (fabsf(media) > LOFF_SAT_V);
#else
          s.ecgDesconectado = false;
#endif
          ecgMin = 1e9f; ecgMax = -1e9f; ecgSuma = 0.0f; ecgN = 0;
        }
        break;
      }

      case MSG_PPG: {
        float voltios = m.v;

        // [FR] muestra cruda en cuentas (conserva la línea base respiratoria)
        frAlimentarPpg(voltios / FACTOR_VOLT, m.a != 0);

        if (!hayPpg) {
          biquadPrime(&ppgLP1, voltios);
          biquadPrime(&ppgLP2, voltios);
          biquadPrimeHP(&ppgHP, voltios);
          ppgDC = voltios;
          hayPpg = true;
        }
        float filtrada = biquad_process(&ppgLP2, biquad_process(&ppgLP1, voltios));
        float grafica  = biquad_process(&ppgHP, filtrada);

        ppgGraficaUlt  = grafica;
        ppgFiltradaUlt = filtrada;

        acumPpg += PPG_PASO;
        if (acumPpg >= PPG_DIVISOR) {
          acumPpg -= PPG_DIVISOR;
          enviarPunto(ID_WAVE_PPG, grafica);
        }

        ppgDC += kDC * (filtrada - ppgDC);

        if (!tPpgIni) { tiempoPpgUs = micros(); tPpgIni = true; }
        else          { tiempoPpgUs += PERIODO_PPG_US; }

        PT_Result r = pt_process_t(&ptPpg, (filtrada - ppgDC) * PT_CUENTAS_POR_V_PPG, tiempoPpgUs);
        if (r.beat) tUltLatPpg = xTaskGetTickCount();
        break;
      }

      case MSG_ESTADO_SPO2:
        if (s.estadoSpo2 == 0 && m.a != 0) {
          iniciarPTPpg();
          tPpgIni = false;
          s.pr = 0;
          Rprev = 0;
          iniciarCalibracion(SBP0, DBP0);
        }
        s.estadoSpo2 = m.a;
        break;

      case MSG_SPO2:
        s.spo2 = m.a;
        xQueueOverwrite(buzonSignos, &s);
        break;

      case MSG_PIR:
        pirSpo2 = m.v;
        break;
    }

    // Una vez por segundo: FC, PR, PA, FR y comandos de calibración
    TickType_t ahora = xTaskGetTickCount();
    if (ahora - tInfo >= pdMS_TO_TICKS(PERIODO_INFO_MS)) {
      tInfo = ahora;

      // ---- Calibración PA pedida desde T_display ----
      CalPA cal;
      if (xQueueReceive(colaCal, &cal, 0) == pdPASS) {
        iniciarCalibracion(cal.sbp, cal.dbp);
      }

      // ---- FC (ECG) ----
      bool sinLatidoEcg = (ahora - tUltLatEcg) > pdMS_TO_TICKS(TIMEOUT_LATIDO_MS);
      if (s.ecgDesconectado || sinLatidoEcg) {
        if (s.fc != 0) { s.fc = 0; iniciarPTEcg(); }
      } else {
        int bpmEcg = (int)(pt_bpm(&ptEcg) + 0.5f);
        if (bpmEcg >= 40 && bpmEcg <= 220) s.fc = bpmEcg;
      }

      // ---- PR (PPG) ----
      bool sinLatidoPpg = (ahora - tUltLatPpg) > pdMS_TO_TICKS(TIMEOUT_LATIDO_MS);
      if (s.estadoSpo2 == 0 || sinLatidoPpg) {
        if (s.pr != 0) { s.pr = 0; if (s.estadoSpo2 != 0) iniciarPTPpg(); }
      } else {
        int bpmPpg = (int)(pt_bpm(&ptPpg) + 0.5f);
        if (bpmPpg >= 40 && bpmPpg <= 220) s.pr = bpmPpg;
      }

      // ---- Presión arterial ----
      if (s.estadoSpo2 == 0 && s.pr == 0) {
        s.sbp = 0;
        s.dbp = 0;
      } else {
        s.sbp = (sbpEstable > 0) ? sbpEstable : (int)SBP0;
        s.dbp = (dbpEstable > 0) ? dbpEstable : (int)DBP0;
      }
      s.pas = (s.sbp > 0 && sbpEstable > 0) ? sbpEstable : -1;

      // ---- [FR] Frecuencia respiratoria estimada ----
      frActualizarSignos(s);

      // ---- Pendiente: se llenará con su sensor ----
      s.temp = -1.0f;

      xQueueOverwrite(buzonSignos, &s);
    }
  }
}


// ============================================================
// T_display  (núcleo 0, prio 2)
// ============================================================

// Lee "CAL <SBP> <DBP>" sin bloquear y lo manda a T_proc
void leerComandosSerial() {
  static char cmdBuf[32];
  static uint8_t idx = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      cmdBuf[idx] = '\0';
      CalPA cal;
      if (sscanf(cmdBuf, "CAL %f %f", &cal.sbp, &cal.dbp) == 2 &&
          cal.sbp > cal.dbp &&
          cal.sbp >= 60.0f && cal.sbp <= 250.0f &&
          cal.dbp >= 30.0f && cal.dbp <= 150.0f) {
        xQueueSendToBack(colaCal, &cal, 0);
      }
      idx = 0;
    } else if (c >= 32 && c <= 126) {
      if (idx < sizeof(cmdBuf) - 1) cmdBuf[idx++] = c;
    } else {
      idx = 0;   // bytes de eventos Nextion (0x65, 0xFF...) reinician el buffer
    }
  }
}

void tareaDisplay(void* arg) {
  EscalaAutomatica escalaEcg = {0.0f, 0.0f, 0, false};
  EscalaAutomatica escalaPpg = {0.0f, 0.0f, 0, false};

  TickType_t tUlt = xTaskGetTickCount();
  uint32_t ciclo = 0;
  PuntoGraf p;
  Signos s;

  for (;;) {
    vTaskDelayUntil(&tUlt, pdMS_TO_TICKS(PERIODO_GRAFICA_MS));

    while (xQueueReceive(colaGraf, &p, 0) == pdPASS) {
      if (p.id == ID_WAVE_ECG)
        nexAdd(ID_WAVE_ECG, escalarSenal(escalaEcg, p.v, SPAN_MIN_ECG_V, INVERTIR_ECG));
      else
        nexAdd(ID_WAVE_PPG, escalarSenal(escalaPpg, p.v, SPAN_MIN_PPG_V, INVERTIR_PPG));
    }

    if (++ciclo % (PERIODO_INFO_MS / PERIODO_GRAFICA_MS) == 0 &&
        xQueuePeek(buzonSignos, &s, 0) == pdPASS) {
      nexNumero("nHR", s.fc);
#if CALIBRAR_LOFF
      nexNumero("nPR",   s.calPpUv);
      nexNumero("nSpO2", s.calMediaMv);
#else
      nexNumero("nPR",   s.pr);
      nexNumero("nSpO2", s.spo2);
#endif
      nexNumero("nSBP", s.sbp);
      nexNumero("nDBP", s.dbp);

      // [FR] 0 en pantalla = todavía sin valor fiable
      nexNumero(NEX_OBJ_FR, (s.fr > 0) ? s.fr : 0);
#if USAR_TEXTO_ESTADO_FR
      char b[40];
      snprintf(b, sizeof(b), "%s.txt=\"%s\"", NEX_OBJ_FR_TXT, textoEstadoFR(s.frEstado));
      nexComando(b);
#endif
    }

    leerComandosSerial();
  }
}


// ============================================================
// T_alarma  (núcleo 0, prio 1): NEWS2 + alarmas técnicas
// ============================================================

#if SIMULAR_NEWS2
EntradaNews2 entradaSimulada() {
  static uint32_t seg = 0;
  uint8_t fase = (seg++ / 20) % 4;
  EntradaNews2 e = {16, 98, false, -1, 75, true, 36.8f, true, true};
  if (fase == 1) { e.pulso = 135; }
  if (fase == 2) { e.pulso = 115; e.spo2 = 93; e.fr = 22; }
  if (fase == 3) { e.pulso = 135; e.spo2 = 90; e.fr = 26; }
  return e;
}
#endif

void tareaAlarma(void* arg) {
  TickType_t  tUlt = xTaskGetTickCount();
  NivelAlarma nivelActual  = ALARMA_BAJA;
  NivelAlarma candidato    = ALARMA_BAJA;
  uint8_t     segCandidato = 0;

  bool ecgEnUso = false, spo2EnUso = false;
  uint8_t segTecEcg = 0, segTecSpo2 = 0;

  for (;;) {
    vTaskDelayUntil(&tUlt, pdMS_TO_TICKS(1000));

    Signos s;
    bool haySignos = (xQueuePeek(buzonSignos, &s, 0) == pdPASS);

#if SIMULAR_NEWS2
    EntradaNews2 e = entradaSimulada();
#else
    if (!haySignos) continue;

    EntradaNews2 e;
#if USAR_FR_EN_NEWS2
    e.fr    = s.fr;                 // [FR] -1 si no hay estimación fiable
#else
    e.fr    = -1;
#endif
    e.spo2  = (s.estadoSpo2 == 2) ? s.spo2 : -1;
#if USAR_PAS_EN_NEWS2
    e.pas   = s.pas;
#else
    e.pas   = -1;
#endif
    e.pulso = (s.fc > 0) ? s.fc : (s.pr > 0 ? s.pr : -1);
    e.temp  = s.temp;
    e.o2Suplementario    = false;
    e.o2Conocido         = false;   // pendiente: botón en Nextion
    e.alerta             = true;
    e.concienciaConocida = false;   // pendiente: botón en Nextion
#endif

    EstadoAlarma a = calcularNews2(e);

    // ---- Persistencia del nivel NEWS2 ----
    if (a.nivel == nivelActual) {
      segCandidato = 0;
    } else {
      if (a.nivel != candidato) { candidato = a.nivel; segCandidato = 0; }
      segCandidato++;
      uint8_t necesario = (candidato > nivelActual) ? ESCALAR_SEG : DESESCALAR_SEG;
      if (segCandidato >= necesario) { nivelActual = candidato; segCandidato = 0; }
    }
    a.nivel = nivelActual;

    // ---- Alarmas técnicas ----
#if SIMULAR_NEWS2
    a.tecEcg = false;
    a.tecSpo2 = false;
#else
    if (s.fc > 0)           ecgEnUso  = true;
    if (s.estadoSpo2 == 2)  spo2EnUso = true;

    bool condEcg  = ecgEnUso  && s.ecgDesconectado;
    bool condSpo2 = spo2EnUso && (s.estadoSpo2 == 0);

    segTecEcg  = condEcg  ? (segTecEcg  < 255 ? segTecEcg  + 1 : 255) : 0;
    segTecSpo2 = condSpo2 ? (segTecSpo2 < 255 ? segTecSpo2 + 1 : 255) : 0;

    a.tecEcg  = segTecEcg  >= TECNICA_SEG;
    a.tecSpo2 = segTecSpo2 >= TECNICA_SEG;
#endif

    xQueueOverwrite(buzonAlarma, &a);
  }
}


// ============================================================
// T_sonido  (núcleo 1, prio 2): ÚNICA dueña del buzzer
//   Prioridad: ALTA > MEDIA > TÉCNICA > bip de pulso
// ============================================================

ModoSonido modoSonidoActual() {
  EstadoAlarma a;
  if (xQueuePeek(buzonAlarma, &a, 0) != pdPASS) return SON_PULSO;
  if (a.nivel == ALARMA_ALTA)  return SON_ALTA;
  if (a.nivel == ALARMA_MEDIA) return SON_MEDIA;
  if (a.tecEcg || a.tecSpo2)   return SON_TECNICA;
  return SON_PULSO;
}

void reproducirPatron(const Paso* patron, size_t n, ModoSonido modo) {
  for (size_t i = 0; i < n; i++) {
    ledcWriteTone(PIN_BUZZER, patron[i].freq);
    uint16_t restante = patron[i].ms;
    while (restante > 0) {
      uint16_t paso = restante > 50 ? 50 : restante;
      vTaskDelay(pdMS_TO_TICKS(paso));
      restante -= paso;
      if (modoSonidoActual() != modo) { ledcWriteTone(PIN_BUZZER, 0); return; }
    }
  }
  ledcWriteTone(PIN_BUZZER, 0);
}

void tareaSonido(void* arg) {
  Signos s;
  for (;;) {
    ModoSonido modo = modoSonidoActual();

    switch (modo) {
      case SON_ALTA:
        reproducirPatron(PATRON_ALTA, sizeof(PATRON_ALTA) / sizeof(Paso), modo);
        continue;
      case SON_MEDIA:
        reproducirPatron(PATRON_MEDIA, sizeof(PATRON_MEDIA) / sizeof(Paso), modo);
        continue;
      case SON_TECNICA:
        reproducirPatron(PATRON_TECNICA, sizeof(PATRON_TECNICA) / sizeof(Paso), modo);
        continue;
      default:
        break;
    }

    int bpm = 0;
    if (xQueuePeek(buzonSignos, &s, 0) == pdPASS) bpm = (s.fc > 0) ? s.fc : s.pr;

    if (bpm < 40) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    ledcWriteTone(PIN_BUZZER, BUZZER_FREQ_HZ);
    vTaskDelay(pdMS_TO_TICKS(BUZZER_DURACION_MS));
    ledcWriteTone(PIN_BUZZER, 0);
    vTaskDelay(pdMS_TO_TICKS(60000UL / bpm - BUZZER_DURACION_MS));
  }
}


// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(NEX_BAUD);
#if DEBUG_FR_SERIAL1
  Serial1.begin(115200, SERIAL_8N1, -1, DEBUG_TX_PIN);   // [FR] solo TX, adaptador USB-TTL aparte
#endif

  ledcAttach(PIN_BUZZER, BUZZER_FREQ_HZ, 8);
  ledcWriteTone(PIN_BUZZER, 0);

  colaMsg     = xQueueCreate(256, sizeof(Msg));
  colaGraf    = xQueueCreate(64,  sizeof(PuntoGraf));
  colaCal     = xQueueCreate(2,   sizeof(CalPA));
  buzonSignos = xQueueCreate(1,   sizeof(Signos));
  buzonAlarma = xQueueCreate(1,   sizeof(EstadoAlarma));
  assert(colaMsg && colaGraf && colaCal && buzonSignos && buzonAlarma);

  while (!afe.configure(SCLK, MISO, MOSI, CS, AFE_RST, FIFO_RDY)) {
    delay(500);
  }
  afe.enableSpO2(true);

  nexComando("page 0");
  delay(50);
  nexComando("cle 1,0");
  nexComando("cle 2,0");
  delay(50);

  // Tareas            función        nombre  stack  par  prio handle núcleo
  xTaskCreatePinnedToCore(tareaProc,    "proc", 10240, NULL, 3, NULL, 1);   // [FR] +2 KB
  xTaskCreatePinnedToCore(tareaAdq,     "adq",  4096,  NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(tareaSonido,  "son",  2048,  NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(tareaDisplay, "disp", 4096,  NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(tareaAlarma,  "alm",  4096,  NULL, 1, NULL, 0);

  afe.enableCapture();
}

void loop() { vTaskDelete(NULL); }