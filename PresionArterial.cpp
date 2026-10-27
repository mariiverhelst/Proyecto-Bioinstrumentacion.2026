#include "PresionArterial.h"

// ============================================================
// Buffers circulares a 500 Hz
// ============================================================

uint32_t PresionArterial::agregarMuestra(float ecgGrafica, float ppgGrafica, float ppgFiltrada) {
  uint32_t idx = nMuestras;
  B(ecgBuf, idx) = INVERTIR_ECG ? -ecgGrafica : ecgGrafica;
  B(ppgBuf, idx) = INVERTIR_PPG ? -ppgGrafica : ppgGrafica;
  B(dcBuf,  idx) = ppgFiltrada;
  nMuestras = idx + 1;
  return idx;
}

// ============================================================
// Calibración
// ============================================================

void PresionArterial::calibrar(float sbp, float dbp) {
  SBP0 = sbp;
  DBP0 = dbp;
  sumPtt = 0.0f;
  sumPir = 0.0f;
  nCal = 0;
  calibrado  = false;
  calibrando = true;
}

void PresionArterial::reiniciar() {
  Rprev = 0;
  calibrar(SBP0, DBP0);
}

void PresionArterial::acumularCalibracion(float ptt, float pir) {
  sumPtt += ptt;
  sumPir += pir;
  nCal++;
  PTT0 = sumPtt / nCal;      // se actualiza progresivamente
  PIR0 = sumPir / nCal;
  if (nCal >= N_CAL) {
    calibrando = false;
    calibrado  = true;
  }
}

// ============================================================
// Ubicar el pico R alrededor del latido detectado
// ============================================================

void PresionArterial::nuevoLatido(uint32_t m, float ppgDC, float pirSpo2) {
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

// ============================================================
// PTT y PIR de un latido
// ============================================================

void PresionArterial::procesarLatido(uint32_t R, uint32_t Rsig, float ppgDC, float pirSpo2) {
  if (Rsig <= R || (Rsig - R) >= N_BUF - 10) return;

  float rr = (Rsig - R) * 1000.0f / FS_PA;
  if (rr < 300.0f || rr > 2200.0f) return;

  // Frente más pronunciado del PPG en [R + 80 ms, R + 600 ms]
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

  // PTT en ms
  float ptt = (k - R) * 1000.0f / FS_PA;

  // PIR
  float pir = 1.02f;
#if USAR_PIR
  if (pirSpo2 > 0.0f) {
    pir = pirSpo2;                         // calculado en Adquisicion con acIr / dcIr
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

// ============================================================
// Revisar y promediar
// ============================================================

void PresionArterial::revisarYPromediar(float ptt, float pir) {
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

  calcular(pttProm, pirProm);
}

// ============================================================
// Modelo de presión (Ding et al., 2017)
// ============================================================

void PresionArterial::calcular(float ptt, float pir) {
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