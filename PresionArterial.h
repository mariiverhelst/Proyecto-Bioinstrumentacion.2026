#ifndef PRESION_ARTERIAL_H
#define PRESION_ARTERIAL_H

#include <Arduino.h>
#include "Config.h"

// ============================================================
// PresionArterial: estimación de PA por PTT + PIR (Ding et al., 2017)
//
// IMPORTANTE: el objeto ocupa ~24 KB (buffers), debe ser GLOBAL.
// ============================================================

class PresionArterial {
 public:
  uint32_t agregarMuestra(float ecgGrafica, float ppgGrafica, float ppgFiltrada);
  void     nuevoLatido(uint32_t idx, float ppgDC, float pirSpo2);
  void     calibrar(float sbp, float dbp);
  void     reiniciar();

  bool calculada()  const { return sbpEstable > 0; }
  int  sistolica()  const { return (sbpEstable > 0) ? sbpEstable : (int)SBP0; }
  int  diastolica() const { return (dbpEstable > 0) ? dbpEstable : (int)DBP0; }

 private:
  void procesarLatido(uint32_t R, uint32_t Rsig, float ppgDC, float pirSpo2);
  void revisarYPromediar(float ptt, float pir);
  void acumularCalibracion(float ptt, float pir);
  void calcular(float ptt, float pir);

  float    ecgBuf[N_BUF];
  float    ppgBuf[N_BUF];
  float    dcBuf[N_BUF];
  uint32_t nMuestras = 0;
  uint32_t Rprev = 0;

  float hPtt[NPROM];
  float hPir[NPROM];
  int   nProm = 0, iProm = 0;
  float pttProm = 0.0f, pirProm = 1.02f;

  bool  calibrando = true;
  bool  calibrado  = false;
  float SBP0 = SBP_DEF, DBP0 = DBP_DEF;
  float PTT0 = 260.0f,  PIR0 = 1.02f;
  float sumPtt = 0.0f,  sumPir = 0.0f;
  int   nCal = 0;

  int sbpEstable = 0;
  int dbpEstable = 0;
};

#endif // PRESION_ARTERIAL_H