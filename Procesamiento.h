#ifndef PROCESAMIENTO_H
#define PROCESAMIENTO_H

#include <Arduino.h>
#include "Config.h"
#include "Tipos.h"
#include "biosignals.h"
#include "Utilidades.h"
#include "RespRateEstimator.h"
#include "PresionArterial.h"

// ============================================================
// Procesamiento: filtros, gráficas, FC, PR, SpO2, PA y FR
//
// IMPORTANTE: el objeto ocupa ~15 KB (estimador de FR), debe ser GLOBAL.
// ============================================================

class Procesamiento {
 public:
  explicit Procesamiento(PresionArterial& pa) : presion(pa) {}

  void bucle();
  static void tarea(void* param);

 private:
  void inicializar();
  void iniciarPTEcg();
  void iniciarPTPpg();
  void enviarPunto(uint8_t id, float v);
  void procesarEcg(float voltios, bool saturada);
  void procesarPpg(float voltios, bool saturada);
  void procesarEstadoSpo2(uint8_t estado);
  void cadaSegundo(TickType_t ahora);

  // --- frecuencia respiratoria ---
  void frAlimentarEcg(float cuentas, bool saturada);
  void frAlimentarPpg(float cuentas, bool saturada);
  void frActualizar();

  PresionArterial& presion;

  // --- ECG ---
  PanTompkins ptEcg;
  Biquad ecgLP1, ecgLP2, ecgHP;
  bool     hayEcg = false;
  uint8_t  decimEcg = 0;
  uint32_t tiempoEcgUs = 0;
  bool     tEcgIni = false;
  uint32_t beatsEcgPrev = 0;
  bool     ecgPlano = true;

  // --- PPG ---
  PanTompkins ptPpg;
  Biquad ppgLP1, ppgLP2, ppgHP;
  bool     hayPpg = false;
  uint8_t  acumPpg = 0;
  float    kDC = 0.0f, ppgDC = 0.0f;
  uint32_t tiempoPpgUs = 0;
  bool     tPpgIni = false;
  uint32_t beatsPpgPrev = 0;

  // --- últimos valores de PPG (para la PA) ---
  float ppgGraficaUlt  = 0.0f;
  float ppgFiltradaUlt = 0.0f;
  float pirSpo2        = 0.0f;

  // --- tiempos ---
  TickType_t tUltLatEcg    = 0;
  TickType_t tUltLatPpg    = 0;
  TickType_t tInfo         = 0;
  TickType_t tEcgPerdido   = 0;
  bool       ecgPerdido    = false;
  bool       ecgReiniciado = false;
  TickType_t tReinicioPpg  = 0;
  TickType_t tPpgAusente   = 0;

  // --- SpO2 ---
  uint8_t estadoLib = 0;
  int     spo2Lib   = 0;
  bool    sinPulso  = false;

  // --- frecuencia respiratoria ---
  RespRate::Estimator frEst;
  bool     frCalibrando    = true;
  bool     frDeshabilitado = false;
  uint32_t frCalInicioUs = 0, frCalMuestras = 0;
  uint32_t frTbInicioUs  = 0, frTbMuestras  = 0;

  Signos s = {0, 0, 0, 0, 0, 0, -1, -1, FR_EST_CAL_RELOJ, 0, 0, 0, 0};
};

#endif // PROCESAMIENTO_H