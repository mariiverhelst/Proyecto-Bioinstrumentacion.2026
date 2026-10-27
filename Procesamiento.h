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
// Procesamiento: recibe las muestras de Adquisicion y calcula todo
//   - Filtros visuales y puntos de las gráficas (decimados a 100 Hz)
//   - Pan-Tompkins ECG (FC) y PPG (PR), con caída a 0 sin latidos
//   - Estado efectivo de SpO2 (sin pulso = sin sensor)
//   - Alimenta PresionArterial y el estimador de FR
//   - Publica los signos en buzonSignos una vez por segundo
//
// IMPORTANTE: el objeto ocupa ~15 KB (estimador de FR), debe ser GLOBAL.
// ============================================================

class Procesamiento {
 public:
  explicit Procesamiento(PresionArterial& pa) : presion(pa) {}

  void bucle();                       // lazo infinito de la tarea
  static void tarea(void* param);     // función para xTaskCreatePinnedToCore

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

  // --- filtros y detectores ---
  PanTompkins ptEcg, ptPpg;
  Biquad ecgLP1, ecgLP2, ecgHP;
  Biquad ppgLP1, ppgLP2, ppgHP;
  float kDC = 0.0f;
  float ppgDC = 0.0f;
  uint32_t beatsEcgPrev = 0, beatsPpgPrev = 0;

  uint32_t tiempoEcgUs = 0, tiempoPpgUs = 0;
  bool tEcgIni = false, tPpgIni = false;
  bool hayEcg = false, hayPpg = false;
  uint8_t decimEcg = 0;
  uint8_t acumPpg  = 0;

  // --- últimos valores de PPG (para la PA) ---
  float ppgGraficaUlt  = 0.0f;
  float ppgFiltradaUlt = 0.0f;
  float pirSpo2        = 0.0f;

  // --- tiempos de latidos ---
  TickType_t tUltLatEcg  = 0;
  TickType_t tUltLatPpg  = 0;
  TickType_t tPpgAusente = 0;
  TickType_t tInfo       = 0;

  // --- ECG ausente (reinicio del Pan-Tompkins solo si es largo) ---
  TickType_t tEcgPerdido   = 0;
  bool       ecgPerdido    = false;
  bool       ecgReiniciado = false;

  // --- estado de SpO2 ---
  uint8_t estadoLib = 0;      // estado efectivo recibido de Adquisicion
  int     spo2Lib   = 0;
  bool    sinPulso  = false;  // sin pulsos en la PPG aunque la librería diga "válida"

  // --- frecuencia respiratoria ---
  RespRate::Estimator frEst;
  bool     frCalibrando    = true;
  bool     frDeshabilitado = false;
  uint32_t frCalInicioUs = 0, frCalMuestras = 0;
  uint32_t frTbInicioUs  = 0, frTbMuestras  = 0;

  // --- signos publicados ---
  Signos s = {0, 0, 0, 0, -1, -1.0f, -1, 0, 0, 0, 0, 0, 0, FR_EST_CAL_RELOJ};
};

#endif // PROCESAMIENTO_H