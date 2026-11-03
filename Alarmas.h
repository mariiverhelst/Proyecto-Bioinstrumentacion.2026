#ifndef ALARMAS_H
#define ALARMAS_H

#include <Arduino.h>
#include "Config.h"
#include "Tipos.h"

// ============================================================
// GestorAlarmas: NEWS2 cada 1 s, persistencia (subir 5 s / bajar 10 s)
// y alarma técnica del sensor de SpO2. Publica en buzonAlarma.
// ============================================================

class GestorAlarmas {
 public:
  void bucle();
  static void tarea(void* param);

  static EstadoAlarma calcularNews2(const EntradaNews2& e);

 private:
  static int puntajeFR(int fr);
  static int puntajeSpO2(int spo2);
  static int puntajePAS(int pas);
  static int puntajePulso(int p);
  static int puntajeTemp(float t);

  EntradaNews2 armarEntrada(const Signos& s);
  EntradaNews2 entradaSimulada();
  NivelAlarma  aplicarPersistencia(NivelAlarma nuevo);
  void         alarmaTecnica(const Signos& s, EstadoAlarma& a);

  NivelAlarma nivelActual  = ALARMA_BAJA;
  NivelAlarma candidato    = ALARMA_BAJA;
  uint8_t     segCandidato = 0;

  bool    spo2EnUso  = false;
  uint8_t segTecSpo2 = 0;

  uint32_t segSimulacion = 0;
};

#endif // ALARMAS_H