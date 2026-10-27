#ifndef ALARMAS_H
#define ALARMAS_H

#include <Arduino.h>
#include "Config.h"
#include "Tipos.h"

// ============================================================
// GestorAlarmas: calcula el NEWS2 cada 1 s, aplica persistencia
// (subir 5 s / bajar 10 s) y detecta la alarma técnica del
// sensor de SpO2. Publica el resultado en buzonAlarma.
// ============================================================

class GestorAlarmas {
 public:
  void bucle();                       // lazo infinito de la tarea
  static void tarea(void* param);     // función para xTaskCreatePinnedToCore

  // NEWS2 (escala 1 de SpO2): puntaje total y nivel sin persistencia
  static EstadoAlarma calcularNews2(const EntradaNews2& e);

 private:
  // Puntaje de cada parámetro: 0-3, o -1 si no hay dato
  static int puntajeFR(int fr);
  static int puntajeSpO2(int spo2);
  static int puntajePAS(int pas);
  static int puntajePulso(int p);
  static int puntajeTemp(float t);

  EntradaNews2 armarEntrada(const Signos& s);
  EntradaNews2 entradaSimulada();
  NivelAlarma  aplicarPersistencia(NivelAlarma nuevo);
  void         alarmaTecnica(const Signos& s, EstadoAlarma& a);

  // --- estado de la persistencia ---
  NivelAlarma nivelActual  = ALARMA_BAJA;
  NivelAlarma candidato    = ALARMA_BAJA;
  uint8_t     segCandidato = 0;

  // --- estado de la alarma técnica de SpO2 ---
  bool    spo2EnUso  = false;
  uint8_t segTecSpo2 = 0;

  // --- simulación ---
  uint32_t segSimulacion = 0;
};

#endif // ALARMAS_H