#ifndef ADQUISICION_H
#define ADQUISICION_H

#include <Arduino.h>
#include <AFE4950.h>
#include "Config.h"
#include "Tipos.h"

// ============================================================
// Adquisicion: ÚNICA dueña del AFE4950
//   - Lee el FIFO y decodifica las palabras (LSB, MID, MSB, TAG)
//   - Separa ECG y PPG (ciclo continuo ECG, ECG, PPG)
//   - Valida la SpO2 (R, PI, DC) con histéresis
//   - Envía todo a Procesamiento por colaMsg
// ============================================================

class Adquisicion {
 public:
  void begin();                       // configura el AFE (llamar en setup)
  void iniciarCaptura();              // arranca la captura (al final de setup)
  void bucle();                       // lazo infinito de la tarea
  static void tarea(void* param);     // función para xTaskCreatePinnedToCore

 private:
  void encolar(TipoMsg tipo, uint8_t a, float v);
  void procesarPaquete(const uint8_t* datos, size_t longitud);
  void procesarMuestra(uint32_t valor24);
  void procesarSpO2();                // se llama al llegar DC_RED (1 vez por segundo)

  AFE4950 afe;

  // --- decodificación ---
  uint8_t cicloIdx = 0;               // 0 = ECG, 1 = ECG, 2 = PPG (igual que _fifoSlot)

  // --- parámetros de SpO2 que entrega la librería ---
  uint8_t  spo2Estado = 0;            // 0 sin dedo, 1 estabilizando, 2 válida
  uint32_t acIr = 0, dcIr = 0, acRed = 0, dcRed = 0;

  // --- histéresis de la validación ---
  uint8_t segVal   = 0;
  uint8_t segInval = 0;
  bool    validada = false;
  float   ultimoR  = 0.0f;
};

#endif // ADQUISICION_H