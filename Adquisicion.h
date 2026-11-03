#ifndef ADQUISICION_H
#define ADQUISICION_H

#include <Arduino.h>
#include <AFE4950.h>
#include "Config.h"
#include "Tipos.h"

// ============================================================
// Adquisicion: ÚNICA dueña del AFE4950
//   - Lee el FIFO (ciclo ECG, ECG, PPG) y separa los canales
//   - Resincroniza el ciclo si la PPG no está en la posición esperada
//   - Valida la SpO2 (R, PI, DC) con histéresis y calcula el PIR
// ============================================================

class Adquisicion {
 public:
  void begin();
  void iniciarCaptura();
  void bucle();
  static void tarea(void* param);

 private:
  void encolar(TipoMsg tipo, uint8_t a, float v);
  void procesarPaquete(const uint8_t* datos, size_t longitud);
  void procesarMuestra(uint32_t valor24);
  void evaluarFase();
  void procesarSpO2();

  AFE4950 afe;

  uint8_t cicloIdx = 0;               // 0 = ECG, 1 = ECG, 2 = PPG

  // --- resincronización ---
  float    sumaSlot[3] = {0, 0, 0};
  uint16_t nSlot[3]    = {0, 0, 0};
  uint16_t nSync       = 0;

  // --- SpO2 de la librería ---
  uint8_t  spo2Estado = 0;
  uint32_t acIr = 0, dcIr = 0, acRed = 0, dcRed = 0;

  // --- histéresis de la validación ---
  uint8_t segVal = 0, segInval = 0;
  bool    validada = false;
  float   ultimoR  = 0.0f;
};

#endif // ADQUISICION_H