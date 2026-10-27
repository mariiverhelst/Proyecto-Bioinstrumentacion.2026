#ifndef SONIDO_H
#define SONIDO_H

#include <Arduino.h>
#include "Config.h"
#include "Tipos.h"

// ============================================================
// Sonido: ÚNICO dueño del buzzer
//   Prioridad: ALTA > MEDIA > TÉCNICA > bip de pulso
// ============================================================

class Sonido {
 public:
  void begin();                       // configura el PWM del buzzer (llamar en setup)
  void bucle();                       // lazo infinito de la tarea
  static void tarea(void* param);     // función para xTaskCreatePinnedToCore

 private:
  ModoSonido modoActual();
  void reproducir(const Paso* patron, size_t n, ModoSonido modo);
  void bipPulso();
};

#endif // SONIDO_H