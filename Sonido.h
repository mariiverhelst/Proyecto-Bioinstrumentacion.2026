#ifndef SONIDO_H
#define SONIDO_H

#include <Arduino.h>
#include "Config.h"
#include "Tipos.h"

// ============================================================
// Sonido: ÚNICO dueño del buzzer
//   Prioridad: ALTA > MEDIA > TÉCNICA > bip de pulso
//   Con USAR_BUZZER = 0 el transistor queda siempre apagado.
// ============================================================

class Sonido {
 public:
  void begin();
  void bucle();
  static void tarea(void* param);

 private:
  void       tono(uint16_t freq);
  ModoSonido modoActual();
  void       reproducir(const Paso* patron, size_t n, ModoSonido modo);
  void       bipPulso();
};

#endif // SONIDO_H