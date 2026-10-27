#ifndef PANTALLA_H
#define PANTALLA_H

#include <Arduino.h>
#include "Config.h"
#include "Tipos.h"

// ============================================================
// Pantalla: ÚNICA dueña de Serial (= Nextion)
//   - Dibuja las ondas (con autoescalado)
//   - Muestra los números una vez por segundo
//   - Envía el nivel de alarma visual (el parpadeo lo hace la Nextion)
//   - Lee el comando "CAL <SBP> <DBP>"
// ============================================================

class Pantalla {
 public:
  void begin();                     // inicializa la Nextion (llamar en setup)
  void bucle();                     // lazo infinito de la tarea
  static void tarea(void* param);   // función para xTaskCreatePinnedToCore

 private:
  // --- comandos Nextion ---
  void enviar(char* buffer, int longitud);
  void comando(const char* cmd);
  void add(uint8_t id, uint8_t valor);
  void numero(const char* objeto, int valor);

  // --- autoescalado ---
  uint8_t escalar(EscalaAutomatica& escala, float valor, float spanMinimo, bool invertir);

  // --- alarma visual y comandos ---
  uint8_t nivelVisual(const EstadoAlarma& a);
  void    leerComandos();
  void    mostrarNumeros(const Signos& s);

  // --- estado ---
  EscalaAutomatica escalaEcg = {0.0f, 0.0f, 0, false};
  EscalaAutomatica escalaPpg = {0.0f, 0.0f, 0, false};
  uint8_t visEnviado = 255;         // 255 = fuerza el primer envío
  char    cmdBuf[32];
  uint8_t cmdIdx = 0;
};

#endif // PANTALLA_H