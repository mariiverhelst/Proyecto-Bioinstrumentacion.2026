#ifndef PANTALLA_H
#define PANTALLA_H

#include <Arduino.h>
#include "Config.h"
#include "Tipos.h"

// ============================================================
// Pantalla: ondas, números, alarma visual, diagnóstico y comando CAL
// ============================================================

class Pantalla {
 public:
  void begin();
  void bucle();
  static void tarea(void* param);

 private:
  void enviar(char* buffer, int longitud);
  void comando(const char* cmd);
  void add(uint8_t id, uint8_t valor);
  void numero(const char* objeto, int valor);
  uint8_t escalar(EscalaAutomatica& escala, float valor, float spanMinimo, bool invertir);
  uint8_t nivelVisual(const EstadoAlarma& a);
  void    mostrarNumeros(const Signos& s);
  void    leerComandos();

  EscalaAutomatica escalaEcg = {0.0f, 0.0f, 0, false};
  EscalaAutomatica escalaPpg = {0.0f, 0.0f, 0, false};
  uint8_t visEnviado = 255;         // 255 = fuerza el primer envío
  char    cmdBuf[32];
  uint8_t cmdIdx = 0;
};

#endif // PANTALLA_H