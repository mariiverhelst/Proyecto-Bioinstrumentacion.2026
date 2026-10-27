/*
 * MONITOR DE SIGNOS VITALES - ESP32 + AFE4950 + NEXTION  (FreeRTOS + POO)
 * ECG + PPG + FC + PR + SpO2 + PA (PTT + PIR) + FR estimada
 * + alarmas NEWS2 (sonoras y visuales) + alarma técnica del sensor de SpO2
 *
 * Organización (una clase por pestaña):
 *   Config.h            parámetros ajustables
 *   Tipos.h             estructuras compartidas
 *   Colas.h             colas y buzones (se definen aquí abajo)
 *   Utilidades.h        funciones pequeñas de filtros
 *   Adquisicion         AFE4950, decodificación, validación de SpO2   (núcleo 1, prio 5)
 *   Procesamiento       filtros, FC/PR, FR, signos                     (núcleo 1, prio 3)
 *   PresionArterial     PA por PTT + PIR (la usa Procesamiento)
 *   Sonido              buzzer y patrones                              (núcleo 1, prio 2)
 *   Pantalla            Nextion: ondas, números, alarma visual, CAL    (núcleo 0, prio 2)
 *   GestorAlarmas       NEWS2, persistencia, alarma técnica            (núcleo 0, prio 1)
 *
 * Por Serial solo se envían comandos Nextion (no usar Serial.print).
 */

#include <Arduino.h>
#include "Config.h"
#include "Tipos.h"
#include "Colas.h"
#include "Adquisicion.h"
#include "PresionArterial.h"
#include "Procesamiento.h"
#include "Sonido.h"
#include "Pantalla.h"
#include "Alarmas.h"

// ============================================================
// COLAS Y BUZONES (declarados en Colas.h)
// ============================================================

QueueHandle_t colaMsg;       // Adquisicion   -> Procesamiento  (muestras y eventos)
QueueHandle_t colaGraf;      // Procesamiento -> Pantalla       (puntos de las ondas)
QueueHandle_t colaCal;       // Pantalla      -> Procesamiento  (comando CAL de PA)
QueueHandle_t buzonSignos;   // Procesamiento -> todos          (último valor de los signos)
QueueHandle_t buzonAlarma;   // GestorAlarmas -> Sonido / Pantalla

portMUX_TYPE      muxCont  = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t perdidas = 0;

// ============================================================
// OBJETOS (globales: PresionArterial y Procesamiento son grandes)
// ============================================================

Adquisicion     adquisicion;
PresionArterial presion;
Procesamiento   procesamiento(presion);
Sonido          sonido;
Pantalla        pantalla;
GestorAlarmas   alarmas;

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(NEX_BAUD);
#if DEBUG_FR_SERIAL1
  Serial1.begin(115200, SERIAL_8N1, -1, DEBUG_TX_PIN);   // solo TX, adaptador USB-TTL aparte
#endif

  sonido.begin();

  // 1) Colas antes que las tareas
  colaMsg     = xQueueCreate(256, sizeof(Msg));
  colaGraf    = xQueueCreate(64,  sizeof(PuntoGraf));
  colaCal     = xQueueCreate(2,   sizeof(CalPA));
  buzonSignos = xQueueCreate(1,   sizeof(Signos));
  buzonAlarma = xQueueCreate(1,   sizeof(EstadoAlarma));
  assert(colaMsg && colaGraf && colaCal && buzonSignos && buzonAlarma);

  // 2) Hardware
  adquisicion.begin();
  pantalla.begin();

  // 3) Tareas               función               nombre  stack  objeto          prio  núcleo
  xTaskCreatePinnedToCore(Procesamiento::tarea, "proc", 10240, &procesamiento, 3, NULL, 1);
  xTaskCreatePinnedToCore(Adquisicion::tarea,   "adq",  4096,  &adquisicion,   5, NULL, 1);
  xTaskCreatePinnedToCore(Sonido::tarea,        "son",  2048,  &sonido,        2, NULL, 1);
  xTaskCreatePinnedToCore(Pantalla::tarea,      "disp", 4096,  &pantalla,      2, NULL, 0);
  xTaskCreatePinnedToCore(GestorAlarmas::tarea, "alm",  4096,  &alarmas,       1, NULL, 0);

  // 4) Al final: arrancar la captura (el ciclo ECG, ECG, PPG empieza sincronizado)
  adquisicion.iniciarCaptura();
}

void loop() { vTaskDelete(NULL); }   // todo vive en tareas