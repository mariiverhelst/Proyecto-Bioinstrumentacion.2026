/*
 * MONITOR DE SIGNOS VITALES - ECG + PPG + SpO2 + PA + FR + ALARMAS
 * ESP32 + AFE4950 + NEXTION  (FreeRTOS + POO)
 *
 *   Adquisicion      AFE4950, ciclo ECG/PPG, validación SpO2, PIR   (núcleo 1, prio 5)
 *   Procesamiento    filtros, gráficas, FC, PR, SpO2, PA, FR        (núcleo 1, prio 3)
 *   PresionArterial  PA por PTT + PIR (la usa Procesamiento)
 *   Sonido           buzzer: bip y patrones de alarma               (núcleo 1, prio 2)
 *   Pantalla         ondas, números, alarma visual, CAL             (núcleo 0, prio 2)
 *   GestorAlarmas    NEWS2, persistencia, alarma técnica            (núcleo 0, prio 1)
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

// ---------------- Colas y variables compartidas ----------------
QueueHandle_t colaMsg;
QueueHandle_t colaGraf;
QueueHandle_t colaCal;
QueueHandle_t buzonSignos;
QueueHandle_t buzonAlarma;

portMUX_TYPE      muxCont            = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t perdidas           = 0;
volatile uint32_t resincronizaciones = 0;
volatile int32_t  difPpgEcgUv        = 0;

// ---------------- Objetos (globales: PresionArterial y Procesamiento son grandes) ----------------
Adquisicion     adquisicion;
PresionArterial presion;
Procesamiento   procesamiento(presion);
Sonido          sonido;
Pantalla        pantalla;
GestorAlarmas   alarmas;

void setup() {
  sonido.begin();                          // primero: transistor del buzzer apagado

  Serial.begin(NEX_BAUD);
#if DEBUG_FR_SERIAL1
  Serial1.begin(115200, SERIAL_8N1, -1, DEBUG_TX_PIN);
#endif

  colaMsg     = xQueueCreate(256, sizeof(Msg));
  colaGraf    = xQueueCreate(64,  sizeof(PuntoGraf));
  colaCal     = xQueueCreate(2,   sizeof(CalPA));
  buzonSignos = xQueueCreate(1,   sizeof(Signos));
  buzonAlarma = xQueueCreate(1,   sizeof(EstadoAlarma));
  assert(colaMsg && colaGraf && colaCal && buzonSignos && buzonAlarma);

  adquisicion.begin();
  pantalla.begin();

  //                       función               nombre  stack   objeto          prio núcleo
  xTaskCreatePinnedToCore(Procesamiento::tarea, "proc", 10240, &procesamiento, 3, NULL, 1);
  xTaskCreatePinnedToCore(Adquisicion::tarea,   "adq",  4096,  &adquisicion,   5, NULL, 1);
  xTaskCreatePinnedToCore(Sonido::tarea,        "son",  2048,  &sonido,        2, NULL, 1);
  xTaskCreatePinnedToCore(Pantalla::tarea,      "disp", 4096,  &pantalla,      2, NULL, 0);
  xTaskCreatePinnedToCore(GestorAlarmas::tarea, "alm",  4096,  &alarmas,       1, NULL, 0);

  adquisicion.iniciarCaptura();            // al final: el ciclo empieza en ECG
}

void loop() { vTaskDelete(NULL); }