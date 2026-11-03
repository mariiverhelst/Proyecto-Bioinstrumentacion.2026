#ifndef COLAS_H
#define COLAS_H

#include <Arduino.h>

// ============================================================
// COLAS Y VARIABLES COMPARTIDAS (se definen en el .ino)
// ============================================================

extern QueueHandle_t colaMsg;       // Adquisicion   -> Procesamiento
extern QueueHandle_t colaGraf;      // Procesamiento -> Pantalla
extern QueueHandle_t colaCal;       // Pantalla      -> Procesamiento (comando CAL)
extern QueueHandle_t buzonSignos;   // Procesamiento -> Pantalla / Sonido / GestorAlarmas
extern QueueHandle_t buzonAlarma;   // GestorAlarmas -> Sonido / Pantalla

extern portMUX_TYPE      muxCont;
extern volatile uint32_t perdidas;
extern volatile uint32_t resincronizaciones;
extern volatile int32_t  difPpgEcgUv;

#endif // COLAS_H