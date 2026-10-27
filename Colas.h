#ifndef COLAS_H
#define COLAS_H

#include <Arduino.h>

// ============================================================
// COLAS Y BUZONES COMPARTIDOS ENTRE TAREAS
// Se DEFINEN en el .ino; aquí solo se declaran.
// ============================================================

extern QueueHandle_t colaMsg;       // Adquisicion   -> Procesamiento  (muestras y eventos)
extern QueueHandle_t colaGraf;      // Procesamiento -> Pantalla       (puntos de las ondas)
extern QueueHandle_t colaCal;       // Pantalla      -> Procesamiento  (comando CAL de PA)
extern QueueHandle_t buzonSignos;   // Procesamiento -> todos          (último valor de los signos)
extern QueueHandle_t buzonAlarma;   // GestorAlarmas -> Sonido / Pantalla

extern portMUX_TYPE      muxCont;
extern volatile uint32_t perdidas;

#endif // COLAS_H