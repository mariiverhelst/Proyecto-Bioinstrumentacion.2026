#ifndef TIPOS_H
#define TIPOS_H

#include <Arduino.h>

// ============================================================
// TIPOS COMPARTIDOS ENTRE TAREAS
// ============================================================

enum TipoMsg : uint8_t {
  MSG_ECG = 0,        // v = voltios, a = 1 si la muestra está saturada
  MSG_PPG,            // v = voltios, a = 1 si la muestra está saturada
  MSG_ESTADO_SPO2,    // a = estado EFECTIVO (0 = sin sensor, 1 = estabilizando, 2 = válida)
  MSG_SPO2,           // a = SpO2 (%), 0 si no es válida
  MSG_PIR,            // v = PIR del procesador SpO2 (0 = no válido)
  MSG_CAL_R,          // v = R        (calibración)
  MSG_CAL_PI,         // v = PI en %  (calibración)
  MSG_CAL_DC,         // v = DC IR    (calibración)
  MSG_CAL_EST         // a = estado que reporta la librería (diagnóstico)
};

struct Msg {                 // Adquisicion -> Procesamiento
  TipoMsg tipo;
  uint8_t a;
  float   v;
};

struct PuntoGraf {           // Procesamiento -> Pantalla
  uint8_t id;
  float   v;
};

struct CalPA {               // Pantalla -> Procesamiento
  float sbp;
  float dbp;
};

struct Signos {              // Procesamiento -> Pantalla / Sonido / GestorAlarmas (mailbox)
  int     fc;                // lpm ECG (0 = sin dato)
  int     pr;                // lpm PPG (0 = sin dato)
  int     spo2;              // %       (0 = sin dato)
  uint8_t estadoSpo2;        // estado efectivo: 0 = sin sensor, 1 = estabilizando, 2 = válida
  int     fr;                // rpm   (-1 = no disponible)
  float   temp;              // °C    (-1 = no disponible todavía)
  int     pas;               // mmHg para NEWS2 (-1 = no disponible)
  int     sbp;               // mmHg para pantalla (0 = sin dato)
  int     dbp;               // mmHg para pantalla (0 = sin dato)
  // calibración / diagnóstico de SpO2
  int     calR100;           // R x 100
  int     calPi100;          // PI x 100 (%)
  int     calDcK;            // DC IR (miles de cuentas)
  uint8_t calEstadoLib;      // estado que reporta la librería
  uint8_t frEstado;          // RespRate::Status o FR_EST_CAL_RELOJ / FR_EST_CONFIG
};

struct EscalaAutomatica {    // autoescalado de las gráficas (Pantalla)
  float maximo;
  float bloqueMaximo;
  uint16_t cantidad;
  bool inicializada;
};

enum NivelAlarma : uint8_t { ALARMA_BAJA = 0, ALARMA_MEDIA = 1, ALARMA_ALTA = 2 };

struct EntradaNews2 {        // -1 = parámetro no disponible (no se puntúa)
  int   fr;
  int   spo2;
  bool  o2Suplementario;
  int   pas;
  int   pulso;
  bool  alerta;
  float temp;
  bool  o2Conocido;
  bool  concienciaConocida;
};

struct EstadoAlarma {        // GestorAlarmas -> Sonido / Pantalla (mailbox)
  uint8_t     news2;
  NivelAlarma nivel;         // filtrado por persistencia
  bool        algunTres;
  uint8_t     parametros;    // cuántos de los 7 se puntuaron
  bool        tecSpo2;       // alarma técnica: sensor SpO2 retirado
};

enum ModoSonido : uint8_t { SON_PULSO = 0, SON_TECNICA, SON_MEDIA, SON_ALTA };

struct Paso {                // un paso de un patrón sonoro
  uint16_t freq;             // Hz (0 = silencio)
  uint16_t ms;
};

#endif // TIPOS_H