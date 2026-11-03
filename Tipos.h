#ifndef TIPOS_H
#define TIPOS_H

#include <Arduino.h>

// ============================================================
// TIPOS COMPARTIDOS - con alarmas
// ============================================================

enum TipoMsg : uint8_t {
  MSG_ECG = 0,        // v = voltios, a = 1 si la muestra está saturada
  MSG_PPG,            // v = voltios, a = 1 si la muestra está saturada
  MSG_ESTADO_SPO2,    // a = estado efectivo (0 sin sensor, 1 estabilizando, 2 válida)
  MSG_SPO2,           // a = SpO2 (%), 0 si no es válida
  MSG_PIR,            // v = PIR desde AC/DC infrarrojo (0 = no válido)
  MSG_CAL_R,          // v = R           (calibración)
  MSG_CAL_PI,         // v = PI en %     (calibración)
  MSG_CAL_DC,         // v = DC IR       (calibración)
  MSG_CAL_EST         // a = estado de la librería (calibración)
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

struct CalPA {               // Pantalla -> Procesamiento (comando CAL)
  float sbp;
  float dbp;
};

struct Signos {              // Procesamiento -> Pantalla / Sonido / GestorAlarmas (mailbox)
  int     fc;                // lpm ECG (0 = sin dato)
  int     pr;                // lpm PPG (0 = sin dato)
  int     spo2;              // %       (0 = sin dato)
  uint8_t estadoSpo2;        // 0 sin sensor, 1 estabilizando, 2 válida
  int     sbp;               // mmHg para pantalla (0 = sin dato)
  int     dbp;               // mmHg para pantalla (0 = sin dato)
  int     pas;               // mmHg para NEWS2 (-1 = no hay PA calculada)
  int     fr;                // rpm (-1 = sin valor fiable)
  uint8_t frEstado;          // RespRate::Status o FR_EST_CAL_RELOJ / FR_EST_CONFIG
  // calibración
  int     calR100;
  int     calPi100;
  int     calDcK;
  uint8_t calEstadoLib;
};

struct EscalaAutomatica {
  float    maximo;
  float    bloqueMaximo;
  uint16_t cantidad;
  bool     inicializada;
};

// ---------------- Alarmas ----------------

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
  uint8_t     news2;         // puntaje total
  NivelAlarma nivel;         // nivel con persistencia
  bool        algunTres;     // algún parámetro sumó 3
  uint8_t     parametros;    // cuántos de los 7 se puntuaron
  bool        tecSpo2;       // alarma técnica: sensor de SpO2 retirado
};

enum ModoSonido : uint8_t { SON_PULSO = 0, SON_TECNICA, SON_MEDIA, SON_ALTA };

struct Paso {                // un paso de un patrón sonoro
  uint16_t freq;             // Hz (0 = silencio)
  uint16_t ms;
};

#endif // TIPOS_H