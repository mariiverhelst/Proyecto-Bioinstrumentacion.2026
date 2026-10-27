#include "Alarmas.h"
#include "Colas.h"

void GestorAlarmas::tarea(void* param) {
  static_cast<GestorAlarmas*>(param)->bucle();
}

// ============================================================
// Puntajes NEWS2 (escala 1 de SpO2)
// ============================================================

int GestorAlarmas::puntajeFR(int fr) {
  if (fr < 0)   return -1;
  if (fr <= 8)  return 3;
  if (fr <= 11) return 1;
  if (fr <= 20) return 0;
  if (fr <= 24) return 2;
  return 3;
}

int GestorAlarmas::puntajeSpO2(int spo2) {
  if (spo2 <= 0)  return -1;
  if (spo2 <= 91) return 3;
  if (spo2 <= 93) return 2;
  if (spo2 <= 95) return 1;
  return 0;
}

int GestorAlarmas::puntajePAS(int pas) {
  if (pas <= 0)   return -1;
  if (pas <= 90)  return 3;
  if (pas <= 100) return 2;
  if (pas <= 110) return 1;
  if (pas <= 219) return 0;
  return 3;
}

int GestorAlarmas::puntajePulso(int p) {
  if (p <= 0)   return -1;
  if (p <= 40)  return 3;
  if (p <= 50)  return 1;
  if (p <= 90)  return 0;
  if (p <= 110) return 1;
  if (p <= 130) return 2;
  return 3;
}

int GestorAlarmas::puntajeTemp(float t) {
  if (t < 0.0f)   return -1;
  if (t <= 35.0f) return 3;
  if (t <= 36.0f) return 1;
  if (t <= 38.0f) return 0;
  if (t <= 39.0f) return 1;
  return 2;
}

EstadoAlarma GestorAlarmas::calcularNews2(const EntradaNews2& e) {
  int p[7] = {
    puntajeFR(e.fr),
    puntajeSpO2(e.spo2),
    e.o2Conocido ? (e.o2Suplementario ? 2 : 0) : -1,
    puntajePAS(e.pas),
    puntajePulso(e.pulso),
    e.concienciaConocida ? (e.alerta ? 0 : 3) : -1,
    puntajeTemp(e.temp)
  };

  EstadoAlarma a = {0, ALARMA_BAJA, false, 0, false};
  for (int i = 0; i < 7; i++) {
    if (p[i] < 0) continue;            // parámetro no disponible
    a.news2 += p[i];
    a.parametros++;
    if (p[i] == 3) a.algunTres = true;
  }

  if (a.news2 >= 7)                     a.nivel = ALARMA_ALTA;
  else if (a.news2 >= 5 || a.algunTres) a.nivel = ALARMA_MEDIA;
  else                                  a.nivel = ALARMA_BAJA;
  return a;
}

// ============================================================
// Entrada del NEWS2: real o simulada
// ============================================================

EntradaNews2 GestorAlarmas::armarEntrada(const Signos& s) {
  EntradaNews2 e;
#if USAR_FR_EN_NEWS2
  e.fr    = s.fr;                      // -1 si no hay estimación fiable
#else
  e.fr    = -1;
#endif
  e.spo2  = (s.estadoSpo2 == 2) ? s.spo2 : -1;
#if USAR_PAS_EN_NEWS2
  e.pas   = s.pas;
#else
  e.pas   = -1;
#endif
  e.pulso = (s.fc > 0) ? s.fc : (s.pr > 0 ? s.pr : -1);
#if USAR_TEMP_EN_NEWS2
  e.temp  = s.temp;
#else
  e.temp  = -1.0f;
#endif
  e.o2Suplementario    = false;
  e.o2Conocido         = false;        // sin entrada manual
  e.alerta             = true;
  e.concienciaConocida = false;        // sin entrada manual
  return e;
}

// 4 fases de 20 s: normal -> medio (un 3) -> medio (total 6) -> alto (total 9)
EntradaNews2 GestorAlarmas::entradaSimulada() {
  uint8_t fase = (segSimulacion++ / 20) % 4;
  EntradaNews2 e = {16, 98, false, 120, 75, true, 36.8f, true, true};
  if (fase == 1) { e.pulso = 135; }                              // pulso 3       -> MEDIA
  if (fase == 2) { e.pulso = 115; e.spo2 = 93; e.fr = 22; }      // 2 + 2 + 2 = 6 -> MEDIA
  if (fase == 3) { e.pulso = 135; e.spo2 = 90; e.fr = 26; }      // 3 + 3 + 3 = 9 -> ALTA
  return e;
}

// ============================================================
// Persistencia: el nivel solo cambia si se sostiene
// ============================================================

NivelAlarma GestorAlarmas::aplicarPersistencia(NivelAlarma nuevo) {
  if (nuevo == nivelActual) {
    segCandidato = 0;
  } else {
    if (nuevo != candidato) { candidato = nuevo; segCandidato = 0; }
    segCandidato++;
    uint8_t necesario = (candidato > nivelActual) ? ESCALAR_SEG : DESESCALAR_SEG;
    if (segCandidato >= necesario) { nivelActual = candidato; segCandidato = 0; }
  }
  return nivelActual;
}

// ============================================================
// Alarma técnica: sensor de SpO2 retirado (solo si estuvo en uso)
// ============================================================

void GestorAlarmas::alarmaTecnica(const Signos& s, EstadoAlarma& a) {
  if (s.estadoSpo2 == 2) spo2EnUso = true;

  bool cond = spo2EnUso && (s.estadoSpo2 == 0);
  segTecSpo2 = cond ? (segTecSpo2 < 255 ? segTecSpo2 + 1 : 255) : 0;

  a.tecSpo2 = segTecSpo2 >= TECNICA_SEG;
}

// ============================================================
// Lazo de la tarea (núcleo 0, prio 1)
// ============================================================

void GestorAlarmas::bucle() {
  TickType_t tUlt = xTaskGetTickCount();
  Signos s;

  for (;;) {
    vTaskDelayUntil(&tUlt, pdMS_TO_TICKS(1000));

#if SIMULAR_NEWS2
    EstadoAlarma a = calcularNews2(entradaSimulada());
    a.nivel   = aplicarPersistencia(a.nivel);
    a.tecSpo2 = false;
#else
    if (xQueuePeek(buzonSignos, &s, 0) != pdPASS) continue;

    EstadoAlarma a = calcularNews2(armarEntrada(s));
    a.nivel = aplicarPersistencia(a.nivel);
    alarmaTecnica(s, a);
#endif

    xQueueOverwrite(buzonAlarma, &a);
  }
}