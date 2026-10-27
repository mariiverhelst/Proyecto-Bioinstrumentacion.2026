#include "Sonido.h"
#include "Colas.h"

// ============================================================
// Patrones sonoros
// ============================================================

static const Paso PATRON_ALTA[] = {          // 3 + 2 pulsos rápidos, 2 s de silencio
  {ALARMA_FREQ_HZ, 150}, {0, 100}, {ALARMA_FREQ_HZ, 150}, {0, 100}, {ALARMA_FREQ_HZ, 150}, {0, 350},
  {ALARMA_FREQ_HZ, 150}, {0, 100}, {ALARMA_FREQ_HZ, 150}, {0, 2000}
};

static const Paso PATRON_MEDIA[] = {         // 3 pulsos, 5 s de silencio
  {ALARMA_FREQ_HZ, 200}, {0, 200}, {ALARMA_FREQ_HZ, 200}, {0, 200}, {ALARMA_FREQ_HZ, 200}, {0, 5000}
};

static const Paso PATRON_TECNICA[] = {       // 2 pulsos cortos, 10 s de silencio
  {ALARMA_FREQ_HZ, 100}, {0, 150}, {ALARMA_FREQ_HZ, 100}, {0, 10000}
};

#define N_PASOS(p) (sizeof(p) / sizeof(Paso))

// ============================================================

void Sonido::begin() {
  ledcAttach(PIN_BUZZER, BUZZER_FREQ_HZ, 8);
  ledcWriteTone(PIN_BUZZER, 0);
}

void Sonido::tarea(void* param) {
  static_cast<Sonido*>(param)->bucle();
}

ModoSonido Sonido::modoActual() {
  EstadoAlarma a;
  if (xQueuePeek(buzonAlarma, &a, 0) != pdPASS) return SON_PULSO;
  if (a.nivel == ALARMA_ALTA)  return SON_ALTA;
  if (a.nivel == ALARMA_MEDIA) return SON_MEDIA;
  if (a.tecSpo2)               return SON_TECNICA;
  return SON_PULSO;
}

// Reproduce un patrón. Si el modo cambia a mitad, se corta y vuelve.
void Sonido::reproducir(const Paso* patron, size_t n, ModoSonido modo) {
  for (size_t i = 0; i < n; i++) {
    ledcWriteTone(PIN_BUZZER, patron[i].freq);
    uint16_t restante = patron[i].ms;
    while (restante > 0) {
      uint16_t paso = restante > 50 ? 50 : restante;
      vTaskDelay(pdMS_TO_TICKS(paso));
      restante -= paso;
      if (modoActual() != modo) { ledcWriteTone(PIN_BUZZER, 0); return; }
    }
  }
  ledcWriteTone(PIN_BUZZER, 0);
}

// Bip de pulso: FC del ECG, o PR si no hay ECG
void Sonido::bipPulso() {
  Signos s;
  int bpm = 0;
  if (xQueuePeek(buzonSignos, &s, 0) == pdPASS) bpm = (s.fc > 0) ? s.fc : s.pr;

  if (bpm < 40) {
    vTaskDelay(pdMS_TO_TICKS(100));
    return;
  }
  ledcWriteTone(PIN_BUZZER, BUZZER_FREQ_HZ);
  vTaskDelay(pdMS_TO_TICKS(BUZZER_DURACION_MS));
  ledcWriteTone(PIN_BUZZER, 0);
  vTaskDelay(pdMS_TO_TICKS(60000UL / bpm - BUZZER_DURACION_MS));
}

// ============================================================
// Lazo de la tarea (núcleo 1, prio 2)
// ============================================================

void Sonido::bucle() {
  for (;;) {
    ModoSonido modo = modoActual();
    switch (modo) {
      case SON_ALTA:    reproducir(PATRON_ALTA,    N_PASOS(PATRON_ALTA),    modo); break;
      case SON_MEDIA:   reproducir(PATRON_MEDIA,   N_PASOS(PATRON_MEDIA),   modo); break;
      case SON_TECNICA: reproducir(PATRON_TECNICA, N_PASOS(PATRON_TECNICA), modo); break;
      default:          bipPulso();                                                 break;
    }
  }
}