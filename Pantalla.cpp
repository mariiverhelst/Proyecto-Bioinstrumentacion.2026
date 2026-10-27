#include "Pantalla.h"
#include "Colas.h"
#include "RespRateEstimator.h"

// ------------------------------------------------------------
// Texto del estado de la FR (solo se usa si USAR_TEXTO_ESTADO_FR = 1)
// ------------------------------------------------------------
static const char* textoEstadoFR(uint8_t e) {
  switch (e) {
    case FR_EST_CAL_RELOJ:        return "calentando";
    case FR_EST_CONFIG:           return "config?";
    case RespRate::ST_WARMUP:     return "calentando";
    case RespRate::ST_OK_HIGH:    return "estimada";
    case RespRate::ST_OK_MEDIUM:  return "est 1 sens";
    case RespRate::ST_HELD:       return "retenida";
    case RespRate::ST_UNRELIABLE: return "no fiable";
    default:                      return "sin senal";
  }
}

// ============================================================
// Inicialización
// ============================================================

void Pantalla::begin() {
  comando("page 0");
  delay(50);
  comando("cle 1,0");
  comando("cle 2,0");
  comando("vNivel.val=0");          // barras en azul al arrancar
  delay(50);
}

void Pantalla::tarea(void* param) {
  static_cast<Pantalla*>(param)->bucle();
}

// ============================================================
// Comandos Nextion (SIN espacios dentro de los parámetros)
// ============================================================

void Pantalla::enviar(char* buffer, int longitud) {
  buffer[longitud++] = 0xFF;
  buffer[longitud++] = 0xFF;
  buffer[longitud++] = 0xFF;
  Serial.write((const uint8_t*)buffer, longitud);
}

void Pantalla::comando(const char* cmd) {
  char buffer[48];
  int longitud = snprintf(buffer, sizeof(buffer) - 3, "%s", cmd);
  enviar(buffer, longitud);
}

void Pantalla::add(uint8_t id, uint8_t valor) {
  char buffer[32];
  int longitud = snprintf(buffer, sizeof(buffer) - 3, "add %u,%u,%u", id, CANAL, valor);
  enviar(buffer, longitud);
}

void Pantalla::numero(const char* objeto, int valor) {
  char buffer[40];
  int longitud = snprintf(buffer, sizeof(buffer) - 3, "%s.val=%d", objeto, valor);
  enviar(buffer, longitud);
}

// ============================================================
// Autoescalado (solo visual)
// ============================================================

uint8_t Pantalla::escalar(EscalaAutomatica& escala, float valor, float spanMinimo, bool invertir) {
  float a = fabsf(valor);

  if (a > escala.bloqueMaximo) escala.bloqueMaximo = a;
  escala.cantidad++;

  // Arranque: aprende la amplitud (línea centrada mientras tanto)
  if (!escala.inicializada) {
    if (escala.cantidad >= ESCALA_APRENDER) {
      escala.maximo = escala.bloqueMaximo;
      escala.inicializada = true;
      escala.bloqueMaximo = 0.0f;
      escala.cantidad = 0;
    }
    return ALTO_WAVE / 2;
  }

  // Si la señal crece, la escala sube al instante
  if (a > escala.maximo) escala.maximo = a;

  // Si la señal se achica, la escala baja cada bloque
  if (escala.cantidad >= ESCALA_BLOQUE) {
    escala.maximo += ESCALA_FACTOR * (escala.bloqueMaximo - escala.maximo);
    escala.bloqueMaximo = 0.0f;
    escala.cantidad = 0;
  }

  float amp = escala.maximo;
  if (amp < 0.5f * spanMinimo) amp = 0.5f * spanMinimo;
  amp *= 1.1f;   // margen del 10 %

  int salida = (int)(ALTO_WAVE * 0.5f + (valor / amp) * ALTO_WAVE * 0.5f + 0.5f);
  salida = constrain(salida, 0, ALTO_WAVE);
  if (invertir) salida = ALTO_WAVE - salida;

  return (uint8_t)salida;
}

// ============================================================
// Alarma visual: misma prioridad que el sonido
// ALTO > MEDIO > TÉCNICA > BAJO
// ============================================================

uint8_t Pantalla::nivelVisual(const EstadoAlarma& a) {
  if (a.nivel == ALARMA_ALTA)  return VIS_ALTO;
  if (a.nivel == ALARMA_MEDIA) return VIS_MEDIO;
  if (a.tecSpo2)               return VIS_TECNICA;
  return VIS_BAJO;
}

// ============================================================
// Lee "CAL <SBP> <DBP>" sin bloquear y lo manda a Procesamiento
// ============================================================

void Pantalla::leerComandos() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      cmdBuf[cmdIdx] = '\0';
      CalPA cal;
      if (sscanf(cmdBuf, "CAL %f %f", &cal.sbp, &cal.dbp) == 2 &&
          cal.sbp > cal.dbp &&
          cal.sbp >= 60.0f && cal.sbp <= 250.0f &&
          cal.dbp >= 30.0f && cal.dbp <= 150.0f) {
        xQueueSendToBack(colaCal, &cal, 0);
      }
      cmdIdx = 0;
    } else if (c >= 32 && c <= 126) {
      if (cmdIdx < sizeof(cmdBuf) - 1) cmdBuf[cmdIdx++] = c;
    } else {
      cmdIdx = 0;   // bytes de eventos Nextion (0x65, 0xFF...) reinician el buffer
    }
  }
}

// ============================================================
// Números (normal o modo calibración de SpO2)
// ============================================================

void Pantalla::mostrarNumeros(const Signos& s) {
#if CALIBRAR_SPO2
  numero("nHR",   s.calR100);        // R x 100
  numero("nPR",   s.calPi100);       // índice de perfusión x 100 (%)
  numero("nSpO2", s.calDcK);         // DC IR en miles de cuentas
  numero("nSBP",  s.calEstadoLib);   // estado de la librería (0, 1, 2)
  numero("nDBP",  s.estadoSpo2);     // estado efectivo        (0, 1, 2)
#else
  numero("nHR",   s.fc);
  numero("nPR",   s.pr);
  numero("nSpO2", s.spo2);
  numero("nSBP",  s.sbp);
  numero("nDBP",  s.dbp);
#endif

  // FR: 0 en pantalla = todavía sin valor fiable
  numero(NEX_OBJ_FR, (s.fr > 0) ? s.fr : 0);
#if USAR_TEXTO_ESTADO_FR
  char b[40];
  snprintf(b, sizeof(b), "%s.txt=\"%s\"", NEX_OBJ_FR_TXT, textoEstadoFR(s.frEstado));
  comando(b);
#else
  (void)textoEstadoFR;
#endif
}

// ============================================================
// Lazo de la tarea (núcleo 0, prio 2)
// ============================================================

void Pantalla::bucle() {
  TickType_t tUlt = xTaskGetTickCount();
  uint32_t ciclo = 0;
  PuntoGraf p;
  Signos s;
  EstadoAlarma a;

  for (;;) {
    vTaskDelayUntil(&tUlt, pdMS_TO_TICKS(PERIODO_GRAFICA_MS));

    // 1) Ondas
    while (xQueueReceive(colaGraf, &p, 0) == pdPASS) {
      if (p.id == ID_WAVE_ECG)
        add(ID_WAVE_ECG, escalar(escalaEcg, p.v, SPAN_MIN_ECG_V, INVERTIR_ECG));
      else
        add(ID_WAVE_PPG, escalar(escalaPpg, p.v, SPAN_MIN_PPG_V, INVERTIR_PPG));
    }

    // 2) Alarma visual: solo se envía cuando cambia
    if (xQueuePeek(buzonAlarma, &a, 0) == pdPASS) {
      uint8_t vis = nivelVisual(a);
      if (vis != visEnviado) {
        numero(NEX_NIVEL_ALARMA, vis);
        visEnviado = vis;
      }
    }

    // 3) Números, 1 vez por segundo
    if (++ciclo % (PERIODO_INFO_MS / PERIODO_GRAFICA_MS) == 0 &&
        xQueuePeek(buzonSignos, &s, 0) == pdPASS) {
      mostrarNumeros(s);

      // Reenvío periódico del nivel (por si la Nextion se reinició)
      if (ciclo % (5 * PERIODO_INFO_MS / PERIODO_GRAFICA_MS) == 0) visEnviado = 255;
    }

    // 4) Comando CAL
    leerComandos();
  }
}