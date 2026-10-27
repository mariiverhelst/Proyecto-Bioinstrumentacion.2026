#include "Adquisicion.h"
#include "Colas.h"

// ============================================================
// Inicialización
// ============================================================

void Adquisicion::begin() {
  while (!afe.configure(SCLK, MISO, MOSI, CS, AFE_RST, FIFO_RDY)) {
    delay(500);
  }
  afe.enableSpO2(true);
}

// Reinicia el FIFO: el ciclo de la librería empieza en ECG (cicloIdx = 0)
void Adquisicion::iniciarCaptura() {
  cicloIdx = 0;
  afe.enableCapture();
}

void Adquisicion::tarea(void* param) {
  static_cast<Adquisicion*>(param)->bucle();
}

// ============================================================
// Envío a Procesamiento (nunca espera: si la cola está llena, cuenta la pérdida)
// ============================================================

void Adquisicion::encolar(TipoMsg tipo, uint8_t a, float v) {
  Msg m = {tipo, a, v};
  if (xQueueSendToBack(colaMsg, &m, 0) != pdPASS) {
    portENTER_CRITICAL(&muxCont); perdidas++; portEXIT_CRITICAL(&muxCont);
  }
}

// ============================================================
// Decodificación
// ============================================================

// Palabras de 4 bytes: LSB, MID, MSB, TAG
void Adquisicion::procesarPaquete(const uint8_t* datos, size_t longitud) {
  for (size_t i = 0; i + 4 <= longitud; i += 4) {
    uint32_t valor24 = (uint32_t)datos[i] |
                       ((uint32_t)datos[i + 1] << 8) |
                       ((uint32_t)datos[i + 2] << 16);
    uint8_t tag = datos[i + 3];

    switch (tag) {
      case 0x02:                       // cabecera: NO reinicia el ciclo
      case 0x03:                       // POINTER_DIFF: no se usa
        break;
      case 0x05: procesarMuestra(valor24); break;
      case 0x06: spo2Estado = datos[i];    break;   // se procesa al llegar 0x0A
      case 0x07: acIr  = valor24;          break;
      case 0x08: dcIr  = valor24;          break;
      case 0x09: acRed = valor24;          break;
      case 0x0A: dcRed = valor24; procesarSpO2(); break;
    }
  }
}

// Muestra del FIFO: complemento a 2, voltios y separación ECG / PPG
void Adquisicion::procesarMuestra(uint32_t valor24) {
  int32_t raw = (int32_t)valor24;
  if (raw & 0x00800000L) raw -= 0x01000000L;
  float voltios = raw * FACTOR_VOLT;

  // Saturación según el manual del AFE: |raw| fuera de +-2^21
  uint8_t sat = ((raw >= 2097152L) || (raw < -2097152L)) ? 1 : 0;

  if (cicloIdx == 2) encolar(MSG_PPG, sat, voltios);
  else               encolar(MSG_ECG, sat, voltios);

  cicloIdx = (cicloIdx + 1) % 3;
}

// ============================================================
// Validación de SpO2 (1 vez por segundo)
// ============================================================

void Adquisicion::procesarSpO2() {
  float R = 0.0f, pi = 0.0f;
  bool datosOk = (dcRed > 0 && dcIr > 0 && acIr > 0);
  if (datosOk) {
    R  = ((float)acRed / dcRed) / ((float)acIr / dcIr);
    pi = 100.0f * (float)acIr / (float)dcIr;
  }

#if SPO2_VALIDAR
  bool plausible = datosOk &&
                   R  >= SPO2_R_MIN  && R  <= SPO2_R_MAX &&
                   pi >= SPO2_PI_MIN && pi <= SPO2_PI_MAX;
#if SPO2_VALIDAR_DC
  plausible = plausible && dcIr >= SPO2_DC_MIN && dcIr <= SPO2_DC_MAX;
#endif
#else
  bool plausible = datosOk;
#endif

  // ---- Estado efectivo con histéresis ----
  uint8_t efectivo;
  if (spo2Estado == 2) {
    if (plausible) {
      segInval = 0;
      if (segVal < 255) segVal++;
      if (segVal >= SPO2_SEG_VALIDO) validada = true;
      ultimoR = R;
    } else {
      segVal = 0;
      if (segInval < 255) segInval++;
      if (segInval >= SPO2_SEG_INVALIDO) validada = false;
    }
    if (validada)                           efectivo = 2;   // válida (o retenida unos s)
    else if (segInval >= SPO2_SEG_INVALIDO) efectivo = 0;   // falla sostenida
    else                                    efectivo = 1;   // confirmando
  } else {
    segVal = segInval = 0;
    validada = false;
    efectivo = spo2Estado;                                  // 0 o 1 según la librería
  }

  encolar(MSG_CAL_EST, spo2Estado, 0.0f);
  encolar(MSG_ESTADO_SPO2, efectivo, 0.0f);

  // ---- PIR para la presión arterial (solo con lectura actual plausible) ----
  float pir = 0.0f;
  if (efectivo == 2 && plausible && dcIr > 100 && acIr < dcIr) {
    pir = ((float)dcIr + 0.5f * (float)acIr) /
          ((float)dcIr - 0.5f * (float)acIr);
  }
  encolar(MSG_PIR, 0, pir);

  // ---- SpO2: si la lectura actual no es plausible se retiene la última ----
  int spo2 = 0;
  if (efectivo == 2) {
    float Ruso = plausible ? R : ultimoR;
    float sv = 110.0f - 25.0f * Ruso;
    if (sv < 0.0f)   sv = 0.0f;
    if (sv > 100.0f) sv = 100.0f;
    spo2 = (int)(sv + 0.5f);
  }
  encolar(MSG_SPO2, (uint8_t)spo2, 0.0f);

  // ---- Valores crudos para el modo calibración ----
  encolar(MSG_CAL_R,  0, R);
  encolar(MSG_CAL_PI, 0, pi);
  encolar(MSG_CAL_DC, 0, (float)dcIr);
}

// ============================================================
// Lazo de la tarea (núcleo 1, prio 5)
// ============================================================

void Adquisicion::bucle() {
  for (;;) {
    if (!afe.ready()) {              // ready() ejecuta capture() y lee el FIFO
      vTaskDelay(1);                 // cede la CPU 1 ms
      continue;
    }
    procesarPaquete(afe.getData(), afe.getBytesToSend());
  }
}