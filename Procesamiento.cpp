#include "Procesamiento.h"
#include "Colas.h"

void Procesamiento::tarea(void* param) {
  static_cast<Procesamiento*>(param)->bucle();
}

// ============================================================
// Inicialización (se llama al inicio de la tarea)
// ============================================================

void Procesamiento::inicializar() {
  biquad_lowpass (&ecgLP1, FS_ECG_HZ, ECG_FC_HZ,    ECG_Q);
  biquad_lowpass (&ecgLP2, FS_ECG_HZ, ECG_FC_HZ,    ECG_Q);
  biquad_highpass(&ecgHP,  FS_ECG_HZ, ECG_HP_FC_HZ, ECG_Q);

  biquad_lowpass (&ppgLP1, FS_PPG_HZ, PPG_FC_HZ,    PPG_Q);
  biquad_lowpass (&ppgLP2, FS_PPG_HZ, PPG_FC_HZ,    PPG_Q);
  biquad_highpass(&ppgHP,  FS_PPG_HZ, PPG_HP_FC_HZ, PPG_Q);

  kDC = 2.0f * PI * PT_DC_FC_HZ / FS_PPG_HZ;

  iniciarPTEcg();
  iniciarPTPpg();

  tInfo = xTaskGetTickCount();
}

void Procesamiento::iniciarPTEcg() {
  pt_init(&ptEcg, FS_ECG_HZ);
  beatsEcgPrev = 0;
}

void Procesamiento::iniciarPTPpg() {
  pt_init(&ptPpg, FS_PPG_HZ);
  biquad_highpass(&ptPpg.hp, FS_PPG_HZ, PT_HP_FC_PPG, PT_Q_PPG);
  biquad_lowpass (&ptPpg.lp, FS_PPG_HZ, PT_LP_FC_PPG, PT_Q_PPG);
  beatsPpgPrev = 0;
}

void Procesamiento::enviarPunto(uint8_t id, float v) {
  PuntoGraf p = {id, v};
  xQueueSendToBack(colaGraf, &p, 0);   // si la Nextion se atrasa, se descarta el punto
}

// ============================================================
// Muestra de ECG (500 Hz)
// ============================================================

void Procesamiento::procesarEcg(float voltios, bool saturada) {
  // FR: muestra cruda en cuentas (antes de cualquier filtro visual)
  frAlimentarEcg(voltios / FACTOR_VOLT, saturada);

  if (!hayEcg) {                       // arranque de los filtros en régimen permanente
    biquadPrime(&ecgLP1, voltios);
    biquadPrime(&ecgLP2, voltios);
    biquadPrimeHP(&ecgHP, voltios);
    hayEcg = true;
  }
  float filtrada = biquad_process(&ecgLP2, biquad_process(&ecgLP1, voltios));
  float grafica  = biquad_process(&ecgHP, filtrada);

  if (++decimEcg >= DECIM_ECG) {       // 500 Hz -> 100 Hz
    decimEcg = 0;
    enviarPunto(ID_WAVE_ECG, grafica);
  }

  if (!tEcgIni) { tiempoEcgUs = micros(); tEcgIni = true; }
  else          { tiempoEcgUs += PERIODO_ECG_US; }

  // Presión arterial: guarda la muestra junto con la última PPG
  uint32_t idx = presion.agregarMuestra(grafica, ppgGraficaUlt, ppgFiltradaUlt);

  // Pan-Tompkins ECG: latido nuevo cuando sube el contador
  pt_process_t(&ptEcg, voltios * PT_CUENTAS_POR_V_ECG, tiempoEcgUs);
  uint32_t bE = pt_beats(&ptEcg);
  if (bE != beatsEcgPrev) {
    beatsEcgPrev = bE;
    tUltLatEcg = xTaskGetTickCount();
    if (s.estadoSpo2 == 2) {           // PA: solo con SpO2 válida
      presion.nuevoLatido(idx, ppgDC, pirSpo2);
    }
  }
}

// ============================================================
// Muestra de PPG (250 Hz)
// ============================================================

void Procesamiento::procesarPpg(float voltios, bool saturada) {
  // FR: muestra cruda en cuentas (conserva la línea base respiratoria)
  frAlimentarPpg(voltios / FACTOR_VOLT, saturada);

  if (!hayPpg) {
    biquadPrime(&ppgLP1, voltios);
    biquadPrime(&ppgLP2, voltios);
    biquadPrimeHP(&ppgHP, voltios);
    ppgDC = voltios;
    hayPpg = true;
  }
  float filtrada = biquad_process(&ppgLP2, biquad_process(&ppgLP1, voltios));
  float grafica  = biquad_process(&ppgHP, filtrada);

  ppgGraficaUlt  = grafica;
  ppgFiltradaUlt = filtrada;

  acumPpg += PPG_PASO;                 // 250 Hz -> 100 Hz
  if (acumPpg >= PPG_DIVISOR) {
    acumPpg -= PPG_DIVISOR;
    enviarPunto(ID_WAVE_PPG, grafica);
  }

  ppgDC += kDC * (filtrada - ppgDC);   // seguidor lento del offset

  if (!tPpgIni) { tiempoPpgUs = micros(); tPpgIni = true; }
  else          { tiempoPpgUs += PERIODO_PPG_US; }

  pt_process_t(&ptPpg, (filtrada - ppgDC) * PT_CUENTAS_POR_V_PPG, tiempoPpgUs);
  uint32_t bP = pt_beats(&ptPpg);
  if (bP != beatsPpgPrev) {
    beatsPpgPrev = bP;
    tUltLatPpg = xTaskGetTickCount();
  }
}

// ============================================================
// Estado del sensor de SpO2 (1 vez por segundo desde Adquisicion)
// ============================================================

void Procesamiento::procesarEstadoSpo2(uint8_t estado) {
  TickType_t tAhora = xTaskGetTickCount();
  if (estadoLib != 0 && estado == 0) tPpgAusente = tAhora;      // se fue el sensor

  // Reiniciar solo si el sensor estuvo ausente un tiempo real (no por un parpadeo)
  if (estadoLib == 0 && estado != 0 &&
      (tAhora - tPpgAusente) > pdMS_TO_TICKS(PPG_REINICIO_MS)) {
    iniciarPTPpg();
    tPpgIni = false;
    presion.reiniciar();                // recalibra la PA con la última referencia
  }

  estadoLib = estado;
  s.estadoSpo2 = sinPulso ? 0 : estadoLib;
  if (s.estadoSpo2 != 2) { s.spo2 = 0; s.pr = 0; }
}

// ============================================================
// Una vez por segundo: FC, PR, estado SpO2, PA, FR, comando CAL
// ============================================================

void Procesamiento::cadaSegundo(TickType_t ahora) {
  // ---- Calibración PA pedida desde la pantalla ----
  CalPA cal;
  if (xQueueReceive(colaCal, &cal, 0) == pdPASS) {
    presion.calibrar(cal.sbp, cal.dbp);
  }

  // ---- FC (ECG) ----
  bool sinLatidoEcg = USAR_TIMEOUT_LATIDO &&
                      (ahora - tUltLatEcg) > pdMS_TO_TICKS(TIMEOUT_LATIDO_MS);
  if (sinLatidoEcg) {
    s.fc = 0;                                          // se oculta mientras falte el ECG
    if (!ecgPerdido) {
      ecgPerdido = true;
      tEcgPerdido = ahora;
    } else if (!ecgReiniciado &&
               (ahora - tEcgPerdido) > pdMS_TO_TICKS(ECG_REINICIO_MS)) {
      iniciarPTEcg();                                  // ausencia larga: reaprender
      ecgReiniciado = true;
    }
  } else {
    ecgPerdido = false;
    ecgReiniciado = false;
    int bpmEcg = (int)(pt_bpm(&ptEcg) + 0.5f);
    if (bpmEcg >= 40 && bpmEcg <= 220) s.fc = bpmEcg;
  }

  // ---- PR interno: con sensor colocado (estado 1 o 2) ----
  bool sinLatidoPpg = USAR_TIMEOUT_LATIDO &&
                      (ahora - tUltLatPpg) > pdMS_TO_TICKS(TIMEOUT_LATIDO_MS);
  int prInterno = 0;
  if (estadoLib != 0 && !sinLatidoPpg) {
    int bpmPpg = (int)(pt_bpm(&ptPpg) + 0.5f);
    if (bpmPpg >= 40 && bpmPpg <= 220) prInterno = bpmPpg;
  }

  // ---- Sin pulsos en la PPG durante 4 s = no hay dedo ----
  sinPulso = (estadoLib == 2) && sinLatidoPpg;

  s.estadoSpo2 = sinPulso ? 0 : estadoLib;
  s.pr   = (s.estadoSpo2 != 0) ? prInterno : 0;   // PR desde que se detecta el dedo
  s.spo2 = (s.estadoSpo2 == 2) ? spo2Lib   : 0;   // SpO2 solo con señal válida

  // ---- Presión arterial (necesita ECG y PPG válidas) ----
  if (s.estadoSpo2 != 2 || s.fc == 0) {
    s.sbp = 0;
    s.dbp = 0;
  } else {
    s.sbp = presion.sistolica();          // base mientras arranca
    s.dbp = presion.diastolica();
  }
  // Para NEWS2 solo cuenta la PA realmente calculada, no la base
  s.pas = (s.sbp > 0 && presion.calculada()) ? presion.sistolica() : -1;

  // ---- Frecuencia respiratoria estimada ----
  frActualizar();

  // ---- Temperatura: aquí se asignará s.temp cuando exista el sensor ----
  s.temp = -1.0f;

  xQueueOverwrite(buzonSignos, &s);
}

// ============================================================
// Frecuencia respiratoria
// ============================================================

// Mide la tasa real de ECG (el AFE usa reloj interno) y luego alimenta el estimador
void Procesamiento::frAlimentarEcg(float cuentas, bool saturada) {
  if (frDeshabilitado) return;

  if (frCalibrando) {
    if (frCalMuestras == 0) frCalInicioUs = micros();
    frCalMuestras++;
    uint32_t dt = micros() - frCalInicioUs;
    if (dt >= FR_CAL_MS * 1000UL) {
      float fsMedida = (float)frCalMuestras * 1.0e6f / (float)dt;
      if (fsMedida < 0.8f * FS_ECG_HZ || fsMedida > 1.25f * FS_ECG_HZ) {
        frDeshabilitado = true;         // el AFE no está en 500 Hz / ECG,ECG,PPG
        return;
      }
      frEst.begin(fsMedida, 2);         // 2 muestras de ECG por cada muestra de PPG
      frCalibrando = false;
      frTbInicioUs = micros();
      frTbMuestras = 0;
    }
    return;
  }

  frTbMuestras++;
  bool nuevaVentana = frEst.pushEcg(cuentas, saturada);

  // Cada 20 s se corrige la deriva lenta del reloj del AFE
  uint32_t dt = micros() - frTbInicioUs;
  if (dt >= 20000000UL) {
    frEst.setTimeBase((float)frTbMuestras * 1.0e6f / (float)dt);
    frTbInicioUs = micros();
    frTbMuestras = 0;
  }

#if DEBUG_FR_SERIAL1
  if (nuevaVentana) {
    const RespRate::Result& w = frEst.lastWindow();
    char b[140];
    snprintf(b, sizeof(b),
             "FR st=%d raw=%.1f f=[%.1f %.1f %.1f %.1f %.1f] q=[%.2f %.2f %.2f %.2f %.2f] mot=0x%X\r\n",
             (int)w.status, w.rrRaw,
             w.featBpm[0], w.featBpm[1], w.featBpm[2], w.featBpm[3], w.featBpm[4],
             w.featQ[0], w.featQ[1], w.featQ[2], w.featQ[3], w.featQ[4], (unsigned)w.reasons);
    Serial1.print(b);
  }
#else
  (void)nuevaVentana;
#endif
}

void Procesamiento::frAlimentarPpg(float cuentas, bool saturada) {
  if (!frCalibrando && !frDeshabilitado) frEst.pushPpg(cuentas, saturada);
}

// Llena s.fr (-1 = sin valor) y s.frEstado
void Procesamiento::frActualizar() {
  if (frDeshabilitado) { s.fr = -1; s.frEstado = FR_EST_CONFIG;    return; }
  if (frCalibrando)    { s.fr = -1; s.frEstado = FR_EST_CAL_RELOJ; return; }

  RespRate::Result r = frEst.snapshot();
  s.frEstado = (uint8_t)r.status;
  s.fr = r.hasValue ? (int)(r.rrBpm + 0.5f) : -1;
}

// ============================================================
// Lazo de la tarea (núcleo 1, prio 3)
// ============================================================

void Procesamiento::bucle() {
  inicializar();
  Msg m;

  for (;;) {
    xQueueReceive(colaMsg, &m, portMAX_DELAY);       // duerme si no hay datos

    switch (m.tipo) {
      case MSG_ECG:         procesarEcg(m.v, m.a != 0);  break;
      case MSG_PPG:         procesarPpg(m.v, m.a != 0);  break;
      case MSG_ESTADO_SPO2: procesarEstadoSpo2(m.a);     break;

      case MSG_SPO2:
        spo2Lib = m.a;
        s.spo2 = (s.estadoSpo2 == 2) ? spo2Lib : 0;
        xQueueOverwrite(buzonSignos, &s);
        break;

      case MSG_PIR:     pirSpo2        = m.v;                  break;
      case MSG_CAL_R:   s.calR100      = (int)(m.v * 100.0f);  break;
      case MSG_CAL_PI:  s.calPi100     = (int)(m.v * 100.0f);  break;
      case MSG_CAL_DC:  s.calDcK       = (int)(m.v / 1000.0f); break;
      case MSG_CAL_EST: s.calEstadoLib = m.a;                  break;
    }

    TickType_t ahora = xTaskGetTickCount();
    if (ahora - tInfo >= pdMS_TO_TICKS(PERIODO_INFO_MS)) {
      tInfo = ahora;
      cadaSegundo(ahora);
    }
  }
}