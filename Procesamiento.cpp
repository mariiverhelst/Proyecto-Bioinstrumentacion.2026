#include "Procesamiento.h"
#include "Colas.h"

void Procesamiento::tarea(void* param) {
  static_cast<Procesamiento*>(param)->bucle();
}

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
  xQueueSendToBack(colaGraf, &p, 0);
}

// ---------------- ECG (500 Hz) ----------------

void Procesamiento::procesarEcg(float voltios, bool saturada) {
  // FR: muestra cruda en cuentas (antes de cualquier filtro visual)
  frAlimentarEcg(voltios / FACTOR_VOLT, saturada);

  if (!hayEcg) {
    biquadPrime(&ecgLP1, voltios);
    biquadPrime(&ecgLP2, voltios);
    biquadPrimeHP(&ecgHP, voltios);
    hayEcg = true;
  }
  float filtrada = biquad_process(&ecgLP2, biquad_process(&ecgLP1, voltios));
  float grafica  = biquad_process(&ecgHP, filtrada);

  if (++decimEcg >= DECIM_ECG) {
    decimEcg = 0;
#if MOSTRAR_ECG_PLANO
    enviarPunto(ID_WAVE_ECG, ecgPlano ? 0.0f : grafica);
#else
    enviarPunto(ID_WAVE_ECG, grafica);
#endif
  }

  if (!tEcgIni) { tiempoEcgUs = micros(); tEcgIni = true; }
  else          { tiempoEcgUs += PERIODO_ECG_US; }

  uint32_t idx = presion.agregarMuestra(grafica, ppgGraficaUlt, ppgFiltradaUlt);

  pt_process_t(&ptEcg, voltios * PT_CUENTAS_POR_V_ECG, tiempoEcgUs);
  uint32_t b = pt_beats(&ptEcg);
  if (b != beatsEcgPrev) {
    beatsEcgPrev = b;
    tUltLatEcg = xTaskGetTickCount();
    ecgPlano = false;
    if (s.estadoSpo2 == 2) {
      presion.nuevoLatido(idx, ppgDC, pirSpo2);
    }
  }
}

// ---------------- PPG (250 Hz) ----------------

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

  acumPpg += PPG_PASO;
  if (acumPpg >= PPG_DIVISOR) {
    acumPpg -= PPG_DIVISOR;
    enviarPunto(ID_WAVE_PPG, (s.estadoSpo2 != 0) ? grafica : 0.0f);   // sin sensor: línea recta
  }

  ppgDC += kDC * (filtrada - ppgDC);

  if (!tPpgIni) { tiempoPpgUs = micros(); tPpgIni = true; }
  else          { tiempoPpgUs += PERIODO_PPG_US; }

  pt_process_t(&ptPpg, (filtrada - ppgDC) * PT_CUENTAS_POR_V_PPG, tiempoPpgUs);
  uint32_t b = pt_beats(&ptPpg);
  if (b != beatsPpgPrev) {
    beatsPpgPrev = b;
    tUltLatPpg = xTaskGetTickCount();
  }
}

// ---------------- Estado del sensor de SpO2 ----------------

void Procesamiento::procesarEstadoSpo2(uint8_t estado) {
  TickType_t tAhora = xTaskGetTickCount();
  if (estadoLib != 0 && estado == 0) tPpgAusente = tAhora;

  if (estadoLib == 0 && estado != 0 &&
      (tAhora - tPpgAusente) > pdMS_TO_TICKS(PPG_REINICIO_MS)) {
    iniciarPTPpg();
    tPpgIni = false;
    tReinicioPpg = tAhora;
    presion.reiniciar();                          // recalibra la PA con la última referencia
  }

  estadoLib = estado;
  s.estadoSpo2 = sinPulso ? 0 : estadoLib;
  if (s.estadoSpo2 != 2) s.spo2 = 0;
  if (s.estadoSpo2 == 0) s.pr = 0;
}

// ---------------- Una vez por segundo ----------------

void Procesamiento::cadaSegundo(TickType_t ahora) {
  // ---- Calibración de PA ----
  CalPA cal;
  if (xQueueReceive(colaCal, &cal, 0) == pdPASS) {
    presion.calibrar(cal.sbp, cal.dbp);
  }

  // ---- FC (ECG) ----
  bool sinLatidoEcg = (ahora - tUltLatEcg) > pdMS_TO_TICKS(TIMEOUT_LATIDO_MS);
  ecgPlano = sinLatidoEcg;
  if (sinLatidoEcg) {
    s.fc = 0;
    if (!ecgPerdido) {
      ecgPerdido = true;
      tEcgPerdido = ahora;
    } else if (!ecgReiniciado && (ahora - tEcgPerdido) > pdMS_TO_TICKS(ECG_REINICIO_MS)) {
      iniciarPTEcg();
      ecgReiniciado = true;
    }
  } else {
    ecgPerdido = false;
    ecgReiniciado = false;
    int bpm = (int)(pt_bpm(&ptEcg) + 0.5f);
    if (bpm >= 40 && bpm <= 220) s.fc = bpm;
  }

  // ---- PR (PPG) ----
  bool sinLatidoPpg = (ahora - tUltLatPpg) > pdMS_TO_TICKS(TIMEOUT_LATIDO_MS);
  int prInterno = 0;
  if (sinLatidoPpg) {
    if ((ahora - tReinicioPpg) > pdMS_TO_TICKS(PPG_REAPRENDER_MS)) {
      iniciarPTPpg();
      tPpgIni = false;
      tReinicioPpg = ahora;
    }
  } else {
    int bpm = (int)(pt_bpm(&ptPpg) + 0.5f);
    if (bpm >= 40 && bpm <= 220) prInterno = bpm;
  }

  // ---- Estado del sensor ----
  sinPulso = (estadoLib == 2) && sinLatidoPpg;

  s.estadoSpo2 = sinPulso ? 0 : estadoLib;
  s.pr   = (s.estadoSpo2 != 0) ? prInterno : 0;
  s.spo2 = (s.estadoSpo2 == 2) ? spo2Lib   : 0;

  // ---- Presión arterial ----
  if (s.estadoSpo2 != 2 || s.fc == 0) {
    s.sbp = 0;
    s.dbp = 0;
  } else {
    s.sbp = presion.sistolica();
    s.dbp = presion.diastolica();
  }
  s.pas = (s.sbp > 0 && presion.calculada()) ? presion.sistolica() : -1;

  // ---- Frecuencia respiratoria ----
  frActualizar();

  xQueueOverwrite(buzonSignos, &s);
}

// ============================================================
// Frecuencia respiratoria
// ============================================================

void Procesamiento::frAlimentarEcg(float cuentas, bool saturada) {
  if (frDeshabilitado) return;

  if (frCalibrando) {
    if (frCalMuestras == 0) frCalInicioUs = micros();
    frCalMuestras++;
    uint32_t dt = micros() - frCalInicioUs;
    if (dt >= FR_CAL_MS * 1000UL) {
      float fsMedida = (float)frCalMuestras * 1.0e6f / (float)dt;
      if (fsMedida < 0.8f * FS_ECG_HZ || fsMedida > 1.25f * FS_ECG_HZ) {
        frDeshabilitado = true;
        return;
      }
      frEst.begin(fsMedida, 2);
      frCalibrando = false;
      frTbInicioUs = micros();
      frTbMuestras = 0;
    }
    return;
  }

  frTbMuestras++;
  bool nuevaVentana = frEst.pushEcg(cuentas, saturada);

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
    xQueueReceive(colaMsg, &m, portMAX_DELAY);

    switch (m.tipo) {
      case MSG_ECG:         procesarEcg(m.v, m.a != 0); break;
      case MSG_PPG:         procesarPpg(m.v, m.a != 0); break;
      case MSG_ESTADO_SPO2: procesarEstadoSpo2(m.a);    break;
      case MSG_SPO2:
        spo2Lib = m.a;
        s.spo2 = (s.estadoSpo2 == 2) ? spo2Lib : 0;
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