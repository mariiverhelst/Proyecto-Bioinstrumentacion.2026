/*
 * AFE4950 + ESP32 -> NEXTION
 * ECG + PPG + HR + PR + SpO2 + BUZZER PASIVO
 *
 * Por Serial solo se envian comandos Nextion (no usar Serial.print).
 * Nextion Editor -> Debug -> User MCU Input -> 115200 -> COM del ESP32 -> Start
 */

#include <Arduino.h>
#include <AFE4950.h>
#include "biosignals.h"

// ---------------- PINES AFE4950 ----------------
#define SCLK      18
#define MISO      19
#define MOSI      23
#define CS         5
#define AFE_RST    4
#define FIFO_RDY  16

// ---------------- BUZZER PASIVO ----------------
// Pin libre: 25, 26 o 27. Resistencia de 100-330 ohm en serie.
#define PIN_BUZZER          25
#define BUZZER_DURACION_MS  40
#define BUZZER_FREQ_HZ      2000

// ---------------- NEXTION ----------------
#define NEX_BAUD    115200
#define ID_WAVE_ECG 1
#define ID_WAVE_PPG 2
#define CANAL       0
#define ALTO_WAVE   199

// Cambiar a true si alguna onda aparece invertida
#define INVERTIR_ECG true
#define INVERTIR_PPG false

// ---------------- FRECUENCIAS DE MUESTREO ----------------
#define FS_ECG_HZ 500.0f
#define FS_PPG_HZ 250.0f

static const uint32_t PERIODO_ECG_US = 2000;
static const uint32_t PERIODO_PPG_US = 4000;

// Conversion a voltaje
static const float FACTOR_VOLT = 1.2f / 2097151.0f;

// Actualizacion Nextion
static const unsigned long PERIODO_GRAFICA_MS = 10;
static const unsigned long PERIODO_INFO_MS    = 1000;

// Filtros visuales (solo para la grafica)
#define ECG_FC_HZ    40.0f    // pasa-bajos
#define ECG_HP_FC_HZ 0.5f     // pasa-altos (quita respiracion / linea base)
#define ECG_Q        0.7071f

#define PPG_FC_HZ    8.0f     // pasa-bajos
#define PPG_HP_FC_HZ 0.5f     // pasa-altos (quita respiracion / linea base)
#define PPG_Q        0.7071f

// Pan-Tompkins para PPG
#define PT_HP_FC_PPG 0.7f
#define PT_LP_FC_PPG 5.0f
#define PT_Q_PPG     0.7071f

#define PT_CUENTAS_POR_V_PPG 20000.0f
#define PT_DC_FC_HZ          0.3f

// Escala de entrada Pan-Tompkins ECG
#define PT_CUENTAS_POR_V_ECG 100000.0f

// ---------------- OBJETOS ----------------
AFE4950 afe;

PanTompkins ptEcg;
PanTompkins ptPpg;

Biquad ecgLP1, ecgLP2, ecgHP;
Biquad ppgLP1, ppgLP2, ppgHP;

// ---------------- SECUENCIA DEL FIFO ----------------
uint8_t cicloIdx = 0;

// ---------------- ESTADO ECG ----------------
bool  hayEcg = false;
float ecgVoltios = 0.0f;
float ecgFiltrada = 0.0f;   // pasa-bajos
float ecgGrafica = 0.0f;    // pasa-bajos + pasa-altos (lo que se dibuja)
int   fcEstable = 0;

// ---------------- ESTADO PPG ----------------
bool  hayPpg = false;
float ppgVoltios = 0.0f;
float ppgFiltrada = 0.0f;   // pasa-bajos
float ppgGrafica = 0.0f;    // pasa-bajos + pasa-altos (lo que se dibuja)
float ppgDC = 0.0f;
float kDC = 0.0f;
int   prEstable = 0;

// ---------------- TIEMPOS RECONSTRUIDOS ----------------
uint32_t tiempoEcgUs = 0;
uint32_t tiempoPpgUs = 0;
bool tiempoEcgInicializado = false;
bool tiempoPpgInicializado = false;

// ---------------- SpO2 ----------------
uint8_t spo2Estado = 0;
uint8_t spo2Seg = 0;
uint32_t acIr = 0, dcIr = 0;
uint32_t acRed = 0, dcRed = 0;
bool spo2Nuevo = false;

// ---------------- AUTOESCALADO ----------------
struct EscalaAutomatica {
  float maximo;          // amplitud maxima vigente
  float bloqueMaximo;    // maxima del bloque actual
  uint16_t cantidad;
  bool inicializada;
};

EscalaAutomatica escalaEcg = {0.0f, 0.0f, 0, false};
EscalaAutomatica escalaPpg = {0.0f, 0.0f, 0, false};

static const float SPAN_MIN_ECG_V = 0.0002f;
static const float SPAN_MIN_PPG_V = 0.0020f;

// ---------------- TEMPORIZADORES ----------------
unsigned long tGrafica = 0;
unsigned long tInfo = 0;

// ---------------- ESTADO BUZZER ----------------
bool buzzerSonando = false;
unsigned long buzzerApagarEn = 0;
unsigned long proximoBip = 0;


// ============================================================
// UTILIDADES
// ============================================================

// Inicializa un biquad pasa-bajos en regimen permanente
void biquadPrime(Biquad* filtro, float x0) {
  filtro->z2 = (filtro->b2 - filtro->a2) * x0;
  filtro->z1 = (filtro->b1 - filtro->a1 + filtro->b2 - filtro->a2) * x0;
}

// Inicializa un biquad pasa-altos en regimen permanente (salida = 0)
void biquadPrimeHP(Biquad* filtro, float x0) {
  filtro->z2 = filtro->b2 * x0;
  filtro->z1 = (filtro->b1 + filtro->b2) * x0;
}

void iniciarPTEcg() {
  pt_init(&ptEcg, FS_ECG_HZ);
  tiempoEcgInicializado = false;
  fcEstable = 0;
}

void iniciarPTPpg() {
  pt_init(&ptPpg, FS_PPG_HZ);

  biquad_highpass(&ptPpg.hp, FS_PPG_HZ, PT_HP_FC_PPG, PT_Q_PPG);
  biquad_lowpass(&ptPpg.lp, FS_PPG_HZ, PT_LP_FC_PPG, PT_Q_PPG);

  tiempoPpgInicializado = false;
  prEstable = 0;
}

// Escalado automatico para la Nextion.
// El cero queda fijo en el centro de la grafica; solo se ajusta la amplitud.
uint8_t escalarSenal(EscalaAutomatica& escala, float valor, float spanMinimo, bool invertir) {
  float a = fabsf(valor);

  if (a > escala.bloqueMaximo) escala.bloqueMaximo = a;
  escala.cantidad++;

  // Primer segundo: aprende la amplitud, linea plana centrada
  if (!escala.inicializada) {
    if (escala.cantidad >= 100) {
      escala.maximo = escala.bloqueMaximo;
      escala.inicializada = true;
      escala.bloqueMaximo = 0.0f;
      escala.cantidad = 0;
    }
    return ALTO_WAVE / 2;
  }

  // Si la senal crece, la escala sube al instante
  if (a > escala.maximo) escala.maximo = a;

  // Si se achica, la escala baja suave cada 3 s
  if (escala.cantidad >= 300) {
    escala.maximo += 0.35f * (escala.bloqueMaximo - escala.maximo);
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
// NEXTION
// ============================================================

void nexEnviar(char* buffer, int longitud) {
  buffer[longitud++] = 0xFF;
  buffer[longitud++] = 0xFF;
  buffer[longitud++] = 0xFF;
  Serial.write((const uint8_t*)buffer, longitud);
}

void nexComando(const char* comando) {
  char buffer[48];
  int longitud = snprintf(buffer, sizeof(buffer) - 3, "%s", comando);
  nexEnviar(buffer, longitud);
}

void nexAdd(uint8_t id, uint8_t valor) {
  char buffer[32];
  int longitud = snprintf(buffer, sizeof(buffer) - 3, "add %u,%u,%u", id, CANAL, valor);
  nexEnviar(buffer, longitud);
}

void nexNumero(const char* objeto, int valor) {
  char buffer[40];
  int longitud = snprintf(buffer, sizeof(buffer) - 3, "%s.val=%d", objeto, valor);
  nexEnviar(buffer, longitud);
}


// ============================================================
// PROCESAMIENTO DE MUESTRAS
// ============================================================

void nuevaMuestraEcg(float voltios) {
  ecgVoltios = voltios;

  // Primera muestra: arrancar los filtros en regimen permanente
  if (!hayEcg) {
    biquadPrime(&ecgLP1, voltios);
    biquadPrime(&ecgLP2, voltios);
    biquadPrimeHP(&ecgHP, voltios);   // a la entrada del HP llega 'voltios' (LP en regimen)
    hayEcg = true;
  }

  // Pasa-bajos (suaviza) y luego pasa-altos (quita la linea base).
  // Esto se ejecuta en CADA muestra.
  ecgFiltrada = biquad_process(&ecgLP2, biquad_process(&ecgLP1, ecgVoltios));
  ecgGrafica  = biquad_process(&ecgHP, ecgFiltrada);

  // Timestamp reconstruido a 500 Hz
  if (!tiempoEcgInicializado) {
    tiempoEcgUs = micros();
    tiempoEcgInicializado = true;
  } else {
    tiempoEcgUs += PERIODO_ECG_US;
  }

  // Pan-Tompkins recibe la ECG original escalada (sin los filtros visuales)
  float entradaPT = ecgVoltios * PT_CUENTAS_POR_V_ECG;
  pt_process_t(&ptEcg, entradaPT, tiempoEcgUs);
}

void nuevaMuestraPpg(float voltios) {
  ppgVoltios = voltios;

  // Primera muestra: arrancar los filtros en regimen permanente
  if (!hayPpg) {
    biquadPrime(&ppgLP1, voltios);
    biquadPrime(&ppgLP2, voltios);
    biquadPrimeHP(&ppgHP, voltios);
    ppgDC = voltios;
    hayPpg = true;
  }

  // Pasa-bajos y luego pasa-altos. Se ejecuta en CADA muestra.
  ppgFiltrada = biquad_process(&ppgLP2, biquad_process(&ppgLP1, ppgVoltios));
  ppgGrafica  = biquad_process(&ppgHP, ppgFiltrada);

  // Seguidor lento del offset (para Pan-Tompkins)
  ppgDC += kDC * (ppgFiltrada - ppgDC);

  // Timestamp reconstruido a 250 Hz
  if (!tiempoPpgInicializado) {
    tiempoPpgUs = micros();
    tiempoPpgInicializado = true;
  } else {
    tiempoPpgUs += PERIODO_PPG_US;
  }

  float entradaPT = (ppgFiltrada - ppgDC) * PT_CUENTAS_POR_V_PPG;
  pt_process_t(&ptPpg, entradaPT, tiempoPpgUs);
}

// Decodifica palabras de 4 bytes: LSB, MID, MSB, TAG
void procesarPaquete(const uint8_t* datos, size_t longitud) {
  for (size_t i = 0; i + 4 <= longitud; i += 4) {
    uint32_t valor24 = (uint32_t)datos[i] |
                       ((uint32_t)datos[i + 1] << 8) |
                       ((uint32_t)datos[i + 2] << 16);
    uint8_t tag = datos[i + 3];

    switch (tag) {
      case 0x02:               // cabecera de paquete
        cicloIdx = 0;
        break;

      case 0x05: {             // muestra ECG o PPG
        int32_t raw = (int32_t)valor24;
        if (raw & 0x00800000L) raw -= 0x01000000L;

        float voltios = raw * FACTOR_VOLT;

        // Ciclo: 0 -> ECG, 1 -> ECG, 2 -> PPG
        if (cicloIdx == 0 || cicloIdx == 1) nuevaMuestraEcg(voltios);
        else                                nuevaMuestraPpg(voltios);

        cicloIdx = (cicloIdx + 1) % 3;
        break;
      }

      case 0x06: {             // estado del sensor oximetrico
        uint8_t nuevoEstado = datos[i];

        // Al detectar el dedo se reinicia solo el procesamiento PPG
        if (spo2Estado == 0 && nuevoEstado != 0) iniciarPTPpg();

        spo2Estado = nuevoEstado;
        spo2Seg = datos[i + 1];
        break;
      }

      case 0x07: acIr = valor24;  break;
      case 0x08: dcIr = valor24;  break;
      case 0x09: acRed = valor24; break;
      case 0x0A:
        dcRed = valor24;
        spo2Nuevo = true;
        break;
    }
  }
}

int calcularSpO2() {
  if (spo2Estado != 2 || dcRed == 0 || dcIr == 0 || acIr == 0) return 0;

  float relacionRoja = (float)acRed / (float)dcRed;
  float relacionIr   = (float)acIr  / (float)dcIr;
  float R = relacionRoja / relacionIr;

  float spo2 = 110.0f - 25.0f * R;
  if (spo2 < 0.0f)   spo2 = 0.0f;
  if (spo2 > 100.0f) spo2 = 100.0f;

  return (int)(spo2 + 0.5f);
}


// ============================================================
// BUZZER PASIVO (no bloqueante, sin delay)
// Core ESP32 3.x. Si tu core es 2.x, ver nota al final de la respuesta.
// ============================================================

void actualizarBuzzer(unsigned long ahora, int bpm) {
  // Apagar cuando se cumple la duracion
  if (buzzerSonando && (long)(ahora - buzzerApagarEn) >= 0) {
    ledcWriteTone(PIN_BUZZER, 0);
    buzzerSonando = false;
  }

  // Sin frecuencia valida: silencio
  if (bpm < 40) {
    proximoBip = ahora;
    return;
  }

  // Encender en el siguiente latido
  if (!buzzerSonando && (long)(ahora - proximoBip) >= 0) {
    ledcWriteTone(PIN_BUZZER, BUZZER_FREQ_HZ);
    buzzerSonando = true;
    buzzerApagarEn = ahora + BUZZER_DURACION_MS;
    proximoBip = ahora + 60000UL / bpm;
  }
}


// ============================================================
// BLOQUES DEL LOOP
// ============================================================

void leerAFE() {
  if (afe.ready()) {
    procesarPaquete(afe.getData(), afe.getBytesToSend());
  }
}

void actualizarGraficas(unsigned long ahora) {
  if (ahora - tGrafica < PERIODO_GRAFICA_MS) return;

  tGrafica += PERIODO_GRAFICA_MS;

  // Si hay un retraso grande, resincronizar
  if (ahora - tGrafica > 100) tGrafica = ahora;

  if (hayEcg) {
    nexAdd(ID_WAVE_ECG, escalarSenal(escalaEcg, ecgGrafica, SPAN_MIN_ECG_V, INVERTIR_ECG));
  }

  if (hayPpg) {
    nexAdd(ID_WAVE_PPG, escalarSenal(escalaPpg, ppgGrafica, SPAN_MIN_PPG_V, INVERTIR_PPG));
  }
}

void actualizarSpO2() {
  if (!spo2Nuevo) return;
  spo2Nuevo = false;
  nexNumero("nSpO2", calcularSpO2());
}

void actualizarHrPr(unsigned long ahora) {
  if (ahora - tInfo < PERIODO_INFO_MS) return;
  tInfo = ahora;

  // Frecuencia cardiaca desde ECG
  int bpmEcg = (int)(pt_bpm(&ptEcg) + 0.5f);
  if (bpmEcg >= 40 && bpmEcg <= 220) fcEstable = bpmEcg;
  nexNumero("nHR", fcEstable);

  // Frecuencia de pulso desde PPG
  int bpmPpg = (int)(pt_bpm(&ptPpg) + 0.5f);
  if (spo2Estado == 0)                    prEstable = 0;
  else if (bpmPpg >= 40 && bpmPpg <= 220) prEstable = bpmPpg;
  nexNumero("nPR", prEstable);
}

void descartarSerial() {
  while (Serial.available()) Serial.read();
}


// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(NEX_BAUD);

  // Buzzer pasivo apagado desde el inicio
  ledcAttach(PIN_BUZZER, BUZZER_FREQ_HZ, 8);
  ledcWriteTone(PIN_BUZZER, 0);

  // Filtros visuales ECG
  biquad_lowpass(&ecgLP1, FS_ECG_HZ, ECG_FC_HZ, ECG_Q);
  biquad_lowpass(&ecgLP2, FS_ECG_HZ, ECG_FC_HZ, ECG_Q);
  biquad_highpass(&ecgHP, FS_ECG_HZ, ECG_HP_FC_HZ, ECG_Q);

  // Filtros visuales PPG
  biquad_lowpass(&ppgLP1, FS_PPG_HZ, PPG_FC_HZ, PPG_Q);
  biquad_lowpass(&ppgLP2, FS_PPG_HZ, PPG_FC_HZ, PPG_Q);
  biquad_highpass(&ppgHP, FS_PPG_HZ, PPG_HP_FC_HZ, PPG_Q);

  // Coeficiente del seguidor DC de la PPG
  kDC = 2.0f * PI * PT_DC_FC_HZ / FS_PPG_HZ;

  iniciarPTEcg();
  iniciarPTPpg();

  // Configurar AFE4950
  while (!afe.configure(SCLK, MISO, MOSI, CS, AFE_RST, FIFO_RDY)) {
    delay(500);
  }

  afe.enableSpO2(true);

  // Inicializar Nextion
  nexComando("page 0");
  delay(50);
  nexComando("cle 1,0");
  nexComando("cle 2,0");
  delay(50);

  afe.enableCapture();

  tGrafica = millis();
  tInfo = millis();
}


// ============================================================
// LOOP
// ============================================================

void loop() {
  leerAFE();

  unsigned long ahora = millis();

  actualizarGraficas(ahora);
  actualizarSpO2();
  actualizarHrPr(ahora);
  actualizarBuzzer(ahora, fcEstable);
  descartarSerial();
}