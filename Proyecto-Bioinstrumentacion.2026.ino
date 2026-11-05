// AFE4950 + ESP32 -> NEXTION   (base: código del otro equipo, adaptado)
// ECG + PPG + HR + PR + SpO2 + PTT + PRESION ARTERIAL + FRECUENCIA RESPIRATORIA
// + ALARMAS NEWS2 (visual y sonora) + BLUETOOTH (ble.ino):
//   página web: números, ondas, límites, silencio, calibración de PA y simulación NEWS2
//
// Partición: Tools -> Partition Scheme -> "Huge APP (3MB No OTA/1MB SPIFFS)"

#include <Arduino.h>
#include <AFE4950.h>
#include <Preferences.h>
#include "biosignals.h"
#include "RespRateEstimator.h"

// Funciones del Bluetooth (están en ble.ino)
void bleIniciar();
void bleAgregarPunto(char tipo, float voltios);
void bleProcesarComandos();

// PINES
#define SCLK      18
#define MISO      19
#define MOSI      23
#define CS         5
#define AFE_RST    4
#define FIFO_RDY  16

// NEXTION
#define NEX_BAUD 115200
#define ID_WAVE_ECG 1
#define ID_WAVE_PPG 2
#define CANAL     0
#define ALTO_WAVE 199

// IDs de página que responde "sendme"
#define NEX_ID_PRINCIPAL  0
#define NEX_ID_INFO       1
#define NEX_ID_ALARMAS    4

// Objetos de la página pAlarm
#define NEX_LIM_FCMAX     "n0"
#define NEX_LIM_FCMIN     "n2"
#define NEX_LIM_SPO2MIN   "n1"
#define NEX_LIM_PASMAX    "n4"
#define NEX_LIM_PASMIN    "n3"
#define NEX_SILENCIO      "bt0"

// OPCIONES
#define INVERTIR_ECG true
#define INVERTIR_PPG false
#define HR_FUENTE_PROPIO 0
#define HR_FUENTE_PT     1
#define HR_FUENTE HR_FUENTE_PROPIO
#define DEBUG_NDBG 0
#define INVERTIR_PULSO_PPG 1   // <<< el valor que les funcionó

// ===================== ALARMAS =====================
#define USAR_ALARMAS         1
#define USAR_SONIDO_ALARMAS  1
#define SIMULAR_NEWS2        0      // simulación fija por código (la de la página es independiente)
#define USAR_PAS_EN_NEWS2    1
#define USAR_FR_EN_NEWS2     1
#define ALARMA_FREQ_HZ       540
#define ESCALAR_SEG          5
#define DESESCALAR_SEG       10
#define TECNICA_SEG          3
#define SPO2_DC_MIN          24000UL

// ===================== LÍMITES Y SILENCIO =====================
#define USAR_LIMITES         1      // los límites se configuran desde la página web
#define USAR_SILENCIO        1      // el silencio se activa desde la página web
#define LEER_PAG_ALARMAS     0      // 0 = no leer la página de alarmas de la Nextion
#define SILENCIO_MS          120000UL
#define LIM_FCMAX_DEF        110
#define LIM_FCMIN_DEF        50
#define LIM_SPO2MIN_DEF      94
#define LIM_PASMAX_DEF       160
#define LIM_PASMIN_DEF       90

// MUESTREO
#define FS_ECG_HZ 500.0f
#define FS_PPG_HZ 250.0f
static const uint32_t PERIODO_ECG_US = 2000;
static const uint32_t PERIODO_PPG_US = 4000;
static const float FACTOR_VOLT = 1.2f / 2097151.0f;
static const unsigned long PERIODO_GRAFICA_MS = 10;
static const unsigned long PERIODO_INFO_MS    = 1000;

// FILTROS
#define ECG_FC_HZ 40.0f
#define ECG_Q     0.7071f
#define PPG_FC_HZ 10.0f
#define PPG_Q     0.7071f
#define PPG_HP_FC_HZ 0.5f
#define PPG_HP_Q     0.7071f

// PAN-TOMPKINS
#define PT_HP_FC_PPG 0.7f
#define PT_LP_FC_PPG 5.0f
#define PT_Q_PPG     0.7071f
#define PT_CUENTAS_POR_V_PPG 20000.0f
#define PT_DC_FC_HZ           0.3f
#define PT_CUENTAS_POR_V_ECG 100000.0f

// DETECTOR ECG
#define BPM_MIN 20
#define BPM_MAX 220
#define REFRACTARIO_ECG_MUESTRAS 125
#define UMBRAL_FRACCION 0.5f
#define AMP_MIN_ECG_V 0.00005f
#define ECG_BASE_FC_HZ 0.5f
#define ENV_TAU_S 3.0f
#define N_RR 4
#define TIMEOUT_LATIDO_S 5

// PTT / PA
#define USAR_PIR 1
#define PTT_MIN_MS 150.0f
#define PTT_MAX_MS 450.0f
#define PIR_MIN 1.0f
#define PIR_MAX 1.3f
#define RR_MIN_MS 333.0f
#define RR_MAX_MS 2000.0f
#define R_ANTES_MS   60
#define R_DESPUES_MS 80
#define N_BUF 2048
#define NPROM 4
#define N_CAL 10
#define CAL_AUTO 1
#define DEBUG_PA      0
#define USAR_PIR_SPO2 1
#define CAL_AL_INICIO 1     // 1 = calibra sola al encender con SBP_REF / DBP_REF
#define SBP_REF  121.0f     // <<< sistólica del tensiómetro
#define DBP_REF   81.0f     // <<< diastólica del tensiómetro

#define MS_A_MUESTRAS(x) ((uint32_t)((x) * FS_ECG_HZ / 1000.0f))
#define BUF(buf, i) buf[(i) % N_BUF]

// OBJETOS
AFE4950 afe;
PanTompkins ptEcg;
PanTompkins ptPpg;
Biquad ecgLP1;
Biquad ecgLP2;
Biquad ecgNotch;
Biquad ecgVis1, ecgVis2;
Biquad ppgLP1;
Biquad ppgLP2;
Biquad ppgHP;
Preferences prefs;

// FR
RespRate::Estimator estimadorFR;
struct MuestraFR { int32_t raw; uint8_t esPpg; };
QueueHandle_t colaFR = NULL;
volatile int frMostrar = -1;

// ESTADO
uint8_t cicloIdx = 0;

// ECG
bool hayEcg = false;
float ecgVoltios = 0.0f;
float ecgFiltrada = 0.0f;
int fcEstable = 0;

// PPG
bool hayPpg = false;
float ppgVoltios = 0.0f;
float ppgFiltrada = 0.0f;
float ppgPulsatil = 0.0f;
float ppgDC = 0.0f;
float kDC = 0.0f;
int prEstable = 0;

// TIEMPOS
uint32_t tiempoEcgUs = 0;
uint32_t tiempoPpgUs = 0;
bool tiempoEcgInicializado = false;
bool tiempoPpgInicializado = false;

// SpO2
uint8_t spo2Estado = 0;
uint8_t spo2Seg = 0;
uint32_t acIr = 0;
uint32_t dcIr = 0;
uint32_t acRed = 0;
uint32_t dcRed = 0;
bool spo2Nuevo = false;
int spo2Actual = 0;

// DETECTOR ECG
float ecgBase = 0.0f;
float kBase = 0.0f;
float envPico = 0.0f;
float decayEnv = 0.0f;
bool sobreUmbral = false;
uint32_t muestrasEcg = 0;
uint32_t ultimoLatidoMuestra = 0;
bool hayLatidoPrevio = false;
float rrBuf[N_RR];
uint8_t rrCount = 0;
uint8_t rrIdx = 0;
float bpmPropio = 0.0f;
uint32_t ultimoLatidoValidoMs = 0;
uint16_t contadorMuestrasEcgSeg = 0;

// BUFFERS PTT
float ecgBuf[N_BUF];
float ppgBuf[N_BUF];
float dcBuf[N_BUF];

// DETECCION R
bool latidoPendiente = false;
uint32_t muestraDeteccion = 0;
uint32_t Rprev = 0;
bool hayRprev = false;

// PROMEDIO PTT/PIR
float hPtt[NPROM];
float hPir[NPROM];
int nProm = 0;
int iProm = 0;
float pttProm = 0.0f;
float pirProm = 1.0f;

// CALIBRACION
bool calibrando = false;
bool calibrado = false;
float SBP0 = 120.0f;
float DBP0 = 80.0f;
float PTT0 = 0.0f;
float PIR0 = 1.0f;
float sumPtt = 0.0f;
float sumPir = 0.0f;
int nCal = 0;
int pasActual = -1;
int dbpActual = 0;
uint8_t ultimoMotivo = 0;

// CALIBRACION (referencia)
float sbpRef = SBP_REF;
float dbpRef = DBP_REF;
bool refRecibida = false;

// AUTOESCALADO
struct EscalaAutomatica {
    float minimo;
    float maximo;
    float bloqueMinimo;
    float bloqueMaximo;
    uint16_t cantidad;
    bool inicializada;
};
EscalaAutomatica escalaEcg = {0.0f, 0.0f, 1e9f, -1e9f, 0, false};
EscalaAutomatica escalaPpg = {0.0f, 0.0f, 1e9f, -1e9f, 0, false};
static const float SPAN_MIN_ECG_V = 0.0002f;
static const float SPAN_MIN_PPG_V = 0.0020f;
unsigned long tGrafica = 0;
unsigned long tInfo = 0;

// COLA DE GRAFICA
#define DECIM_ECG 4
#define DECIM_PPG 2
#define N_COLA_G  64

struct ColaGrafica {
    float v[N_COLA_G];
    uint8_t head = 0, tail = 0;
    void push(float x) {
        uint8_t h = (head + 1) % N_COLA_G;
        if (h == tail) tail = (tail + 1) % N_COLA_G;
        v[head] = x; head = h;
    }
    bool pop(float &x) {
        if (tail == head) return false;
        x = v[tail]; tail = (tail + 1) % N_COLA_G;
        return true;
    }
};
ColaGrafica colaEcg, colaPpg;
float accEcg = 0; uint8_t nAccEcg = 0;
float accPpg = 0; uint8_t nAccPpg = 0;

// ===================== ESTADO DE ALARMAS =====================
#define NIVEL_BAJO  0
#define NIVEL_MEDIO 1
#define NIVEL_ALTO  2

#define SON_PULSO   0
#define SON_TECNICA 1
#define SON_MEDIA   2
#define SON_ALTA    3

uint8_t nivelActual  = NIVEL_BAJO;
uint8_t candidato    = NIVEL_BAJO;
uint8_t segCandidato = 0;
uint8_t news2Total   = 0;
bool    spo2EnUso    = false;
uint8_t segTecnica   = 0;
bool    alarmaTecnica = false;
uint8_t visEnviado   = 255;
uint8_t segReenvio   = 0;
uint32_t segSimulacion = 0;
volatile uint8_t modoSonido = SON_PULSO;

// Simulación de NEWS2 desde la página web (0 = apagada, 1 = medio, 2 = alto)
#define DEMO_MS 120000UL               // se apaga sola a los 2 minutos
volatile uint8_t modoDemo    = 0;
uint32_t         demoHastaMs = 0;

// ===================== LÍMITES Y SILENCIO =====================
struct Limites { uint8_t fcMax, fcMin, spo2Min, pasMax, pasMin; };
Limites lim = { LIM_FCMAX_DEF, LIM_FCMIN_DEF, LIM_SPO2MIN_DEF, LIM_PASMAX_DEF, LIM_PASMIN_DEF };

uint32_t silencioHastaMs = 0;

uint8_t  paginaNex     = NEX_ID_PRINCIPAL;
int8_t   pendiente     = -1;
uint32_t tPendiente    = 0;
uint32_t tIgnorarHasta = 0;
uint8_t  idxConsulta   = 0;
int32_t  leidos[6]     = {0, 0, 0, 0, 0, 0};
uint8_t  mascara       = 0;
uint8_t  bt0Previo     = 0;
uint32_t tSendme       = 0;
uint32_t tConsulta     = 0;

static const char* OBJ_ALARMA[6] = {
    NEX_LIM_FCMAX, NEX_LIM_FCMIN, NEX_LIM_SPO2MIN, NEX_LIM_PASMAX, NEX_LIM_PASMIN, NEX_SILENCIO
};

// ===================== BUZZER (tarea FreeRTOS) =====================
#define PIN_BUZZER      25
#define DUR_BUZZER_MS   60
#define FREC_BUZZER_HZ  1000
#define RES_BUZZER      8

TaskHandle_t hBuzzer = NULL;

struct Paso { uint16_t freq; uint16_t ms; };

static const Paso PATRON_ALTA[] = {
  {ALARMA_FREQ_HZ, 150}, {0, 100}, {ALARMA_FREQ_HZ, 150}, {0, 100}, {ALARMA_FREQ_HZ, 150}, {0, 350},
  {ALARMA_FREQ_HZ, 150}, {0, 100}, {ALARMA_FREQ_HZ, 150}, {0, 2000}
};
static const Paso PATRON_MEDIA[] = {
  {ALARMA_FREQ_HZ, 200}, {0, 200}, {ALARMA_FREQ_HZ, 200}, {0, 200}, {ALARMA_FREQ_HZ, 200}, {0, 5000}
};
static const Paso PATRON_TECNICA[] = {
  {ALARMA_FREQ_HZ, 100}, {0, 150}, {ALARMA_FREQ_HZ, 100}, {0, 10000}
};
#define N_PASOS(p) (sizeof(p) / sizeof(Paso))

void reproducirPatron(const Paso* patron, size_t n, uint8_t modo) {
    for (size_t i = 0; i < n; i++) {
        ledcWriteTone(PIN_BUZZER, patron[i].freq);
        uint16_t restante = patron[i].ms;
        while (restante > 0) {
            uint16_t paso = restante > 50 ? 50 : restante;
            vTaskDelay(pdMS_TO_TICKS(paso));
            restante -= paso;
            if (modoSonido != modo) { ledcWriteTone(PIN_BUZZER, 0); return; }
        }
    }
    ledcWriteTone(PIN_BUZZER, 0);
}

void tareaBuzzer(void* arg) {
    ledcAttach(PIN_BUZZER, FREC_BUZZER_HZ, RES_BUZZER);
    ledcWriteTone(PIN_BUZZER, 0);

    for (;;) {
        uint8_t modo = modoSonido;
#if USAR_ALARMAS && USAR_SONIDO_ALARMAS
        if (modo == SON_ALTA)    { reproducirPatron(PATRON_ALTA,    N_PASOS(PATRON_ALTA),    modo); continue; }
        if (modo == SON_MEDIA)   { reproducirPatron(PATRON_MEDIA,   N_PASOS(PATRON_MEDIA),   modo); continue; }
        if (modo == SON_TECNICA) { reproducirPatron(PATRON_TECNICA, N_PASOS(PATRON_TECNICA), modo); continue; }
#endif
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50))) {
            ledcWriteTone(PIN_BUZZER, FREC_BUZZER_HZ);
            vTaskDelay(pdMS_TO_TICKS(DUR_BUZZER_MS));
            ledcWriteTone(PIN_BUZZER, 0);
        }
    }
}

// FR
void tareaFR(void* arg) {
    MuestraFR m;
    uint16_t cuenta = 0;
    for (;;) {
        if (xQueueReceive(colaFR, &m, portMAX_DELAY) == pdTRUE) {
            if (m.esPpg) estimadorFR.pushPpg((float)m.raw);
            else         estimadorFR.pushEcg((float)m.raw);
            if (++cuenta >= 250) {
                cuenta = 0;
                RespRate::Result r = estimadorFR.snapshot();
                frMostrar = r.hasValue ? (int)(r.rrBpm + 0.5f) : -1;
            }
        }
    }
}

// BIQUAD
void biquadPrime(Biquad * filtro, float x0) {
    filtro->z2 = (filtro->b2 - filtro->a2) * x0;
    filtro->z1 = (filtro->b1 - filtro->a1 + filtro->b2 - filtro->a2) * x0;
}
void biquadNotch(Biquad* f, float fs, float f0, float Q) {
    float w0 = 2.0f * PI * f0 / fs;
    float c  = cosf(w0);
    float al = sinf(w0) / (2.0f * Q);
    float a0 = 1.0f + al;
    f->b0 = 1.0f / a0;
    f->b1 = -2.0f * c / a0;
    f->b2 = 1.0f / a0;
    f->a1 = -2.0f * c / a0;
    f->a2 = (1.0f - al) / a0;
    f->z1 = f->z2 = 0.0f;
}

// NEXTION
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

void mostrarFR() {
    int fr = frMostrar;
    nexNumero("nFR", (fr > 0) ? fr : 0);
}

// ===================== LÍMITES =====================
bool limitesValidos(const Limites& L) {
    return L.fcMax   >= 60  && L.fcMax  <= 220 &&
           L.fcMin   >= 30  && L.fcMin  + 10 <= L.fcMax &&
           L.spo2Min >= 80  && L.spo2Min <= 100 &&
           L.pasMax  >= 100 && L.pasMax <= 250 &&
           L.pasMin  >= 60  && L.pasMin + 10 <= L.pasMax;
}

bool limitesIguales(const Limites& a, const Limites& b) {
    return a.fcMax == b.fcMax && a.fcMin == b.fcMin && a.spo2Min == b.spo2Min &&
           a.pasMax == b.pasMax && a.pasMin == b.pasMin;
}

void cargarLimites() {
    prefs.begin("monitor", true);
    Limites L;
    L.fcMax   = prefs.getUChar("fcMax",   LIM_FCMAX_DEF);
    L.fcMin   = prefs.getUChar("fcMin",   LIM_FCMIN_DEF);
    L.spo2Min = prefs.getUChar("spo2Min", LIM_SPO2MIN_DEF);
    L.pasMax  = prefs.getUChar("pasMax",  LIM_PASMAX_DEF);
    L.pasMin  = prefs.getUChar("pasMin",  LIM_PASMIN_DEF);
    prefs.end();
    if (limitesValidos(L)) lim = L;
}

void guardarLimites() {
    prefs.begin("monitor", false);
    prefs.putUChar("fcMax",   lim.fcMax);
    prefs.putUChar("fcMin",   lim.fcMin);
    prefs.putUChar("spo2Min", lim.spo2Min);
    prefs.putUChar("pasMax",  lim.pasMax);
    prefs.putUChar("pasMin",  lim.pasMin);
    prefs.end();
}

void escribirLimites() {
    nexNumero(NEX_LIM_FCMAX,   lim.fcMax);
    nexNumero(NEX_LIM_FCMIN,   lim.fcMin);
    nexNumero(NEX_LIM_SPO2MIN, lim.spo2Min);
    nexNumero(NEX_LIM_PASMAX,  lim.pasMax);
    nexNumero(NEX_LIM_PASMIN,  lim.pasMin);
}

// ===================== SILENCIO =====================
bool silenciado() {
    if (silencioHastaMs == 0) return false;
    if ((int32_t)(millis() - silencioHastaMs) >= 0) {
        silencioHastaMs = 0;
        return false;
    }
    return true;
}

void escribirSilencio(bool activo) {
    nexNumero(NEX_SILENCIO, activo ? 1 : 0);
    char b[32];
    snprintf(b, sizeof(b), "%s.txt=\"%s\"", NEX_SILENCIO, activo ? "ON" : "OFF");
    nexComando(b);
}

// ===================== PÁGINAS Y LECTURA DE pAlarm (desactivado) =====================
void escribirPaginaAlarmas() {
    escribirLimites();
    bool sil = silenciado();
    escribirSilencio(sil);
    bt0Previo = sil ? 1 : 0;
}

void cambiarPagina(uint8_t nueva) {
    if (nueva == paginaNex) return;
    uint8_t vieja = paginaNex;
    paginaNex = nueva;
    if (nueva == NEX_ID_PRINCIPAL) visEnviado = 255;
    if (nueva == NEX_ID_ALARMAS) {
        if (vieja == NEX_ID_PRINCIPAL || vieja == NEX_ID_INFO) escribirPaginaAlarmas();
        tIgnorarHasta = millis() + 400;
        mascara = 0;
        idxConsulta = 0;
        pendiente = -1;
    }
}

void consultar(uint8_t indice) {
    char b[24];
    snprintf(b, sizeof(b), "get %s.val", OBJ_ALARMA[indice]);
    nexComando(b);
    pendiente  = (int8_t)indice;
    tPendiente = millis();
}

void aplicarLecturas() {
#if USAR_LIMITES
    Limites L;
    L.fcMax   = (uint8_t)constrain(leidos[0], 0, 255);
    L.fcMin   = (uint8_t)constrain(leidos[1], 0, 255);
    L.spo2Min = (uint8_t)constrain(leidos[2], 0, 255);
    L.pasMax  = (uint8_t)constrain(leidos[3], 0, 255);
    L.pasMin  = (uint8_t)constrain(leidos[4], 0, 255);
    if (!limitesIguales(L, lim)) {
        if (limitesValidos(L)) { lim = L; guardarLimites(); }
        else                   { escribirLimites(); }
    }
#endif

#if USAR_SILENCIO
    uint8_t bt = (leidos[5] != 0) ? 1 : 0;
    if (bt == 1 && bt0Previo == 0) {
        uint32_t fin = millis() + SILENCIO_MS;
        silencioHastaMs = (fin == 0) ? 1 : fin;
        escribirSilencio(true);
    } else if (bt == 0 && bt0Previo == 1) {
        silencioHastaMs = 0;
        escribirSilencio(false);
    } else if (bt == 1 && !silenciado()) {
        escribirSilencio(false);
        bt = 0;
    }
    bt0Previo = bt;
#endif
}

void procesarTrama(const uint8_t* t, uint8_t n) {
    if (n < 1) return;
    if (t[0] == 0x66 && n >= 2) {
        cambiarPagina(t[1]);
        return;
    }
    if (t[0] == 0x71 && n >= 5 && pendiente >= 0) {
        int32_t v = (int32_t)((uint32_t)t[1] | ((uint32_t)t[2] << 8) |
                              ((uint32_t)t[3] << 16) | ((uint32_t)t[4] << 24));
        leidos[pendiente] = v;
        mascara |= (uint8_t)(1 << pendiente);
        pendiente = -1;
        if (mascara == 0x3F) {
            if (paginaNex == NEX_ID_ALARMAS) aplicarLecturas();
            mascara = 0;
        }
    }
}

// ===================== NEWS2 =====================
int puntajeFR(int fr) {
    if (fr < 0)   return -1;
    if (fr <= 8)  return 3;
    if (fr <= 11) return 1;
    if (fr <= 20) return 0;
    if (fr <= 24) return 2;
    return 3;
}
int puntajeSpO2(int spo2) {
    if (spo2 <= 0)  return -1;
    if (spo2 <= 91) return 3;
    if (spo2 <= 93) return 2;
    if (spo2 <= 95) return 1;
    return 0;
}
int puntajePAS(int pas) {
    if (pas <= 0)   return -1;
    if (pas <= 90)  return 3;
    if (pas <= 100) return 2;
    if (pas <= 110) return 1;
    if (pas <= 219) return 0;
    return 3;
}
int puntajePulso(int p) {
    if (p <= 0)   return -1;
    if (p <= 40)  return 3;
    if (p <= 50)  return 1;
    if (p <= 90)  return 0;
    if (p <= 110) return 1;
    if (p <= 130) return 2;
    return 3;
}

void evaluarAlarmas() {
    int fr = -1, spo2 = -1, pas = -1, pulso = -1;
    bool fueraLimites = false;

    // ¿Simulación activa desde la página?
    bool demo = false;
    if (modoDemo != 0) {
        if ((int32_t)(millis() - demoHastaMs) >= 0) modoDemo = 0;   // pasaron los 2 minutos
        else demo = true;
    }

#if SIMULAR_NEWS2
    uint8_t fase = (segSimulacion++ / 20) % 4;
    fr = 16; spo2 = 98; pas = 120; pulso = 75;
    if (fase == 1) { pulso = 135; }
    if (fase == 2) { pulso = 115; spo2 = 93; fr = 22; }
    if (fase == 3) { pulso = 135; spo2 = 90; fr = 26; }
#else
    if (demo) {
        fr = 16; spo2 = 98; pas = 120; pulso = 75;
        if (modoDemo == 1) { pulso = 135; }                       // un parámetro con 3 -> MEDIO
        if (modoDemo == 2) { pulso = 135; spo2 = 90; fr = 26; }   // total 9 -> ALTO
    } else {
  #if USAR_FR_EN_NEWS2
        fr = (frMostrar > 0) ? frMostrar : -1;
  #endif
        spo2 = (spo2Actual > 0) ? spo2Actual : -1;
  #if USAR_PAS_EN_NEWS2
        pas = pasActual;
  #endif
        pulso = (fcEstable > 0) ? fcEstable : ((prEstable > 0) ? prEstable : -1);

  #if USAR_LIMITES
        if (pulso > 0 && (pulso > lim.fcMax || pulso < lim.fcMin))               fueraLimites = true;
        if (spo2  > 0 &&  spo2  < lim.spo2Min)                                   fueraLimites = true;
        if (pasActual > 0 && (pasActual > lim.pasMax || pasActual < lim.pasMin)) fueraLimites = true;
  #endif
    }
#endif

    int p[4] = { puntajeFR(fr), puntajeSpO2(spo2), puntajePAS(pas), puntajePulso(pulso) };
    int total = 0;
    bool algunTres = false;
    for (int i = 0; i < 4; i++) {
        if (p[i] < 0) continue;
        total += p[i];
        if (p[i] == 3) algunTres = true;
    }
    news2Total = (uint8_t)total;

    uint8_t nuevo = NIVEL_BAJO;
    if (total >= 7)                   nuevo = NIVEL_ALTO;
    else if (total >= 5 || algunTres) nuevo = NIVEL_MEDIO;
    if (fueraLimites && nuevo < NIVEL_MEDIO) nuevo = NIVEL_MEDIO;

    // Persistencia
    if (nuevo == nivelActual) {
        segCandidato = 0;
    } else {
        if (nuevo != candidato) { candidato = nuevo; segCandidato = 0; }
        segCandidato++;
        uint8_t necesario = (candidato > nivelActual) ? ESCALAR_SEG : DESESCALAR_SEG;
        if (segCandidato >= necesario) { nivelActual = candidato; segCandidato = 0; }
    }

    // Alarma técnica
#if SIMULAR_NEWS2
    alarmaTecnica = false;
#else
    if (demo) {
        alarmaTecnica = false;
        segTecnica = 0;
    } else {
        if (spo2Actual > 0) spo2EnUso = true;
        bool cond = spo2EnUso && (spo2Actual == 0);
        if (cond) { if (segTecnica < 255) segTecnica++; } else segTecnica = 0;
        alarmaTecnica = (segTecnica >= TECNICA_SEG);
    }
#endif

    // Visual
    uint8_t vis = (nivelActual == NIVEL_ALTO)  ? 2 :
                  (nivelActual == NIVEL_MEDIO) ? 1 :
                  alarmaTecnica                ? 3 : 0;
    if (++segReenvio >= 5) { segReenvio = 0; visEnviado = 255; }
    if (vis != visEnviado) {
        nexNumero("vNivel", vis);
        nexNumero("page0.vNivel", vis);
        visEnviado = vis;
    }

    // Sonido (silenciado: sin patrones de alarma, el bip de pulso sigue)
    uint8_t modo = (nivelActual == NIVEL_ALTO)  ? SON_ALTA :
                   (nivelActual == NIVEL_MEDIO) ? SON_MEDIA :
                   alarmaTecnica                ? SON_TECNICA : SON_PULSO;
#if USAR_SILENCIO
    if (modo != SON_PULSO && silenciado()) modo = SON_PULSO;
#endif
    modoSonido = modo;
}

// PRESION ARTERIAL
void calcularPA(float ptt, float pir) {
    float PP0 = SBP0 - DBP0;
    float MBP0 = DBP0 + PP0 / 3.0f;
    float r = PTT0 / ptt;
    float mbp = MBP0 * (PIR0 / pir);
    float pp = PP0 * (pir / PIR0) * r * r;
    float sbp = mbp + 2.0f * pp / 3.0f;
    float dbp = mbp - pp / 3.0f;

    sbp = constrain(sbp, 40.0f, 250.0f);
    dbp = constrain(dbp, 20.0f, 200.0f);
    pasActual = (int)(sbp + 0.5f);
    dbpActual = (int)(dbp + 0.5f);
    nexNumero("nSBP", (int)(sbp + 0.5f));
    nexNumero("nDBP", (int)(dbp + 0.5f));
}

// CALIBRACION
void iniciarCalibracion(float sbp, float dbp) {
    SBP0 = sbp;
    DBP0 = dbp;
    sumPtt = 0.0f;
    sumPir = 0.0f;
    nCal = 0;
    nProm = 0;
    iProm = 0;
    pttProm = 0.0f;
    pirProm = 1.0f;
    calibrado = false;
    calibrando = true;
    pasActual = -1;
    nexNumero("nSBP", 0);
    nexNumero("nDBP", 0);
}

void acumularCalibracion(float ptt, float pir) {
    sumPtt += ptt;
    sumPir += pir;
    nCal++;
    if (nCal >= N_CAL) {
        PTT0 = sumPtt / nCal;
        PIR0 = sumPir / nCal;
        calibrando = false;
        calibrado = true;
    }
}

void recibirReferencia(int sbp, int dbp) {
    if (sbp < 60 || sbp > 250 || dbp < 30 || dbp >= sbp)
        return;
    sbpRef = sbp;
    dbpRef = dbp;
    refRecibida = true;
    calibrado = false;
    calibrando = false;
    nCal = 0;
    pasActual = -1;
    nexNumero("nSBP", 0);
    nexNumero("nDBP", 0);
}

// ===================== RECEPCIÓN DE LA NEXTION =====================
void procesarLinea(const char* linea) {
    if (strncmp(linea, "CAL", 3) == 0) {
        int s, d;
        if (sscanf(linea + 3, "%d%*[ -]%d", &s, &d) == 2) recibirReferencia(s, d);
        return;
    }
    int pag;
    if (sscanf(linea, "PAG %d", &pag) == 1) {
        if (pag == 0) cambiarPagina(NEX_ID_PRINCIPAL);
        if (pag == 1) cambiarPagina(NEX_ID_INFO);
        if (pag == 2) cambiarPagina(NEX_ID_ALARMAS);
    }
}

void leerNextion() {
    static uint8_t estado = 0;
    static char    linea[24];
    static uint8_t nLinea = 0;
    static uint8_t trama[32];
    static uint8_t nTrama = 0;
    static uint8_t nFF = 0;

    while (Serial.available()) {
        uint8_t c = (uint8_t)Serial.read();

        if (estado == 1) {
            if (c == '\n' || c == '\r') {
                linea[nLinea] = '\0';
                procesarLinea(linea);
                estado = 0;
                continue;
            }
            if (c >= 32 && c <= 126) {
                if (nLinea < sizeof(linea) - 1) linea[nLinea++] = (char)c;
                continue;
            }
            estado = 0;
        }

        if (estado == 0) {
            if (c >= 'A' && c <= 'Z') { nLinea = 0; linea[nLinea++] = (char)c; estado = 1; continue; }
            if (c == '\n' || c == '\r') continue;
            nTrama = 0; nFF = 0; estado = 2;
        }

        if (nTrama < sizeof(trama)) trama[nTrama++] = c;
        nFF = (c == 0xFF) ? nFF + 1 : 0;
        if (nFF >= 3) {
            procesarTrama(trama, nTrama - 3);
            estado = 0;
        } else if (nTrama >= sizeof(trama)) {
            estado = 0;
        }
    }
}

// PTT/PIR
void revisarYPromediar(float ptt, float pir) {
    if (ptt < PTT_MIN_MS || ptt > PTT_MAX_MS) {
        ultimoMotivo = 1;
        return;
    }
    if (USAR_PIR && (pir <= PIR_MIN || pir > PIR_MAX)) {
        ultimoMotivo = 2;
        return;
    }
    if (nProm == NPROM && fabsf(ptt - pttProm) > 0.2f * pttProm) {
        ultimoMotivo = 3;
        return;
    }
    ultimoMotivo = 0;
    hPtt[iProm] = ptt;
    hPir[iProm] = pir;
    iProm = (iProm + 1) % NPROM;
    if (nProm < NPROM)
        nProm++;
    float sP = 0.0f;
    float sI = 0.0f;
    for (int j = 0; j < nProm; j++) {
        sP += hPtt[j];
        sI += hPir[j];
    }
    pttProm = sP / nProm;
    pirProm = sI / nProm;

    if (calibrando) {
        acumularCalibracion(ptt, pir);
    } else if (calibrado) {
        calcularPA(pttProm, pirProm);
    }
}

// PROCESAR LATIDO
void procesarLatido(uint32_t R, uint32_t Rsig) {
    if (!hayPpg || spo2Estado == 0)
        return;
    float rr = (Rsig - R) * 1000.0f / FS_ECG_HZ;
    if (rr < RR_MIN_MS || rr > RR_MAX_MS)
        return;
    uint32_t fin = R + MS_A_MUESTRAS(500);
    if (fin > Rsig)
        fin = Rsig;
    uint32_t k = R + MS_A_MUESTRAS(100);
    float mejor = -1e9f;
    for (uint32_t i = R + MS_A_MUESTRAS(100); i < fin; i++) {
        float pend = BUF(ppgBuf, i + 5) - BUF(ppgBuf, i - 5);
        if (pend > mejor) {
            mejor = pend;
            k = i;
        }
    }
    if (mejor <= 0.0f) {
        ultimoMotivo = 4;
        return;
    }
    float ptt = (k - R) * 1000.0f / FS_ECG_HZ;
    float pir = 1.0f;
#if USAR_PIR
  #if USAR_PIR_SPO2
    if (dcIr == 0 || acIr == 0 || acIr >= dcIr) {
        ultimoMotivo = 2;
        return;
    }
    pir = ((float)dcIr + 0.5f * (float)acIr) / ((float)dcIr - 0.5f * (float)acIr);
  #else
    uint32_t finL = k + MS_A_MUESTRAS(300);
    if (finL > Rsig)
        finL = Rsig;
    float maxDesp = -1e9f;
    float minAntes = 1e9f;
    for (uint32_t i = R; i <= k; i++) {
        float v = BUF(dcBuf, i);
        if (v < minAntes)
            minAntes = v;
    }
    for (uint32_t i = k; i <= finL; i++) {
        float v = BUF(dcBuf, i);
        if (v > maxDesp)
            maxDesp = v;
    }
    float IH = maxDesp;
    float IL = minAntes;
    if (IL <= 1e-6f) {
        ultimoMotivo = 2;
        return;
    }
    pir = IH / IL;
  #endif
#endif
    revisarYPromediar(ptt, pir);
}

// UBICAR R
void nuevoLatido(uint32_t m) {
    if (muestraDeteccion < MS_A_MUESTRAS(500))
        return;
    uint32_t ini = muestraDeteccion - MS_A_MUESTRAS(R_ANTES_MS);
    if (hayRprev && ini < Rprev + MS_A_MUESTRAS(250))
        ini = Rprev + MS_A_MUESTRAS(250);
    if (ini > m)
        ini = m;
    uint32_t R = ini;
    for (uint32_t i = ini; i <= m; i++) {
        if (BUF(ecgBuf, i) > BUF(ecgBuf, R))
            R = i;
    }
    if (hayRprev)
        procesarLatido(Rprev, R);
    Rprev = R;
    hayRprev = true;
}

// DETECTOR ECG
void reiniciarDetectorEcg() {
    ecgBase = 0.0f;
    envPico = 0.0f;
    sobreUmbral = false;
    hayLatidoPrevio = false;
    ultimoLatidoMuestra = 0;
    rrCount = 0;
    rrIdx = 0;
    bpmPropio = 0.0f;
}

void detectorEcgMuestra(float x) {
    muestrasEcg++;
    ecgBase += kBase * (x - ecgBase);
    float a = x - ecgBase;
    if (a < 0.0f)
        a = -a;
    envPico *= decayEnv;
    if (a > envPico)
        envPico = a;
    float umbral = UMBRAL_FRACCION * envPico;
    if (envPico < AMP_MIN_ECG_V) {
        sobreUmbral = false;
        return;
    }
    if (a > umbral) {
        if (!sobreUmbral) {
            sobreUmbral = true;
            if (!hayLatidoPrevio || (muestrasEcg - ultimoLatidoMuestra) >= REFRACTARIO_ECG_MUESTRAS) {
                if (hayLatidoPrevio) {
                    float rr = (float)(muestrasEcg - ultimoLatidoMuestra) / FS_ECG_HZ;
                    float bpm = 60.0f / rr;
                    if (bpm >= BPM_MIN && bpm <= BPM_MAX) {
                        rrBuf[rrIdx] = rr;
                        rrIdx = (rrIdx + 1) % N_RR;
                        if (rrCount < N_RR)
                            rrCount++;
                        float suma = 0.0f;
                        for (uint8_t k = 0; k < rrCount; k++)
                            suma += rrBuf[k];
                        bpmPropio = 60.0f / (suma / rrCount);
                        ultimoLatidoValidoMs = millis();
                    }
                }
                hayLatidoPrevio = true;
                ultimoLatidoMuestra = muestrasEcg;
                latidoPendiente = true;
                muestraDeteccion = muestrasEcg - 1;
                if (hBuzzer) xTaskNotifyGive(hBuzzer);
            }
        }
    }
    else {
        sobreUmbral = false;
    }
}

// PAN-TOMPKINS
void iniciarPTEcg() {
    pt_init(&ptEcg, FS_ECG_HZ);
    tiempoEcgInicializado = false;
}
void iniciarPTPpg() {
    pt_init(&ptPpg, FS_PPG_HZ);
    biquad_highpass(&ptPpg.hp, FS_PPG_HZ, PT_HP_FC_PPG, PT_Q_PPG);
    biquad_lowpass(&ptPpg.lp, FS_PPG_HZ, PT_LP_FC_PPG, PT_Q_PPG);
    tiempoPpgInicializado = false;
    prEstable = 0;
}

// ESCALADO
uint8_t escalarSenal(EscalaAutomatica& escala, float valor, float spanMinimo, bool invertir) {
    if (valor < escala.bloqueMinimo)
        escala.bloqueMinimo = valor;
    if (valor > escala.bloqueMaximo)
        escala.bloqueMaximo = valor;
    escala.cantidad++;
    if (escala.cantidad >= 200) {
        if (!escala.inicializada) {
            escala.minimo = escala.bloqueMinimo;
            escala.maximo = escala.bloqueMaximo;
            escala.inicializada = true;
        } else {
            escala.minimo = 0.5f * escala.minimo + 0.5f * escala.bloqueMinimo;
            escala.maximo = 0.5f * escala.maximo + 0.5f * escala.bloqueMaximo;
        }
        escala.bloqueMinimo = 1e9f;
        escala.bloqueMaximo = -1e9f;
        escala.cantidad = 0;
    }
    float minimo = escala.inicializada ? escala.minimo : escala.bloqueMinimo;
    float maximo = escala.inicializada ? escala.maximo : escala.bloqueMaximo;
    float span = maximo - minimo;
    if (span < spanMinimo) {
        float centro = 0.5f * (maximo + minimo);
        minimo = centro - 0.5f * spanMinimo;
        maximo = centro + 0.5f * spanMinimo;
        span = spanMinimo;
    }
    minimo -= 0.1f * span;
    maximo += 0.1f * span;
    int salida = (int)(((valor - minimo) / (maximo - minimo)) * ALTO_WAVE + 0.5f);
    salida = constrain(salida, 0, ALTO_WAVE);
    if (invertir)
        salida = ALTO_WAVE - salida;
    return (uint8_t)salida;
}

// NUEVA MUESTRA ECG
void nuevaMuestraEcg(float voltios) {
    ecgVoltios = voltios;
    if (!hayEcg) {
        biquadPrime(&ecgNotch, ecgVoltios);
        biquadPrime(&ecgLP1, ecgVoltios);
        biquadPrime(&ecgLP2, ecgVoltios);
        biquadPrime(&ecgVis1, ecgVoltios);
        biquadPrime(&ecgVis2, ecgVoltios);
        ecgFiltrada = ecgVoltios;
        ecgBase = ecgVoltios;
        hayEcg = true;
    }
    ecgFiltrada = biquad_process(&ecgLP2, biquad_process(&ecgLP1, biquad_process(&ecgNotch, ecgVoltios)));

    float ecgVis = biquad_process(&ecgVis2, biquad_process(&ecgVis1, ecgFiltrada));
    accEcg += ecgVis;
    if (++nAccEcg >= DECIM_ECG) {
        colaEcg.push(accEcg / DECIM_ECG);
        bleAgregarPunto('E', accEcg / DECIM_ECG);        // Bluetooth
        accEcg = 0; nAccEcg = 0;
    }
    contadorMuestrasEcgSeg++;

    {
        uint32_t idx = muestrasEcg;
        BUF(ecgBuf, idx) = INVERTIR_ECG ? -ecgFiltrada : ecgFiltrada;
        if (hayPpg) {
            BUF(ppgBuf, idx) = ppgPulsatil;
            BUF(dcBuf, idx) = fabsf(ppgFiltrada);
        } else {
            BUF(ppgBuf, idx) = 0.0f;
            BUF(dcBuf, idx) = 0.0f;
        }
    }

    detectorEcgMuestra(ecgFiltrada);

    if (latidoPendiente && muestrasEcg > muestraDeteccion + MS_A_MUESTRAS(R_DESPUES_MS)) {
        latidoPendiente = false;
        nuevoLatido(muestrasEcg - 1);
    }

    if (!tiempoEcgInicializado) {
        tiempoEcgUs = micros();
        tiempoEcgInicializado = true;
    } else {
        tiempoEcgUs += PERIODO_ECG_US;
    }
    float entradaPT = ecgVoltios * PT_CUENTAS_POR_V_ECG;
    pt_process_t(&ptEcg, entradaPT, tiempoEcgUs);
}

// NUEVA MUESTRA PPG
void nuevaMuestraPpg(float voltios) {
    ppgVoltios = voltios;

    if (!hayPpg) {
        biquadPrime(&ppgLP1, ppgVoltios);
        biquadPrime(&ppgLP2, ppgVoltios);
        ppgDC = ppgVoltios;
        ppgFiltrada = ppgVoltios;
        ppgHP.z1 = 0.0f;
        ppgHP.z2 = 0.0f;
        ppgPulsatil = 0.0f;
        hayPpg = true;
    }

    ppgFiltrada = biquad_process(&ppgLP2, biquad_process(&ppgLP1, ppgVoltios));
    ppgPulsatil = biquad_process(&ppgHP, ppgFiltrada);

#if INVERTIR_PULSO_PPG
    ppgPulsatil = -ppgPulsatil;
#endif

    accPpg += ppgPulsatil;
    if (++nAccPpg >= DECIM_PPG) {
        colaPpg.push(accPpg / DECIM_PPG);
        bleAgregarPunto('P', accPpg / DECIM_PPG);        // Bluetooth
        accPpg = 0; nAccPpg = 0;
    }

    ppgDC += kDC * (ppgFiltrada - ppgDC);

    if (!tiempoPpgInicializado) {
        tiempoPpgUs = micros();
        tiempoPpgInicializado = true;
    } else {
        tiempoPpgUs += PERIODO_PPG_US;
    }

    float entradaPT = ppgPulsatil * PT_CUENTAS_POR_V_PPG;
    pt_process_t(&ptPpg, entradaPT, tiempoPpgUs);
}

// DECODIFICAR PAQUETE
void procesarPaquete(const uint8_t* datos, size_t longitud) {
    for (size_t i = 0; i + 4 <= longitud; i += 4) {
        uint32_t valor24 = (uint32_t)datos[i] | ((uint32_t)datos[i + 1] << 8) | ((uint32_t)datos[i + 2] << 16);
        uint8_t tag = datos[i + 3];

        switch (tag) {
            case 0x02:
                cicloIdx = 0;
                break;
            case 0x05: {
                int32_t raw = (int32_t)valor24;
                if (raw & 0x00800000L) {
                    raw -= 0x01000000L;
                }
                float voltios = raw * FACTOR_VOLT;
                if (cicloIdx == 0 || cicloIdx == 1) {
                    nuevaMuestraEcg(voltios);
                    if (colaFR) { MuestraFR mfr = { raw, 0 }; xQueueSend(colaFR, &mfr, 0); }
                } else {
                    nuevaMuestraPpg(voltios);
                    if (colaFR) { MuestraFR mfr = { raw, 1 }; xQueueSend(colaFR, &mfr, 0); }
                }
                cicloIdx = (cicloIdx + 1) % 3;
                break;
            }
            case 0x06: {
                uint8_t nuevoEstado = datos[i];
                if (spo2Estado == 0 && nuevoEstado != 0) {
                    iniciarPTPpg();
                }
                spo2Estado = nuevoEstado;
                spo2Seg = datos[i + 1];
                break;
            }
            case 0x07: acIr  = valor24; break;
            case 0x08: dcIr  = valor24; break;
            case 0x09: acRed = valor24; break;
            case 0x0A:
                dcRed = valor24;
                spo2Nuevo = true;
                break;
        }
    }
}

// SpO2
int calcularSpO2() {
    if (spo2Estado != 2 || dcRed == 0 || dcIr == 0 || acIr == 0) {
        return 0;
    }
    if (dcIr < SPO2_DC_MIN) {
        return 0;
    }
    float relacionRoja = (float)acRed / (float)dcRed;
    float relacionIr = (float)acIr / (float)dcIr;
    float R = relacionRoja / relacionIr;
    float spo2 = 110.0f - 25.0f * R;
    if (spo2 < 0.0f)   spo2 = 0.0f;
    if (spo2 > 100.0f) spo2 = 100.0f;
    return (int)(spo2 + 0.5f);
}

// SETUP
void setup() {
    Serial.begin(NEX_BAUD);
    delay(500);
    nexComando("");
    nexComando("bkcmd=0");

    // ECG
    biquadNotch(&ecgNotch, FS_ECG_HZ, 60.0f, 8.0f);
    biquad_lowpass(&ecgLP1, FS_ECG_HZ, ECG_FC_HZ, ECG_Q);
    biquad_lowpass(&ecgLP2, FS_ECG_HZ, ECG_FC_HZ, ECG_Q);
    biquad_lowpass(&ecgVis1, FS_ECG_HZ, 25.0f, 0.7071f);
    biquad_lowpass(&ecgVis2, FS_ECG_HZ, 25.0f, 0.7071f);

    // PPG
    biquad_lowpass(&ppgLP1, FS_PPG_HZ, PPG_FC_HZ, PPG_Q);
    biquad_lowpass(&ppgLP2, FS_PPG_HZ, PPG_FC_HZ, PPG_Q);
    biquad_highpass(&ppgHP, FS_PPG_HZ, PPG_HP_FC_HZ, PPG_HP_Q);

    kDC = 2.0f * PI * PT_DC_FC_HZ / FS_PPG_HZ;

    // DETECTOR ECG
    kBase = 2.0f * PI * ECG_BASE_FC_HZ / FS_ECG_HZ;
    decayEnv = expf(-1.0f / (ENV_TAU_S * FS_ECG_HZ));
    reiniciarDetectorEcg();

    // PAN-TOMPKINS
    iniciarPTEcg();
    iniciarPTPpg();

    // LÍMITES guardados
    cargarLimites();

    // FR
    estimadorFR.begin(FS_ECG_HZ, 2);
    colaFR = xQueueCreate(1024, sizeof(MuestraFR));
    xTaskCreatePinnedToCore(tareaFR, "fr", 6144, NULL, 1, NULL, 0);

    // BLUETOOTH (antes del AFE: se anuncia aunque el AFE tarde en configurarse)
    bleIniciar();

    // AFE4950
    while (!afe.configure(SCLK, MISO, MOSI, CS, AFE_RST, FIFO_RDY)) {
        delay(500);
    }
    afe.enableSpO2(true);

    // NEXTION
    nexComando("page 0");
    delay(50);
    nexComando("cle 1,0");
    nexComando("cle 2,0");
    delay(50);
    nexNumero("nHR", 0);
    nexNumero("nPR", 0);
    nexNumero("nSpO2", 0);
    nexNumero("nSBP", 0);
    nexNumero("nDBP", 0);
    nexNumero("nFR", 0);
    nexNumero("nTemp", 0);
    nexNumero("vNivel", 0);

    // BUZZER
    xTaskCreatePinnedToCore(tareaBuzzer, "buzzer", 2048, NULL, 1, &hBuzzer, 0);

#if CAL_AL_INICIO
    refRecibida = true;
#endif

    afe.enableCapture();
    tGrafica = millis();
    tInfo = millis();
    ultimoLatidoValidoMs = millis();
}

// LOOP
void loop() {
    leerNextion();
    bleProcesarComandos();          // órdenes de la página web (límites, silencio, calibración, simulación)

    // 1. AFE4950
    if (afe.ready()) {
        procesarPaquete(afe.getData(), afe.getBytesToSend());
    }

    unsigned long ahora = millis();

    // 2. GRAFICAS NEXTION
    {
        float v;
        uint8_t n = 0;
        while (n < 8 && colaEcg.pop(v)) {
            nexAdd(ID_WAVE_ECG, escalarSenal(escalaEcg, v, SPAN_MIN_ECG_V, INVERTIR_ECG));
            n++;
        }
        n = 0;
        while (n < 8 && colaPpg.pop(v)) {
            nexAdd(ID_WAVE_PPG, escalarSenal(escalaPpg, v, SPAN_MIN_PPG_V, INVERTIR_PPG));
            n++;
        }
    }

    // 3. SpO2
    if (spo2Nuevo) {
        spo2Nuevo = false;
        spo2Actual = calcularSpO2();
        nexNumero("nSpO2", spo2Actual);
    }

    // 4. Lectura de la página de alarmas de la Nextion (desactivada)
#if LEER_PAG_ALARMAS
    if (ahora - tSendme >= 250) {
        tSendme = ahora;
        nexComando("sendme");
    }
    if (paginaNex == NEX_ID_ALARMAS &&
        (int32_t)(ahora - tIgnorarHasta) >= 0 &&
        ahora - tConsulta >= 100) {
        tConsulta = ahora;
        bool esperando = (pendiente >= 0) && (ahora - tPendiente) < 100;
        if (!esperando) {
            consultar(idxConsulta);
            idxConsulta = (idxConsulta + 1) % 6;
        }
    }
#endif

    // 5. HR / PR / FR / ALARMAS (1 vez por segundo)
    if (ahora - tInfo >= PERIODO_INFO_MS) {
        tInfo = ahora;

#if HR_FUENTE == HR_FUENTE_PROPIO
        if ((ahora - ultimoLatidoValidoMs) > ((unsigned long)TIMEOUT_LATIDO_S * 1000UL)) {
            bpmPropio = 0.0f;
            hayLatidoPrevio = false;
            rrCount = 0;
            rrIdx = 0;
            hayRprev = false;
            latidoPendiente = false;
            nProm = 0;
            iProm = 0;
        }
        fcEstable = (int)(bpmPropio + 0.5f);
#else
        {
            static uint8_t malLecturasEcg = 0;
            int bpmEcg = (int)(pt_bpm(&ptEcg) + 0.5f);
            if (bpmEcg >= BPM_MIN && bpmEcg <= BPM_MAX) {
                fcEstable = bpmEcg;
                malLecturasEcg = 0;
            } else if (++malLecturasEcg >= TIMEOUT_LATIDO_S) {
                fcEstable = 0;
                malLecturasEcg = TIMEOUT_LATIDO_S;
            }
        }
#endif
        nexNumero("nHR", fcEstable);

        {
            static uint8_t malLecturasPpg = 0;
            int bpmPpg = (int)(pt_bpm(&ptPpg) + 0.5f);
            if (spo2Estado == 0) {
                prEstable = 0;
                malLecturasPpg = 0;
            } else if (bpmPpg >= BPM_MIN && bpmPpg <= BPM_MAX) {
                prEstable = bpmPpg;
                malLecturasPpg = 0;
            } else if (++malLecturasPpg >= TIMEOUT_LATIDO_S) {
                prEstable = 0;
                malLecturasPpg = TIMEOUT_LATIDO_S;
            }
        }
        nexNumero("nPR", prEstable);

#if DEBUG_PA
        int estadoPA = !refRecibida ? 0 : calibrado ? 99 : calibrando ? (10 + nCal) : nProm;
        nexNumero("nTemp", estadoPA);
        nexNumero("nFR", ultimoMotivo);
#else
        mostrarFR();
        nexNumero("nTemp", 0);
#endif

#if CAL_AUTO
        if (refRecibida && !calibrado && !calibrando && spo2Estado == 2 && nProm == NPROM) {
            iniciarCalibracion(sbpRef, dbpRef);
        }
#endif
        if (spo2Estado == 0 && nProm > 0) {
            nProm = 0;
            iProm = 0;
            pasActual = -1;
            nexNumero("nSBP", 0);
            nexNumero("nDBP", 0);
        }

#if USAR_ALARMAS
        evaluarAlarmas();
#endif

#if DEBUG_NDBG
        nexNumero("nTemp", contadorMuestrasEcgSeg);
#endif
        contadorMuestrasEcgSeg = 0;
    }
}