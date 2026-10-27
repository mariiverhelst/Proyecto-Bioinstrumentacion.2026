#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================================
// CONFIGURACIÓN GENERAL DEL MONITOR
// Todos los parámetros ajustables están aquí.
// ============================================================

// ---------------- PINES AFE4950 ----------------
#define SCLK      18
#define MISO      19
#define MOSI      23
#define CS         5
#define AFE_RST    4
#define FIFO_RDY  16

// ---------------- BUZZER PASIVO ----------------
#define PIN_BUZZER          25
#define BUZZER_DURACION_MS  40
#define BUZZER_FREQ_HZ      2000   // bip de pulso (no es alarma)

// ---------------- ALARMAS ----------------
#define ALARMA_FREQ_HZ      540    // fundamental 150-1000 Hz; armónicos 1620, 2700, 3780
#define ESCALAR_SEG         5      // segundos sostenidos para SUBIR de nivel
#define DESESCALAR_SEG      10     // segundos sostenidos para BAJAR de nivel
#define TECNICA_SEG         3      // segundos sostenidos para alarma técnica

#define SIMULAR_NEWS2       0      // 1 = valores simulados que recorren todos los niveles
#define USAR_PAS_EN_NEWS2   1      // 1 = incluir la PAS (PTT) en el NEWS2
#define USAR_FR_EN_NEWS2    0      // 1 cuando la FR estimada esté validada
#define USAR_TEMP_EN_NEWS2  0      // 1 cuando el sensor de temperatura esté listo

// ---------------- MODO DE CALIBRACIÓN DE SpO2 ----------------
// nHR = R x100 | nPR = PI x100 (%) | nSpO2 = DC IR (miles de cuentas)
// nSBP = estado de la librería | nDBP = estado efectivo  (0 sin dedo, 1 estab., 2 válida)
#define CALIBRAR_SPO2       0

// ---------------- FC / PR ----------------
#define TIMEOUT_LATIDO_MS   4000   // sin latidos en este tiempo -> FC/PR = 0
#define USAR_TIMEOUT_LATIDO 1      // 1 = FC/PR caen a 0 sin latidos
#define ECG_REINICIO_MS     10000  // solo se reinicia el Pan-Tompkins si el ECG falta > 10 s

// ---------------- VALIDACIÓN SpO2 (rechaza luz ambiente / pinza vacía) ----------------
#define SPO2_VALIDAR        1
#define SPO2_R_MIN          0.3f
#define SPO2_R_MAX          1.6f
#define SPO2_PI_MIN         0.2f
#define SPO2_PI_MAX         20.0f
#define SPO2_VALIDAR_DC     1
#define SPO2_DC_MIN         27000UL    // dedo: ~42 000 | pinza cerrada vacía: ~17 000
#define SPO2_DC_MAX         8388607UL
#define SPO2_SEG_INVALIDO   3          // s seguidos fuera de rango para declarar "sin sensor"
#define SPO2_SEG_VALIDO     2          // s seguidos en rango para declarar "válida"
#define PPG_REINICIO_MS     5000       // ausencia mínima del sensor para reiniciar el PR

// ---------------- FRECUENCIA RESPIRATORIA ESTIMADA ----------------
#define NEX_OBJ_FR           "nFR"
#define NEX_OBJ_FR_TXT       "tFRest"
#define USAR_TEXTO_ESTADO_FR 0
#define FR_CAL_MS            5000UL
#define DEBUG_FR_SERIAL1     0
#define DEBUG_TX_PIN         33

#define FR_EST_CAL_RELOJ     10        // códigos extra de estado (además de RespRate::Status)
#define FR_EST_CONFIG        11

// ---------------- PRESIÓN ARTERIAL (PTT + PIR) ----------------
#define FS_PA        500      // Hz (sincronizado con el ECG a 500 Hz)
#define USAR_PIR     1        // 1 = usa PTT + PIR, 0 = usa solo PTT
#define SBP_DEF      120.0f   // Sistólica base de calibración (mmHg)
#define DBP_DEF      80.0f    // Diastólica base de calibración (mmHg)

#define N_BUF        2048     // ~4 s de historia a 500 Hz (potencia de 2)
#define MS(x)        ((uint32_t)((x) * FS_PA / 1000))
#define B(buf, i)    buf[(i) % N_BUF]

#define NPROM        8        // latidos a promediar
#define N_CAL        5        // latidos de calibración

// ---------------- NEXTION ----------------
#define NEX_BAUD    115200
#define ID_WAVE_ECG 1
#define ID_WAVE_PPG 2
#define CANAL       0
#define ALTO_WAVE   199

#define INVERTIR_ECG true
#define INVERTIR_PPG false

// ---------------- AUTOESCALADO DE LAS GRÁFICAS ----------------
#define ESCALA_APRENDER    50      // puntos para la escala inicial (0.5 s)
#define ESCALA_BLOQUE      100     // cada cuántos puntos se reajusta hacia abajo (1 s)
#define ESCALA_FACTOR      0.5f    // fracción que se corrige en cada reajuste

// ---------------- ALARMA VISUAL (Nextion) ----------------
#define NEX_NIVEL_ALARMA     "vNivel"
#define VIS_BAJO             0          // azul fijo
#define VIS_MEDIO            1          // amarillo 0.5 Hz
#define VIS_ALTO             2          // rojo 2 Hz
#define VIS_TECNICA          3          // amarillo fijo

// ---------------- FRECUENCIAS DE MUESTREO ----------------
#define FS_ECG_HZ 500.0f
#define FS_PPG_HZ 250.0f

static const uint32_t PERIODO_ECG_US = 2000;
static const uint32_t PERIODO_PPG_US = 4000;

static const float FACTOR_VOLT = 1.2f / 2097151.0f;

static const uint32_t PERIODO_GRAFICA_MS = 10;
static const uint32_t PERIODO_INFO_MS    = 1000;

// Decimación para la gráfica (100 puntos/s por onda)
#define DECIM_ECG     5        // 500 Hz -> 100 Hz
#define PPG_PASO      2        // 250 Hz * 2/5 = 100 Hz
#define PPG_DIVISOR   5

// ---------------- FILTROS ----------------
#define ECG_FC_HZ    40.0f
#define ECG_HP_FC_HZ 0.5f
#define ECG_Q        0.7071f

#define PPG_FC_HZ    8.0f
#define PPG_HP_FC_HZ 0.5f
#define PPG_Q        0.7071f

#define PT_HP_FC_PPG 0.7f
#define PT_LP_FC_PPG 5.0f
#define PT_Q_PPG     0.7071f

#define PT_CUENTAS_POR_V_PPG 20000.0f
#define PT_DC_FC_HZ          0.3f
#define PT_CUENTAS_POR_V_ECG 100000.0f

static const float SPAN_MIN_ECG_V = 0.0002f;
static const float SPAN_MIN_PPG_V = 0.0020f;

#endif // CONFIG_H