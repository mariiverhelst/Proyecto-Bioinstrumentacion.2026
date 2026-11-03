#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================================
// CONFIGURACIÓN - ECG + PPG + SpO2 + PA + FR + ALARMAS
// ============================================================

// ---------------- PINES AFE4950 ----------------
#define SCLK      18
#define MISO      19
#define MOSI      23
#define CS         5
#define AFE_RST    4
#define FIFO_RDY  16

// ---------------- BUZZER (12 V con transistor NPN, base por 1 kΩ) ----------------
#define USAR_BUZZER         1      // 0 = sin sonido (prueben primero así) | 1 = con sonido
#define USAR_BIP_PULSO      1      // 1 = bip por latido cuando no hay alarma
#define PIN_BUZZER          25
#define BUZZER_DURACION_MS  40
#define BUZZER_FREQ_HZ      2700   // bip de pulso (no es alarma)

// ---------------- ALARMAS ----------------
#define ALARMA_FREQ_HZ      540    // fundamental 150-1000 Hz; armónicos 1620, 2700, 3780
#define ESCALAR_SEG         5      // segundos sostenidos para SUBIR de nivel
#define DESESCALAR_SEG      10     // segundos sostenidos para BAJAR de nivel
#define TECNICA_SEG         3      // segundos sostenidos para la alarma técnica

#define SIMULAR_NEWS2       0      // 1 = recorre normal -> medio -> alto -> normal (20 s cada fase)
#define USAR_PAS_EN_NEWS2   1      // 1 = la PA calculada entra al NEWS2
#define USAR_FR_EN_NEWS2    1      // 1 = la FR estimada entra al NEWS2
#define USAR_TEMP_EN_NEWS2  0      // 1 cuando el sensor de temperatura esté listo

// ---------------- NEXTION ----------------
#define NEX_BAUD     115200
#define ID_WAVE_ECG  1
#define ID_WAVE_PPG  2
#define CANAL        0
#define ALTO_WAVE    199
#define INVERTIR_ECG true
#define INVERTIR_PPG true         // con la PCB la PPG quedó invertida

// Alarma visual: el timer tmAlarm de la Nextion hace el parpadeo
#define NEX_NIVEL_ALARMA  "vNivel"
#define VIS_BAJO          0       // azul fijo
#define VIS_MEDIO         1       // amarillo 0.5 Hz
#define VIS_ALTO          2       // rojo 2 Hz
#define VIS_TECNICA       3       // amarillo fijo

// Diagnóstico: nTemp = resincronizaciones del ciclo ECG/PPG
#define MOSTRAR_DIAG 0

// Memoria: nTemp = memoria mínima libre desde el arranque (KB)
#define MOSTRAR_MEMORIA 1

// Modo calibración de SpO2 (reemplaza los números en pantalla):
// nHR = R x100 | nPR = PI x100 (%) | nSpO2 = DC IR (miles) | nSBP = estado librería | nDBP = estado efectivo
#define CALIBRAR_SPO2 0

// Línea recta en el ECG cuando no hay latidos durante TIMEOUT_LATIDO_MS
#define MOSTRAR_ECG_PLANO 1

// ---------------- AUTOESCALADO ----------------
#define ESCALA_APRENDER  50
#define ESCALA_BLOQUE    100
#define ESCALA_FACTOR    0.5f
static const float SPAN_MIN_ECG_V = 0.0002f;
static const float SPAN_MIN_PPG_V = 0.0020f;

// ---------------- MUESTREO ----------------
#define FS_ECG_HZ 500.0f
#define FS_PPG_HZ 250.0f
static const uint32_t PERIODO_ECG_US     = 2000;
static const uint32_t PERIODO_PPG_US     = 4000;
static const float    FACTOR_VOLT        = 1.2f / 2097151.0f;
static const uint32_t PERIODO_GRAFICA_MS = 10;
static const uint32_t PERIODO_INFO_MS    = 1000;

#define DECIM_ECG    5            // 500 Hz -> 100 Hz
#define PPG_PASO     2            // 250 Hz * 2/5 = 100 Hz
#define PPG_DIVISOR  5

// ---------------- FILTROS (gráficas) ----------------
#define ECG_FC_HZ    40.0f
#define ECG_HP_FC_HZ 0.5f
#define ECG_Q        0.7071f

#define PPG_FC_HZ    8.0f
#define PPG_HP_FC_HZ 0.5f
#define PPG_Q        0.7071f

// ---------------- PAN-TOMPKINS ----------------
#define PT_CUENTAS_POR_V_ECG 100000.0f
#define PT_CUENTAS_POR_V_PPG 20000.0f
#define PT_HP_FC_PPG         0.7f
#define PT_LP_FC_PPG         5.0f
#define PT_Q_PPG             0.7071f
#define PT_DC_FC_HZ          0.3f

// ---------------- FC / PR ----------------
#define TIMEOUT_LATIDO_MS    4000
#define ECG_REINICIO_MS      10000
#define PPG_REAPRENDER_MS    4000
#define PPG_REINICIO_MS      5000

// ---------------- VALIDACIÓN SpO2 ----------------
#define SPO2_VALIDAR        1
#define SPO2_R_MIN          0.3f
#define SPO2_R_MAX          1.6f
#define SPO2_PI_MIN         0.2f
#define SPO2_PI_MAX         20.0f
#define SPO2_VALIDAR_DC     1
#define SPO2_DC_MIN         24000UL    // PCB: dedo ~36 000 | pinza cerrada ~14 000 | abierta ~15 000
#define SPO2_DC_MAX         8388607UL
#define SPO2_SEG_INVALIDO   3
#define SPO2_SEG_VALIDO     2

// ---------------- PRESIÓN ARTERIAL (PTT + PIR) ----------------
#define FS_PA        500
#define USAR_PIR     1
#define SBP_DEF      120.0f
#define DBP_DEF      80.0f

#define N_BUF        2048
#define MS(x)        ((uint32_t)((x) * FS_PA / 1000))
#define B(buf, i)    buf[(i) % N_BUF]

#define NPROM        8
#define N_CAL        5

// ---------------- FRECUENCIA RESPIRATORIA ESTIMADA ----------------
#define NEX_OBJ_FR           "nFR"
#define NEX_OBJ_FR_TXT       "tFRest"
#define USAR_TEXTO_ESTADO_FR 0
#define FR_CAL_MS            5000UL
#define DEBUG_FR_SERIAL1     0
#define DEBUG_TX_PIN         33

#define FR_EST_CAL_RELOJ     10
#define FR_EST_CONFIG        11

// ---------------- RESINCRONIZACIÓN DEL CICLO ECG, ECG, PPG ----------------
#define SYNC_MUESTRAS   300
#define SYNC_UMBRAL_V   0.001f
#define SYNC_RAZON      5.0f

#endif // CONFIG_H