#include "biosignals.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

void biquad_reset(Biquad* f) {
    f->z1 = 0.0f;
    f->z2 = 0.0f;
}

/* Forma Directa II Transpuesta: robusta y con solo 2 estados. */
float biquad_process(Biquad* f, float x) {
    float y = f->b0 * x + f->z1;
    f->z1   = f->b1 * x - f->a1 * y + f->z2;
    f->z2   = f->b2 * x - f->a2 * y;
    return y;
}

/* Fórmulas del Audio EQ Cookbook (RBJ). Todas normalizan dividiendo por a0. */
void biquad_lowpass(Biquad* f, float fs, float fc, float Q) {
    float w0    = 2.0f * M_PI * fc / fs;
    float cw    = cosf(w0);
    float sw    = sinf(w0);
    float alpha = sw / (2.0f * Q);
    float a0    = 1.0f + alpha;

    f->b0 = ((1.0f - cw) * 0.5f) / a0;
    f->b1 =  (1.0f - cw)         / a0;
    f->b2 = ((1.0f - cw) * 0.5f) / a0;
    f->a1 =  (-2.0f * cw)        / a0;
    f->a2 =  (1.0f - alpha)      / a0;
    biquad_reset(f);
}

void biquad_highpass(Biquad* f, float fs, float fc, float Q) {
    float w0    = 2.0f * M_PI * fc / fs;
    float cw    = cosf(w0);
    float sw    = sinf(w0);
    float alpha = sw / (2.0f * Q);
    float a0    = 1.0f + alpha;

    f->b0 = ((1.0f + cw) * 0.5f) / a0;
    f->b1 = (-(1.0f + cw))       / a0;
    f->b2 = ((1.0f + cw) * 0.5f) / a0;
    f->a1 =  (-2.0f * cw)        / a0;
    f->a2 =  (1.0f - alpha)      / a0;
    biquad_reset(f);
}

/* Pasa-banda con ganancia de pico constante 0 dB. */
void biquad_bandpass(Biquad* f, float fs, float f0, float Q) {
    float w0    = 2.0f * M_PI * f0 / fs;
    float cw    = cosf(w0);
    float sw    = sinf(w0);
    float alpha = sw / (2.0f * Q);
    float a0    = 1.0f + alpha;

    f->b0 =  alpha  / a0;
    f->b1 =  0.0f;
    f->b2 = -alpha  / a0;
    f->a1 = (-2.0f * cw)   / a0;
    f->a2 = (1.0f - alpha) / a0;
    biquad_reset(f);
}

/* Rechaza-banda (notch): útil para 50/60 Hz de red eléctrica. */
void biquad_notch(Biquad* f, float fs, float f0, float Q) {
    float w0    = 2.0f * M_PI * f0 / fs;
    float cw    = cosf(w0);
    float sw    = sinf(w0);
    float alpha = sw / (2.0f * Q);
    float a0    = 1.0f + alpha;

    f->b0 =  1.0f          / a0;
    f->b1 = (-2.0f * cw)   / a0;
    f->b2 =  1.0f          / a0;
    f->a1 = (-2.0f * cw)   / a0;
    f->a2 = (1.0f - alpha) / a0;
    biquad_reset(f);
}

void chain_init(BiquadChain* c) {
    c->n = 0;
    for (int i = 0; i < BIO_MAX_STAGES; i++) {
        c->stage[i].b0 = 1.0f; c->stage[i].b1 = 0.0f; c->stage[i].b2 = 0.0f;
        c->stage[i].a1 = 0.0f; c->stage[i].a2 = 0.0f;
        biquad_reset(&c->stage[i]);
    }
}

void chain_reset(BiquadChain* c) {
    if (c->n < 0) c->n = 0;
    if (c->n > BIO_MAX_STAGES) c->n = BIO_MAX_STAGES;
    for (int i = 0; i < c->n; i++) biquad_reset(&c->stage[i]);
}

float chain_process(BiquadChain* c, float x) {
    float y = x;
    for (int i = 0; i < c->n; i++) y = biquad_process(&c->stage[i], y);
    return y;
}

void mwi_init(MovingIntegrator* m, int window) {
    if (window < 1)            window = 1;
    if (window > BIO_MWI_MAX)  window = BIO_MWI_MAX;
    m->size = window;
    mwi_reset(m);
}

void mwi_reset(MovingIntegrator* m) {
    memset(m->buf, 0, sizeof(m->buf));
    m->idx = 0;
    m->acc = 0.0f;
}

/* O(1): resta la muestra que sale, suma la que entra. */
float mwi_process(MovingIntegrator* m, float x) {
    m->acc -= m->buf[m->idx];   /* saca la más antigua */
    m->buf[m->idx] = x;         /* mete la nueva       */
    m->acc += x;
    m->idx = (m->idx + 1) % m->size;
    return m->acc / (float)m->size;
}

void deriv5_init(Derivative5* d)  { memset(d->x, 0, sizeof(d->x)); }
void deriv5_reset(Derivative5* d) { memset(d->x, 0, sizeof(d->x)); }

float deriv5_process(Derivative5* d, float x) {
    d->x[4] = d->x[3];
    d->x[3] = d->x[2];
    d->x[2] = d->x[1];
    d->x[1] = d->x[0];
    d->x[0] = x;
    return 0.125f * (2.0f*d->x[0] + d->x[1] - d->x[3] - 2.0f*d->x[4]);
}


/* --- Parámetros del algoritmo (en ms; se convierten a nº de muestras) --- */
#define PT_HP_FC     5.0f     /* corte pasa-altos del pasa-banda  */
#define PT_LP_FC    15.0f     /* corte pasa-bajos del pasa-banda  */
#define PT_BW_Q      0.707f   /* Q Butterworth                    */
#define PT_MWI_MS    150.0f   /* ancho de ventana de integración  */
#define PT_REFRACT_MS 200.0f  /* periodo refractario              */
#define PT_TWAVE_MS  360.0f   /* ventana de discriminación onda T */
#define PT_SETTLE_S  1.0f     /* asentamiento de filtros (se descarta) */
#define PT_LEARN_S   2.0f     /* fin de la fase de aprendizaje    */

static int ms_to_samples(float ms, float fs) {
    int s = (int)(ms * 0.001f * fs + 0.5f);
    return (s < 1) ? 1 : s;
}

void pt_init(PanTompkins* pt, float fs) {
    memset(pt, 0, sizeof(*pt));
    pt->fs = fs;

    /* Etapa 1: pasa-banda 5–15 Hz = pasa-altos 5 Hz seguido de pasa-bajos 15 Hz */
    biquad_highpass(&pt->hp, fs, PT_HP_FC, PT_BW_Q);
    biquad_lowpass (&pt->lp, fs, PT_LP_FC, PT_BW_Q);

    /* Etapa 4: integrador de ventana móvil (~150 ms) */
    int mwi_win = ms_to_samples(PT_MWI_MS, fs);
    mwi_init(&pt->mwi, mwi_win);

    /* Tiempos del detector */
    pt->refractory    = ms_to_samples(PT_REFRACT_MS, fs);
    pt->twave_win     = ms_to_samples(PT_TWAVE_MS,  fs);
    pt->learn_settle  = (uint32_t)(PT_SETTLE_S * fs);
    pt->learn_samples = (uint32_t)(PT_LEARN_S * fs);

    pt_reset(pt);
    pt->initialized = true;
}

void pt_reset(PanTompkins* pt) {
    biquad_reset(&pt->hp);
    biquad_reset(&pt->lp);
    mwi_reset(&pt->mwi);
    deriv5_reset(&pt->der);

    pt->spki = 0.0f; pt->npki = 0.0f;
    pt->threshold_i1 = 0.0f; pt->threshold_i2 = 0.0f;

    pt->prev_int = 0.0f; pt->peak_val = 0.0f; pt->rising = false;
    pt->cur_maxslope = 0.0f; pt->last_qrs_slope = 0.0f;

    pt->n = 0; pt->last_qrs_n = 0;
    memset(pt->rr_buf, 0, sizeof(pt->rr_buf));
    pt->rr_idx = 0; pt->rr_count = 0; pt->rr_avg = 0.0f; pt->rr_med = 0.0f;
    pt->last_qrs_us = 0; pt->use_time = false;

    pt->sb_peak = 0.0f; pt->sb_peak_n = 0; pt->sb_peak_slope = 0.0f;
    pt->sb_peak_us = 0;

    pt->learn_max = 0.0f; pt->learn_sum = 0.0f; pt->learn_count = 0;
    pt->learning = true;

    pt->bpm = 0.0f; pt->bpm_inst = 0.0f;
    pt->beats = 0;
}

/* Actualiza umbrales a partir de los estimadores de pico de señal/ruido. */
static void pt_update_thresholds(PanTompkins* pt) {
    pt->threshold_i1 = pt->npki + 0.25f * (pt->spki - pt->npki);
    pt->threshold_i2 = 0.5f * pt->threshold_i1;
}

/* Registra un QRS aceptado en la muestra 'qn' con valor de pico 'peak'. */
static void pt_register_qrs(PanTompkins* pt, uint32_t qn, uint32_t qn_us,
                            float peak, float slope, bool searchback) {
    /* Actualiza el estimador de pico de SEÑAL.
     * En búsqueda hacia atrás se usa un coeficiente mayor (0.25) para
     * recuperar la escala más rápido, como en el algoritmo original. */
    if (searchback) pt->spki = 0.25f * peak + 0.75f * pt->spki;
    else            pt->spki = 0.125f * peak + 0.875f * pt->spki;

    /* ---- Intervalo RR y FC ----
     * El RR se guarda SIEMPRE en milisegundos. Si hay base de tiempo real
     * (use_time) se mide con las marcas de micros(); si no, se deduce de la
     * fs declarada contando muestras. */
    bool tengo_previo = (pt->use_time) ? (pt->last_qrs_us != 0)
                                       : (pt->last_qrs_n  != 0);
    if (tengo_previo) {
        float rr_ms;
        if (pt->use_time) {
            /* resta segura ante desbordamiento de micros() (cada ~71 min) */
            uint32_t d = qn_us - pt->last_qrs_us;
            rr_ms = (float)d * 0.001f;
        } else {
            rr_ms = 1000.0f * (float)(qn - pt->last_qrs_n) / pt->fs;
        }

        float bpm = (rr_ms > 0.0f) ? (60000.0f / rr_ms) : 0.0f;

        /* Filtro de plausibilidad fisiológica (~30-220 bpm). */
        if (bpm >= 30.0f && bpm <= 220.0f) {
            pt->bpm_inst = bpm;
            pt->rr_buf[pt->rr_idx] = rr_ms;
            pt->rr_idx = (pt->rr_idx + 1) & 7;             /* módulo 8 */
            if (pt->rr_count < 8) pt->rr_count++;

            /* MEDIANA de los últimos RR (no promedio).
             * Un latido espurio o uno perdido altera mucho el promedio pero
             * casi nada la mediana: es el estimador robusto estándar en
             * monitores de FC. */
            float tmp[8];
            for (int i = 0; i < pt->rr_count; i++) tmp[i] = pt->rr_buf[i];
            for (int i = 1; i < pt->rr_count; i++) {       /* inserción, n<=8 */
                float v = tmp[i]; int j = i - 1;
                while (j >= 0 && tmp[j] > v) { tmp[j+1] = tmp[j]; j--; }
                tmp[j+1] = v;
            }
            int m = pt->rr_count / 2;
            pt->rr_med = (pt->rr_count & 1) ? tmp[m]
                                            : 0.5f * (tmp[m-1] + tmp[m]);

            pt->bpm    = 60000.0f / pt->rr_med;
            pt->rr_avg = pt->rr_med * pt->fs / 1000.0f;   /* en muestras */
        }
    }

    pt->beats++;
    pt->last_qrs_n     = qn;
    pt->last_qrs_us    = qn_us;
    pt->last_qrs_slope = slope;
    pt->sb_peak        = 0.0f;   /* limpia candidato de búsqueda hacia atrás */
    pt_update_thresholds(pt);
}

PT_Result pt_process(PanTompkins* pt, float sample) {
    return pt_process_t(pt, sample, 0);   /* 0 = sin base de tiempo real */
}

PT_Result pt_process_t(PanTompkins* pt, float sample, uint32_t t_us) {
    if (t_us != 0) pt->use_time = true;
    PT_Result r;
    r.raw = sample;
    pt->n++;

    /* ---- Etapa 1: pasa-banda 5–15 Hz ---- */
    float bp = biquad_process(&pt->hp, sample);
    bp       = biquad_process(&pt->lp, bp);
    r.bandpassed = bp;

    /* ---- Etapa 2: derivada de 5 puntos ----
     * y[n] = (1/8)*(2x[n] + x[n-1] - x[n-3] - 2x[n-4])
     * Resalta la pendiente (alta) del complejo QRS. */
    float deriv = deriv5_process(&pt->der, bp);
    r.derivative = deriv;

    /* Seguimiento de la pendiente máxima del latido en curso (para onda T). */
    float aderiv = fabsf(deriv);
    if (aderiv > pt->cur_maxslope) pt->cur_maxslope = aderiv;

    /* ---- Etapa 3: cuadrado ----
     * Hace todo positivo y amplifica no linealmente los picos altos. */
    float sq = deriv * deriv;
    r.squared = sq;

    /* ---- Etapa 4: integrador de ventana móvil ---- */
    float integ = mwi_process(&pt->mwi, sq);
    r.integrated = integ;

    /* ---- Fase de aprendizaje inicial: siembra spki/npki ----
     * Se descarta el primer segundo (asentamiento de los filtros IIR, cuyo
     * transitorio ante el escalón de DC del ADC produce un pico espurio muy
     * grande). Luego, durante ~1 s, se estiman:
     *   spki  = pico máximo de la señal integrada (nivel de SEÑAL/QRS)
     *   npki  = media de la señal integrada       (nivel de RUIDO de fondo) */
    if (pt->learning) {
        if (pt->n > pt->learn_settle) {
            if (integ > pt->learn_max) pt->learn_max = integ;
            pt->learn_sum += integ;
            pt->learn_count++;
        }
        if (pt->n >= pt->learn_samples) {
            float mean = (pt->learn_count > 0)
                       ? pt->learn_sum / (float)pt->learn_count : 0.0f;
            pt->spki = pt->learn_max;                 /* nivel de señal  */
            pt->npki = mean;                          /* nivel de ruido  */
            if (pt->spki <= pt->npki)                 /* guarda de seguridad */
                pt->spki = pt->npki * 2.0f + 1.0f;
            pt_update_thresholds(pt);
            pt->learning = false;
        }
        r.threshold = pt->threshold_i1;
        r.beat      = false;
        r.bpm       = pt->bpm;
        r.bpm_inst  = pt->bpm_inst;
        pt->prev_int = integ;
        return r;   /* aún no detectamos latidos */
    }

    /* ---- Etapa 5: detección con umbral adaptativo ---- */
    bool beat = false;

    /* (a) Detección de máximo local sobre la señal integrada.
     *     Mientras sube, seguimos el máximo; cuando empieza a bajar,
     *     el máximo seguido es un "pico candidato". */
    if (integ > pt->prev_int) {
        pt->rising = true;
        if (integ > pt->peak_val) pt->peak_val = integ;
    } else if (pt->rising && integ < pt->prev_int) {
        /* Se confirmó un pico candidato en la muestra anterior (n-1). */
        float peak   = pt->peak_val;
        uint32_t pn  = pt->n - 1;
        float slope  = pt->cur_maxslope;
        pt->rising   = false;
        pt->peak_val = 0.0f;

        if (peak >= pt->threshold_i1) {
            /* Supera el umbral: posible QRS si pasó el refractario. */
            if ((pn - pt->last_qrs_n) > (uint32_t)pt->refractory) {
                bool isQRS = true;

                /* Discriminación de onda T: si el candidato cae dentro de
                 * los 360 ms del QRS anterior y su pendiente es < 50% de la
                 * del QRS anterior, es una onda T (no un latido). */
                if ((pn - pt->last_qrs_n) < (uint32_t)pt->twave_win
                    && pt->last_qrs_slope > 0.0f) {
                    if (slope < 0.5f * pt->last_qrs_slope) isQRS = false;
                }

                if (isQRS) {
                    pt_register_qrs(pt, pn, t_us, peak, slope, false);
                    beat = true;
                } else {
                    /* Onda T -> se trata como ruido. */
                    pt->npki = 0.125f*peak + 0.875f*pt->npki;
                    pt_update_thresholds(pt);
                }
            }
            /* Si está dentro del refractario, se ignora (mismo QRS). */
        } else {
            /* Por debajo del umbral -> ruido. */
            pt->npki = 0.125f*peak + 0.875f*pt->npki;
            pt_update_thresholds(pt);

            /* Guardar el mejor candidato entre I2 e I1 para búsqueda atrás. */
            if (peak > pt->threshold_i2 && peak > pt->sb_peak) {
                pt->sb_peak       = peak;
                pt->sb_peak_n     = pn;
                pt->sb_peak_us    = t_us;
                pt->sb_peak_slope = slope;
            }
        }
        /* Reiniciar la pendiente para el siguiente segmento/latido. */
        pt->cur_maxslope = 0.0f;
    }

    /* (b) Búsqueda hacia atrás (search-back): si llevamos más de 1.66*RR_prom
     *     sin QRS y guardamos un candidato razonable, lo aceptamos ahora
     *     bajando de hecho el umbral a I2. */
    if (pt->rr_avg > 0.0f && pt->sb_peak > 0.0f) {
        float missed = 1.66f * pt->rr_avg;
        if ((float)(pt->n - pt->last_qrs_n) > missed) {
            pt_register_qrs(pt, pt->sb_peak_n, pt->sb_peak_us, pt->sb_peak,
                            pt->sb_peak_slope, true);
            beat = true;   /* marcador un poco tardío, pero el RR es correcto */
        }
    }

    pt->prev_int = integ;

    r.threshold = pt->threshold_i1;
    r.beat      = beat;
    r.bpm       = pt->bpm;
    r.bpm_inst  = pt->bpm_inst;
    return r;
}

float pt_bpm(const PanTompkins* pt)      { return pt->bpm; }
float pt_bpm_inst(const PanTompkins* pt) { return pt->bpm_inst; }
uint32_t pt_beats(const PanTompkins* pt) { return pt->beats; }

float pt_rr_ms(const PanTompkins* pt) { return pt->rr_med; }
