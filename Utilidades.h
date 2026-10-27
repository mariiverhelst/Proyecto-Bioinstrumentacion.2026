#ifndef UTILIDADES_H
#define UTILIDADES_H

#include "biosignals.h"

// Inicializa un biquad pasa-bajos en régimen permanente (salida = x0)
inline void biquadPrime(Biquad* filtro, float x0) {
  filtro->z2 = (filtro->b2 - filtro->a2) * x0;
  filtro->z1 = (filtro->b1 - filtro->a1 + filtro->b2 - filtro->a2) * x0;
}

// Inicializa un biquad pasa-altos en régimen permanente (salida = 0)
inline void biquadPrimeHP(Biquad* filtro, float x0) {
  filtro->z2 = filtro->b2 * x0;
  filtro->z1 = (filtro->b1 + filtro->b2) * x0;
}

#endif // UTILIDADES_H