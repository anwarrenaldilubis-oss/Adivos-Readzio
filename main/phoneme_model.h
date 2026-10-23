#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool readzio_phoneme_model_init(void);
bool readzio_phoneme_model_predict(
    const float features[98][40], int *predicted_class,
    float probabilities[5]);

#ifdef __cplusplus
}
#endif
