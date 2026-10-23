#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define PHONEME_SAMPLE_RATE 16000
#define PHONEME_CLIP_SAMPLES 16000
#define PHONEME_FRAME_COUNT 98
#define PHONEME_FEATURE_COUNT 40

bool readzio_extract_phoneme_features(const int16_t *samples,
                                      size_t sample_count,
                                      float features[PHONEME_FRAME_COUNT]
                                                    [PHONEME_FEATURE_COUNT]);
