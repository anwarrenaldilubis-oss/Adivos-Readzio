#include "phoneme_features.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"

#define PHONEME_CAPTURE_MAX_SAMPLES (PHONEME_SAMPLE_RATE * 3)
#define MFCC_COUNT 13
#define FFT_SIZE WINDOW_SIZE
#define FFT_BINS (FFT_SIZE / 2 + 1)
#define MEL_BANDS 128
#define WINDOW_SIZE 400
#define HOP_SIZE 160
#define DELTA_WINDOW 9
#define PHONEME_PI 3.14159265358979323846f

static float *filtered_audio;
static float *mel_power;
static float *mel_filters;
static float *selected_clip;
static float mfcc[PHONEME_FRAME_COUNT][MFCC_COUNT];
static float delta_features[PHONEME_FRAME_COUNT][MFCC_COUNT];
static float delta2_features[PHONEME_FRAME_COUNT][MFCC_COUNT];
static float dct_basis[MFCC_COUNT][MEL_BANDS];
static float hann_window[WINDOW_SIZE];
static bool dct_basis_ready;
static bool audio_windows_ready;

static const float highpass_sos[2][6] = {
    {0.979684760567f, -1.959369521134f, 0.979684760567f,
     1.0f, -1.975269634852f, 0.976244792359f},
    {0.979684760567f, -1.959369521134f, 0.979684760567f,
     1.0f, -1.942638230540f, 0.943597278470f}
};

static float hz_to_mel(float frequency)
{
    if (frequency < 1000.0f) return frequency * (3.0f / 200.0f);
    return 15.0f + 27.0f * logf(frequency / 1000.0f) / logf(6.4f);
}

static float mel_to_hz(float mel)
{
    if (mel < 15.0f) return mel * (200.0f / 3.0f);
    return 1000.0f * expf((mel - 15.0f) * logf(6.4f) / 27.0f);
}

static void ensure_dct_basis(void)
{
    if (dct_basis_ready) return;
    for (int coefficient = 0; coefficient < MFCC_COUNT; ++coefficient) {
        float normalization = coefficient == 0
            ? sqrtf(1.0f / MEL_BANDS) : sqrtf(2.0f / MEL_BANDS);
        for (int band = 0; band < MEL_BANDS; ++band) {
            dct_basis[coefficient][band] =
                normalization * cosf(PHONEME_PI * coefficient *
                                     (2.0f * band + 1.0f) /
                                     (2.0f * MEL_BANDS));
        }
    }
    dct_basis_ready = true;
}

static bool ensure_scratch(void)
{
    if (filtered_audio == NULL) {
        filtered_audio = heap_caps_malloc(
            PHONEME_CAPTURE_MAX_SAMPLES * sizeof(*filtered_audio),
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (filtered_audio == NULL) return false;
    }
    if (mel_power == NULL) {
        mel_power = heap_caps_malloc(
            PHONEME_FRAME_COUNT * MEL_BANDS * sizeof(*mel_power),
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (mel_power == NULL) return false;
    }
    if (mel_filters == NULL) {
        mel_filters = heap_caps_malloc(
            MEL_BANDS * FFT_BINS * sizeof(*mel_filters),
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (mel_filters == NULL) return false;
    }
    if (selected_clip == NULL) {
        selected_clip = heap_caps_malloc(
            PHONEME_CLIP_SAMPLES * sizeof(*selected_clip),
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (selected_clip == NULL) return false;
    }
    return true;
}

static void apply_highpass(float *samples, size_t sample_count)
{
    for (size_t section = 0; section < 2; ++section) {
        const float *sos = highpass_sos[section];
        float x1 = 0.0f, x2 = 0.0f, y1 = 0.0f, y2 = 0.0f;
        for (size_t i = 0; i < sample_count; ++i) {
            float input = samples[i];
            float output = sos[0] * input + sos[1] * x1 + sos[2] * x2
                           - sos[4] * y1 - sos[5] * y2;
            samples[i] = output;
            x2 = x1;
            x1 = input;
            y2 = y1;
            y1 = output;
        }
    }
}

static int compare_float(const void *left, const void *right)
{
    float a = *(const float *)left;
    float b = *(const float *)right;
    return (a > b) - (a < b);
}

static float frame_rms(const float *samples, size_t sample_count, size_t center)
{
    size_t start = center > WINDOW_SIZE / 2
        ? center - WINDOW_SIZE / 2 : 0;
    size_t end = start + WINDOW_SIZE;
    if (end > sample_count) {
        end = sample_count;
        start = end > WINDOW_SIZE ? end - WINDOW_SIZE : 0;
    }
    double energy = 0.0;
    for (size_t i = start; i < end; ++i) {
        energy += (double)samples[i] * samples[i];
    }
    size_t length = end - start;
    return length == 0 ? 0.0f : sqrtf((float)(energy / length));
}

static bool select_speech_clip(const float *samples, size_t sample_count,
                               float clip[PHONEME_CLIP_SAMPLES])
{
    float rms[PHONEME_CAPTURE_MAX_SAMPLES / HOP_SIZE + 2];
    size_t frame_count = (sample_count + HOP_SIZE - 1) / HOP_SIZE;
    if (frame_count > sizeof(rms) / sizeof(rms[0])) return false;
    for (size_t frame = 0; frame < frame_count; ++frame) {
        rms[frame] = frame_rms(samples, sample_count, frame * HOP_SIZE);
    }

    float sorted[PHONEME_CAPTURE_MAX_SAMPLES / HOP_SIZE + 2];
    memcpy(sorted, rms, frame_count * sizeof(*rms));
    qsort(sorted, frame_count, sizeof(*sorted), compare_float);
    float noise = sorted[(frame_count - 1) / 10] + 1e-8f;
    float maximum = sorted[frame_count - 1];
    if (maximum < 1e-5f) return false;
    float threshold = fmaxf(noise * 3.0f, maximum * 0.15f);

    size_t first = frame_count, last = 0;
    for (size_t frame = 0; frame < frame_count; ++frame) {
        if (rms[frame] > threshold) {
            if (first == frame_count) first = frame;
            last = frame;
        }
    }
    if (first == frame_count) return false;

    size_t voice_start = first * HOP_SIZE;
    size_t voice_end = (last + 1) * HOP_SIZE;
    if (voice_end > sample_count) voice_end = sample_count;
    if (voice_end - voice_start >= PHONEME_CLIP_SAMPLES) {
        const size_t window = PHONEME_SAMPLE_RATE / 5;
        double energy = 0.0;
        for (size_t i = 0; i < window; ++i) energy += fabsf(samples[i]);
        size_t best_start = 0;
        double best_energy = energy;
        size_t max_start = sample_count - window;
        for (size_t start = 1; start <= max_start; ++start) {
            energy += fabsf(samples[start + window - 1]) -
                      fabsf(samples[start - 1]);
            if (energy > best_energy) {
                best_energy = energy;
                best_start = start;
            }
        }
        size_t center = best_start + window / 2;
        size_t start = center > PHONEME_CLIP_SAMPLES / 2
            ? center - PHONEME_CLIP_SAMPLES / 2 : 0;
        if (start + PHONEME_CLIP_SAMPLES > sample_count) {
            start = sample_count - PHONEME_CLIP_SAMPLES;
        }
        memcpy(clip, samples + start, sizeof(float) * PHONEME_CLIP_SAMPLES);
        return true;
    }

    size_t noise_frame = 0;
    for (size_t frame = 1; frame < frame_count; ++frame) {
        if (rms[frame] < rms[noise_frame]) noise_frame = frame;
    }
    size_t noise_start = noise_frame * HOP_SIZE;
    noise_start = noise_start > 800 ? noise_start - 800 : 0;
    if (noise_start + 1600 > sample_count) {
        noise_start = sample_count > 1600 ? sample_count - 1600 : 0;
    }

    int64_t requested_start =
        (int64_t)((voice_start + voice_end) / 2) -
        PHONEME_CLIP_SAMPLES / 2;
    for (size_t i = 0; i < PHONEME_CLIP_SAMPLES; ++i) {
        int64_t source = requested_start + (int64_t)i;
        if (source >= 0 && source < (int64_t)sample_count) {
            clip[i] = samples[source];
        } else {
            clip[i] = samples[noise_start +
                              (i % (sample_count - noise_start < 1600
                                        ? sample_count - noise_start : 1600))];
        }
    }
    return true;
}

static void normalize_audio(float clip[PHONEME_CLIP_SAMPLES])
{
    for (size_t i = 0; i < PHONEME_CLIP_SAMPLES; ++i) {
        filtered_audio[i] = fabsf(clip[i]);
    }
    float rank = 0.995f * (PHONEME_CLIP_SAMPLES - 1);
    qsort(filtered_audio, PHONEME_CLIP_SAMPLES, sizeof(*filtered_audio),
          compare_float);
    size_t lower = (size_t)rank;
    float fraction = rank - lower;
    float percentile =
        filtered_audio[lower] * (1.0f - fraction) +
        filtered_audio[lower + 1] * fraction + 1e-6f;
    for (size_t i = 0; i < PHONEME_CLIP_SAMPLES; ++i) {
        float normalized = clip[i] / percentile * 0.9f;
        clip[i] = fminf(1.0f, fmaxf(-1.0f, normalized));
    }
}

static void ensure_audio_windows(void)
{
    if (audio_windows_ready) return;
    ensure_dct_basis();
    for (int sample = 0; sample < WINDOW_SIZE; ++sample) {
        hann_window[sample] = 0.5f - 0.5f *
            cosf((2.0f * PHONEME_PI * sample) / WINDOW_SIZE);
    }
    float low_mel = hz_to_mel(0.0f);
    float high_mel = hz_to_mel(PHONEME_SAMPLE_RATE / 2.0f);
    float mel_step = (high_mel - low_mel) / (MEL_BANDS + 1);
    for (int band = 0; band < MEL_BANDS; ++band) {
        float left = mel_to_hz(low_mel + band * mel_step);
        float center = mel_to_hz(low_mel + (band + 1) * mel_step);
        float right = mel_to_hz(low_mel + (band + 2) * mel_step);
        for (int bin = 0; bin < FFT_BINS; ++bin) {
            float frequency =
                bin * (float)PHONEME_SAMPLE_RATE / FFT_SIZE;
            float weight = fmaxf(0.0f, fminf(
                (frequency - left) / (center - left),
                (right - frequency) / (right - center)));
            mel_filters[band * FFT_BINS + bin] =
                weight * (2.0f / (right - left));
        }
    }
    audio_windows_ready = true;
}

static void real_fft_power(const float frame[WINDOW_SIZE], float power[FFT_BINS])
{
    const float angle_step = (2.0f * PHONEME_PI) / FFT_SIZE;
    for (int bin = 0; bin < FFT_BINS; ++bin) {
        float angle = angle_step * bin;
        float step_real = cosf(angle);
        float step_imag = -sinf(angle);
        float real = 0.0f;
        float imag = 0.0f;
        float twiddle_real = 1.0f;
        float twiddle_imag = 0.0f;
        for (int sample = 0; sample < WINDOW_SIZE; ++sample) {
            float value = frame[sample] * hann_window[sample];
            real += value * twiddle_real;
            imag += value * twiddle_imag;
            float next_real = twiddle_real * step_real -
                              twiddle_imag * step_imag;
            twiddle_imag = twiddle_real * step_imag +
                           twiddle_imag * step_real;
            twiddle_real = next_real;
        }
        power[bin] = real * real + imag * imag;
    }
}

static void solve_polynomial(const float values[DELTA_WINDOW],
                             double coefficients[3])
{
    double matrix[3][4] = {
        {9.0, 36.0, 204.0, 0.0},
        {36.0, 204.0, 1296.0, 0.0},
        {204.0, 1296.0, 8772.0, 0.0}
    };
    for (int i = 0; i < DELTA_WINDOW; ++i) {
        double value = values[i];
        matrix[0][3] += value;
        matrix[1][3] += i * value;
        matrix[2][3] += i * i * value;
    }
    for (int column = 0; column < 3; ++column) {
        double divisor = matrix[column][column];
        for (int j = column; j < 4; ++j) matrix[column][j] /= divisor;
        for (int row = 0; row < 3; ++row) {
            if (row == column) continue;
            double factor = matrix[row][column];
            for (int j = column; j < 4; ++j) {
                matrix[row][j] -= factor * matrix[column][j];
            }
        }
    }
    for (int i = 0; i < 3; ++i) coefficients[i] = matrix[i][3];
}

static void calculate_delta(int order, float output[PHONEME_FRAME_COUNT][MFCC_COUNT])
{
    float edge_values[DELTA_WINDOW];
    for (int coefficient = 0; coefficient < MFCC_COUNT; ++coefficient) {
        for (int frame = 4; frame < PHONEME_FRAME_COUNT - 4; ++frame) {
            float value = 0.0f;
            for (int offset = -4; offset <= 4; ++offset) {
                float weight;
                if (order == 1) {
                    weight = offset / 60.0f;
                } else {
                    weight = (float)(offset * offset - 20.0 / 3.0) / 154.0f;
                }
                value += weight * mfcc[frame + offset][coefficient];
            }
            output[frame][coefficient] = value;
        }

        for (int i = 0; i < DELTA_WINDOW; ++i) {
            edge_values[i] = mfcc[i][coefficient];
        }
        double polynomial[3];
        solve_polynomial(edge_values, polynomial);
        for (int frame = 0; frame < 4; ++frame) {
            output[frame][coefficient] = order == 1
                ? (float)(polynomial[1] + 2.0 * polynomial[2] * frame)
                : (float)(2.0 * polynomial[2]);
        }

        for (int i = 0; i < DELTA_WINDOW; ++i) {
            edge_values[i] =
                mfcc[PHONEME_FRAME_COUNT - DELTA_WINDOW + i][coefficient];
        }
        solve_polynomial(edge_values, polynomial);
        for (int frame = PHONEME_FRAME_COUNT - 4;
             frame < PHONEME_FRAME_COUNT; ++frame) {
            int position = frame - (PHONEME_FRAME_COUNT - DELTA_WINDOW);
            output[frame][coefficient] = order == 1
                ? (float)(polynomial[1] + 2.0 * polynomial[2] * position)
                : (float)(2.0 * polynomial[2]);
        }
    }
}

bool readzio_extract_phoneme_features(
    const int16_t *samples, size_t sample_count,
    float features[PHONEME_FRAME_COUNT][PHONEME_FEATURE_COUNT])
{
    if (samples == NULL || features == NULL || sample_count < PHONEME_SAMPLE_RATE ||
        sample_count > PHONEME_CAPTURE_MAX_SAMPLES || !ensure_scratch()) {
        return false;
    }

    double mean = 0.0;
    for (size_t i = 0; i < sample_count; ++i) mean += samples[i] / 32768.0;
    mean /= sample_count;
    for (size_t i = 0; i < sample_count; ++i) {
        filtered_audio[i] = samples[i] / 32768.0f - (float)mean;
    }
    apply_highpass(filtered_audio, sample_count);

    if (!select_speech_clip(filtered_audio, sample_count, selected_clip)) {
        return false;
    }
    normalize_audio(selected_clip);
    ensure_audio_windows();

    float frame[WINDOW_SIZE];
    float power[FFT_BINS];
    float maximum_mel_power = 1e-10f;
    for (int frame_index = 0; frame_index < PHONEME_FRAME_COUNT; ++frame_index) {
        const float *source = selected_clip + frame_index * HOP_SIZE;
        memcpy(frame, source, sizeof(frame));
        real_fft_power(frame, power);
        for (int band = 0; band < MEL_BANDS; ++band) {
            float energy = 0.0f;
            for (int bin = 0; bin < FFT_BINS; ++bin) {
                energy += power[bin] * mel_filters[band * FFT_BINS + bin];
            }
            if (energy < 1e-10f) energy = 1e-10f;
            mel_power[frame_index * MEL_BANDS + band] = energy;
            if (energy > maximum_mel_power) maximum_mel_power = energy;
        }
    }

    float max_db = 10.0f * log10f(maximum_mel_power);
    for (int frame_index = 0; frame_index < PHONEME_FRAME_COUNT; ++frame_index) {
        for (int coefficient = 0; coefficient < MFCC_COUNT; ++coefficient) {
            float value = 0.0f;
            for (int band = 0; band < MEL_BANDS; ++band) {
                float db = 10.0f * log10f(
                    mel_power[frame_index * MEL_BANDS + band]);
                if (db < max_db - 80.0f) db = max_db - 80.0f;
                value += db * dct_basis[coefficient][band];
            }
            mfcc[frame_index][coefficient] = value;
            features[frame_index][coefficient] = value;
        }
    }

    calculate_delta(1, delta_features);
    calculate_delta(2, delta2_features);
    for (int frame_index = 0; frame_index < PHONEME_FRAME_COUNT; ++frame_index) {
        for (int coefficient = 0; coefficient < MFCC_COUNT; ++coefficient) {
            features[frame_index][MFCC_COUNT + coefficient] =
                delta_features[frame_index][coefficient];
            features[frame_index][2 * MFCC_COUNT + coefficient] =
                delta2_features[frame_index][coefficient];
        }

        const float *source = selected_clip + frame_index * HOP_SIZE;
        size_t crossings = 0;
        for (int sample = 1; sample < WINDOW_SIZE; ++sample) {
            if ((source[sample - 1] >= 0.0f && source[sample] < 0.0f) ||
                (source[sample - 1] < 0.0f && source[sample] >= 0.0f)) {
                ++crossings;
            }
        }
        features[frame_index][39] = crossings / (float)WINDOW_SIZE;
    }

    return true;
}
