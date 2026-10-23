#include "phoneme_model.h"

#include <new>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

extern "C" {
extern const uint8_t _binary_model_int8_tflite_start[];
extern const uint8_t _binary_model_int8_tflite_end[];
}

namespace {
constexpr size_t kTensorArenaBytes = 320 * 1024;
constexpr int kInputFrames = 98;
constexpr int kFeatureCount = 40;
constexpr int kClassCount = 5;
constexpr float kTrainingInputScale = 0.0919705256819725f;
constexpr int kTrainingInputZeroPoint = 2;
constexpr float kTrainingOutputScale = 1.0f / 256.0f;
constexpr int kTrainingOutputZeroPoint = -128;
constexpr char kTag[] = "READZIO_MODEL";

constexpr float kFeatureMean[kFeatureCount] = {
    -339.6122131f, 23.09866333f, 4.668213844f, -3.578094006f,
    -8.776672363f, 0.384624809f, -3.470382929f, -7.840668201f,
    -3.559235573f, -2.164437056f, -4.228971958f, -2.469321489f,
    -4.205699444f, -0.01004631817f, -0.005854757037f, 0.002757436829f,
    0.004528416786f, 0.001590095926f, -0.001189442235f, 0.001715045539f,
    0.001277263509f, 0.002720107092f, 0.00176767644f, -0.003035863861f,
    -0.003049307968f, 0.003203653498f, -0.0008848887519f, -0.0009827348404f,
    -0.004057403188f, -0.00244983891f, -0.0006158567849f, -0.001966833137f,
    0.001579201664f, -0.000773440348f, -0.001123472583f, -0.003112567123f,
    -0.003219017526f, -0.001771220472f, -0.0006947230431f, 0.2994469702f
};
constexpr float kFeatureStd[kFeatureCount] = {
    145.5894775f, 47.16973877f, 28.94118309f, 21.07360077f,
    22.65388298f, 13.7871542f, 12.69756794f, 14.30176544f,
    11.06309795f, 10.42101669f, 11.57301903f, 9.060370445f,
    10.0138979f, 10.36643028f, 4.815740108f, 3.017002106f,
    2.375484467f, 2.696662188f, 1.675930262f, 1.55682528f,
    1.644677281f, 1.36611104f, 1.344383717f, 1.431610107f,
    1.184922814f, 1.26524663f, 3.975939035f, 2.108600855f,
    1.519786716f, 1.346078396f, 1.371322155f, 1.037614465f,
    0.9739592671f, 0.9706818461f, 0.8817816377f, 0.8798680305f,
    0.908077836f, 0.8241382837f, 0.8311666846f, 0.2041695118f
};

const tflite::Model *model;
tflite::MicroMutableOpResolver<9> resolver;
tflite::MicroInterpreter *interpreter;
uint8_t *tensor_arena;
bool initialized;
bool initialization_failed;

bool add_model_operators()
{
    return resolver.AddConv2D() == kTfLiteOk &&
           resolver.AddMul() == kTfLiteOk &&
           resolver.AddAdd() == kTfLiteOk &&
           resolver.AddReshape() == kTfLiteOk &&
           resolver.AddExpandDims() == kTfLiteOk &&
           resolver.AddMaxPool2D() == kTfLiteOk &&
           resolver.AddMean() == kTfLiteOk &&
           resolver.AddFullyConnected() == kTfLiteOk &&
           resolver.AddSoftmax() == kTfLiteOk;
}
}  // namespace

bool readzio_phoneme_model_init(void)
{
    if (initialized) return true;
    if (initialization_failed) return false;

    model = tflite::GetModel(_binary_model_int8_tflite_start);
    if (model == nullptr || model->version() != TFLITE_SCHEMA_VERSION) {
        ESP_LOGE(kTag, "Schema model TFLite tidak cocok dengan runtime.");
        initialization_failed = true;
        return false;
    }

    if (!add_model_operators()) {
        ESP_LOGE(kTag, "Registrasi operator TFLite gagal.");
        initialization_failed = true;
        return false;
    }

    tensor_arena = static_cast<uint8_t *>(heap_caps_malloc(
        kTensorArenaBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (tensor_arena == nullptr) {
        ESP_LOGE(kTag, "Tidak dapat mengalokasikan tensor arena %u byte di PSRAM.",
                 static_cast<unsigned>(kTensorArenaBytes));
        initialization_failed = true;
        return false;
    }

    void *interpreter_memory =
        heap_caps_malloc(sizeof(tflite::MicroInterpreter),
                         MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (interpreter_memory == nullptr) {
        ESP_LOGE(kTag, "Tidak dapat mengalokasikan interpreter di RAM internal.");
        heap_caps_free(tensor_arena);
        tensor_arena = nullptr;
        initialization_failed = true;
        return false;
    }

    interpreter = new (interpreter_memory)
        tflite::MicroInterpreter(model, resolver, tensor_arena,
                                 kTensorArenaBytes);
    if (interpreter->AllocateTensors() != kTfLiteOk) {
        ESP_LOGE(kTag, "Alokasi tensor gagal; arena %u byte tidak cukup.",
                 static_cast<unsigned>(kTensorArenaBytes));
        interpreter->~MicroInterpreter();
        heap_caps_free(interpreter_memory);
        heap_caps_free(tensor_arena);
        interpreter = nullptr;
        tensor_arena = nullptr;
        initialization_failed = true;
        return false;
    }

    TfLiteTensor *input = interpreter->input(0);
    TfLiteTensor *output = interpreter->output(0);
    if (input == nullptr || output == nullptr ||
        input->type != kTfLiteInt8 || output->type != kTfLiteInt8 ||
        input->dims->size != 3 || input->dims->data[0] != 1 ||
        input->dims->data[1] != kInputFrames ||
        input->dims->data[2] != kFeatureCount ||
        output->dims->size != 2 || output->dims->data[0] != 1 ||
        output->dims->data[1] != kClassCount) {
        ESP_LOGE(kTag, "Bentuk/tipe tensor model tidak sesuai konfigurasi Menu 3.");
        interpreter->~MicroInterpreter();
        heap_caps_free(interpreter_memory);
        heap_caps_free(tensor_arena);
        interpreter = nullptr;
        tensor_arena = nullptr;
        initialization_failed = true;
        return false;
    }

    initialized = true;
    ESP_LOGI(kTag, "Model int8 siap; arena tensor %u byte di PSRAM.",
             static_cast<unsigned>(kTensorArenaBytes));
    return true;
}

bool readzio_phoneme_model_predict(
    const float features[98][40], int *predicted_class,
    float probabilities[5])
{
    if (features == nullptr || predicted_class == nullptr ||
        probabilities == nullptr || !readzio_phoneme_model_init()) {
        return false;
    }

    TfLiteTensor *input = interpreter->input(0);
    TfLiteTensor *output = interpreter->output(0);
    if (input->params.scale <= 0.0f || output->params.scale <= 0.0f ||
        input->params.scale != kTrainingInputScale ||
        input->params.zero_point != kTrainingInputZeroPoint ||
        output->params.scale != kTrainingOutputScale ||
        output->params.zero_point != kTrainingOutputZeroPoint) {
        ESP_LOGE(kTag, "Kuantisasi tensor model berubah; prediksi dibatalkan.");
        return false;
    }

    for (int frame = 0; frame < kInputFrames; ++frame) {
        for (int feature = 0; feature < kFeatureCount; ++feature) {
            float normalized = (features[frame][feature] -
                                kFeatureMean[feature]) / kFeatureStd[feature];
            float quantized = normalized / input->params.scale +
                              input->params.zero_point;
            int value = static_cast<int>(quantized >= 0.0f
                                             ? quantized + 0.5f
                                             : quantized - 0.5f);
            if (value < -128) value = -128;
            if (value > 127) value = 127;
            input->data.int8[frame * kFeatureCount + feature] =
                static_cast<int8_t>(value);
        }
    }

    if (interpreter->Invoke() != kTfLiteOk) {
        ESP_LOGE(kTag, "Inferensi TFLite gagal.");
        return false;
    }

    int best_class = 0;
    for (int index = 0; index < kClassCount; ++index) {
        probabilities[index] =
            (output->data.int8[index] - output->params.zero_point) *
            output->params.scale;
        if (probabilities[index] > probabilities[best_class]) {
            best_class = index;
        }
    }
    *predicted_class = best_class;
    return true;
}
