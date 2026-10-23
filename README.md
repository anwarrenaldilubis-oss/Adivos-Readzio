# Readzio

ESP-IDF firmware for an ESP32-S3 reading and phoneme-learning device.

## Project structure

| Path | Purpose |
| --- | --- |
| `main/main.c` | UI, touch handling, lessons, audio playback, and app entry point |
| `main/activity_log.c` | Saves activity results |
| `main/phoneme_features.c` | Extracts audio features for vowel recognition |
| `main/phoneme_model.cc` | Loads and runs the phoneme-recognition model |
| `main/CMakeLists.txt` | Firmware sources, embedded assets, and component dependencies |
| `main/idf_component.yml` | ESP-IDF component dependencies |
| `Aset/` | Local audio assets; intentionally excluded from Git |
| `Hasil_train/` | Local model and normalization files required to build Menu 3; excluded from Git |
| `scripts/` | Audio conversion and activity-export utilities |
| `unity-app/` | Separate ESP-IDF Unity test project |
| `partitions.csv` | ESP32 flash partition layout |
| `sdkconfig.defaults` | Default ESP32-S3 project configuration |

### Learning menus

1. **Mengenal Huruf** — learn uppercase and lowercase letters A–Z.
2. **Belajar AIUEO** — learn the vowels A, I, U, E, O.
3. **Latihan Fonem** — pronounce a vowel for microphone recording and model classification.
4. **Latihan Bentuk** — identify vowels from audio prompts.
5. **Review / Awan Vokal** — find the vowel named in the audio prompt.

## Build and flash

Install ESP-IDF, connect the ESP32-S3, and provide the required audio files under
`Aset/` (`Sound_menu1/`–`Sound_menu5/` and `welcome_readzio.wav`). Audio is not
included in this repository; CMake requires it to build the firmware. The local
`Hasil_train/model_int8.tflite` and `Hasil_train/norm_stats.json` files are also
required by the firmware and are excluded from Git.

From the project root, run:

```powershell
idf.py set-target esp32s3
idf.py build
idf.py -p COM3 flash monitor
```

Replace `COM3` with the serial port for your board. The defaults target 16 MB
flash and 8 MB octal PSRAM; check your board and pin assignments before flashing.
