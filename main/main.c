#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "activity_log.h"
#if CONFIG_READZIO_ENABLE_NETWORK
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_http_client.h"
#endif
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2s_std.h"
#include "esp_rom_sys.h"
#include "phoneme_features.h"
#include "phoneme_model.h"

/* Readzio Re:Learn UI for an ILI9488 SPI panel.
 * Wiring: SCLK=12, MOSI=11, MISO=13, LCD CS=10, DC=9,
 * RESET=8, backlight=18, touch CS=14, IRQ=21.
 */
#if CONFIG_READZIO_ENABLE_NETWORK
#define WIFI_SSID      CONFIG_READZIO_WIFI_SSID
#define WIFI_PASSWORD  CONFIG_READZIO_WIFI_PASSWORD
#define WIFI_MAX_RETRY 10
#define BACKEND_HEALTH_URL CONFIG_READZIO_BACKEND_HEALTH_URL
#endif

#define LCD_PIN_SCLK 12
#define LCD_PIN_MOSI 11
#define LCD_PIN_MISO 13
#define LCD_PIN_CS   10
#define LCD_PIN_DC    9
#define LCD_PIN_RST   8
#define LCD_PIN_BL   18
#define LCD_WIDTH   480
#define LCD_HEIGHT  320

static const char *TAG = "READZIO";
#if CONFIG_READZIO_ENABLE_NETWORK
static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static int retry_count = 0;
#endif
static spi_device_handle_t lcd_spi;

static void lcd_write_cmd(uint8_t cmd)
{
    gpio_set_level(LCD_PIN_DC, 0);
    spi_transaction_t t = { .length = 8, .tx_buffer = &cmd };
    ESP_ERROR_CHECK(spi_device_polling_transmit(lcd_spi, &t));
}
static void lcd_write_data(const uint8_t *data, size_t len)
{
    if (!len) return;
    gpio_set_level(LCD_PIN_DC, 1);
    spi_transaction_t t = { .length = len * 8, .tx_buffer = data };
    ESP_ERROR_CHECK(spi_device_polling_transmit(lcd_spi, &t));
}
static void lcd_write_u8(uint8_t value) { lcd_write_data(&value, 1); }

static void lcd_init(void)
{
    gpio_config_t out = {
        .pin_bit_mask = (1ULL << LCD_PIN_DC) | (1ULL << LCD_PIN_RST),
        .mode = GPIO_MODE_OUTPUT, .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE, .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&out));
    /* Backlight GPIO is set HIGH only after display initialization. */
    gpio_set_level(LCD_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(LCD_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(LCD_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(150));

    spi_bus_config_t bus = {
        .mosi_io_num = LCD_PIN_MOSI, .miso_io_num = LCD_PIN_MISO,
        .sclk_io_num = LCD_PIN_SCLK, .quadwp_io_num = -1,
        .quadhd_io_num = -1, .max_transfer_sz = 4096
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));
    spi_device_interface_config_t dev = {
        .clock_speed_hz = 10 * 1000 * 1000, .mode = 0,
        .spics_io_num = LCD_PIN_CS, .queue_size = 1
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev, &lcd_spi));

    lcd_write_cmd(0x01); vTaskDelay(pdMS_TO_TICKS(120)); /* SWRESET */
    lcd_write_cmd(0x11); vTaskDelay(pdMS_TO_TICKS(120)); /* SLPOUT */
    lcd_write_cmd(0x3A); lcd_write_u8(0x66);             /* RGB666 */
    lcd_write_cmd(0x36); lcd_write_u8(0x28);             /* landscape + BGR */
    lcd_write_cmd(0xB0); lcd_write_u8(0x00);
    lcd_write_cmd(0xB1); { const uint8_t d[]={0xA0}; lcd_write_data(d,sizeof d); }
    lcd_write_cmd(0xB4); lcd_write_u8(0x02);
    lcd_write_cmd(0xC0); { const uint8_t d[]={0x17,0x15}; lcd_write_data(d,sizeof d); }
    lcd_write_cmd(0xC1); lcd_write_u8(0x41);
    lcd_write_cmd(0xC5); { const uint8_t d[]={0x00,0x12,0x80}; lcd_write_data(d,sizeof d); }
    lcd_write_cmd(0xE0); { const uint8_t d[]={0x00,0x03,0x09,0x08,0x16,0x0A,0x3F,0x78,0x4C,0x09,0x0A,0x08,0x16,0x1A,0x0F}; lcd_write_data(d,sizeof d); }
    lcd_write_cmd(0xE1); { const uint8_t d[]={0x00,0x16,0x19,0x03,0x0F,0x05,0x32,0x45,0x46,0x04,0x0E,0x0D,0x35,0x37,0x0F}; lcd_write_data(d,sizeof d); }
    lcd_write_cmd(0x29); vTaskDelay(pdMS_TO_TICKS(20)); /* DISPON */

    gpio_config_t bl = {
        .pin_bit_mask = (1ULL << LCD_PIN_BL), .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&bl));
    gpio_set_level(LCD_PIN_BL, 1);
    ESP_LOGI(TAG, "LCD init selesai; mulai tes warna.");
}

static void lcd_fill(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t col[4] = {0, 0, (LCD_WIDTH - 1) >> 8, (LCD_WIDTH - 1) & 0xff};
    uint8_t row[4] = {0, 0, (LCD_HEIGHT - 1) >> 8, (LCD_HEIGHT - 1) & 0xff};
    lcd_write_cmd(0x2A); lcd_write_data(col, sizeof col);
    lcd_write_cmd(0x2B); lcd_write_data(row, sizeof row);
    lcd_write_cmd(0x2C);
    uint8_t line[LCD_WIDTH * 3];
    for (int x = 0; x < LCD_WIDTH; ++x) { line[x*3]=r & 0xFC; line[x*3+1]=g & 0xFC; line[x*3+2]=b & 0xFC; }
    for (int y = 0; y < LCD_HEIGHT; ++y) lcd_write_data(line, sizeof line);
}


/* --- Readzio Re:Learn UI and XPT2046 navigation --- */
#define TOUCH_PIN_CS 14
#define TOUCH_PIN_IRQ 21
#define TOUCH_RAW_X_MIN 300
#define TOUCH_RAW_X_MAX 3800
#define TOUCH_RAW_Y_MIN 300
#define TOUCH_RAW_Y_MAX 3800
#define TOUCH_SWAP_XY 1
#define TOUCH_INVERT_X 1
#define TOUCH_INVERT_Y 1
#define TOUCH_PRESS_DEBOUNCE_MS 20
#define TOUCH_RELEASE_DEBOUNCE_MS 30
#define TOUCH_POLL_INTERVAL_MS 10
#define TOUCH_DIAGNOSTIC_INTERVAL_MS 3000
#define SPEAKER_PIN_BCLK GPIO_NUM_15
#define SPEAKER_PIN_WS GPIO_NUM_16
#define SPEAKER_PIN_DOUT GPIO_NUM_17
#define SPEAKER_PIN_ENABLE GPIO_NUM_38
#define SPEAKER_SAMPLE_RATE 32000
#define BEEP_TONE_SAMPLES 1440      /* 45 ms at 32 kHz */
#define BEEP_BUFFER_SAMPLES 1600    /* 50 ms including silence */
#define SPEAKER_CHANNELS 2
#define BEEP_AMPLITUDE 4915         /* 15% of signed 16-bit full scale */
#define MIC_PIN_BCLK GPIO_NUM_4
#define MIC_PIN_WS GPIO_NUM_5
#define MIC_PIN_DOUT GPIO_NUM_6
#define MIC_SAMPLE_RATE 16000
#define MIC_TEST_DURATION_MS 5000
#define MIC_READ_TIMEOUT_MS 100
#define MIC_RX_BUFFER_WORDS 256
#define PHONEME_CAPTURE_DURATION_MS 3000
#define PHONEME_CLASS_COUNT 5
static spi_device_handle_t touch_spi;
/* Protect multi-transaction LCD rendering from interleaving with touch SPI reads. */
static SemaphoreHandle_t spi_shared_bus_mutex;
static i2s_chan_handle_t speaker_tx_channel;
static TaskHandle_t speaker_beep_task_handle;
static bool speaker_enable_gpio_ready;
typedef enum {
    SPEAKER_REQUEST_BEEP = 0,
    SPEAKER_REQUEST_WELCOME = 1,
    SPEAKER_REQUEST_MENU1_INTRO = 2,
    SPEAKER_REQUEST_MENU2_INTRO = 3,
    SPEAKER_REQUEST_MENU4_INTRO = 4,
    SPEAKER_REQUEST_MENU4_CORRECT = 5,
    SPEAKER_REQUEST_MENU4_INCORRECT = 6,
    SPEAKER_REQUEST_MENU5_INTRO = 7,
    SPEAKER_REQUEST_MENU5_INCORRECT = 8,
    SPEAKER_REQUEST_MENU3_INTRO = 9,
    SPEAKER_REQUEST_MENU3_CORRECT = 10,
    SPEAKER_REQUEST_MENU3_INCORRECT = 11,
    SPEAKER_REQUEST_MENU1_LETTER_A = 16,
    SPEAKER_REQUEST_MENU2_VOWEL_A = 48,
    SPEAKER_REQUEST_MENU4_QUESTION_A_VS_I = 64,
    SPEAKER_REQUEST_MENU5_QUESTION_A = 80,
    SPEAKER_REQUEST_MENU5_FOUND_A = 96,
    SPEAKER_REQUEST_MENU3_PROMPT_A = 112
} speaker_request_t;
static QueueHandle_t speaker_beep_queue;
static bool welcome_playback_requested;
static QueueHandle_t mic_test_queue;
static QueueHandle_t phoneme_capture_queue;
static QueueHandle_t phoneme_result_queue;
static TaskHandle_t phoneme_capture_task_handle;
static portMUX_TYPE mic_test_state_lock = portMUX_INITIALIZER_UNLOCKED;
static bool mic_test_active;
static int32_t mic_rx_buffer[MIC_RX_BUFFER_WORDS];
static bool phoneme_busy;
typedef enum {
    PHONEME_RECORDING_IDLE,
    PHONEME_RECORDING_PROMPTING,
    PHONEME_RECORDING_ACTIVE,
    PHONEME_RECORDING_STOPPING,
    PHONEME_RECORDING_ANALYZING
} phoneme_recording_state_t;
static volatile phoneme_recording_state_t phoneme_recording_state;
static bool phoneme_vowel_passed[PHONEME_CLASS_COUNT];
static int phoneme_feedback_state;
static int phoneme_predicted_index = -1;
static float phoneme_confidence;
static float phoneme_features[PHONEME_FRAME_COUNT][PHONEME_FEATURE_COUNT];
typedef struct {
    uint8_t target_index;
    int8_t predicted_index;
    float confidence;
    bool valid;
} phoneme_result_event_t;
typedef struct {
    int x;
    int y;
    uint16_t raw_x;
    uint16_t raw_y;
} touch_event_t;
static QueueHandle_t ui_event_queue;
static int active_page = -1;
static bool dashboard_visible = true;
static bool lesson_started = false;
static int home_carousel = 0;
static int lesson_step[6] = {0};
static int vowel_quiz_feedback;
static int vowel_quiz_selected_option = -1;
static uint8_t review_clouds_remaining = 0x1F;
static uint32_t menu4_attempts[5];
static uint32_t menu5_attempts[5];

static void lcd_fill_rect(int x0, int y0, int x1, int y1, uint8_t r, uint8_t g, uint8_t b)
{
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 >= LCD_WIDTH) x1 = LCD_WIDTH - 1;
    if (y1 >= LCD_HEIGHT) y1 = LCD_HEIGHT - 1;
    if (x0 > x1 || y0 > y1) return;
    uint8_t col[4] = {x0 >> 8, x0 & 255, x1 >> 8, x1 & 255};
    uint8_t row[4] = {y0 >> 8, y0 & 255, y1 >> 8, y1 & 255};
    lcd_write_cmd(0x2A); lcd_write_data(col, 4);
    lcd_write_cmd(0x2B); lcd_write_data(row, 4);
    lcd_write_cmd(0x2C);
    uint8_t line[LCD_WIDTH * 3];
    int width = x1 - x0 + 1;
    for (int i = 0; i < width; ++i) {
        line[i * 3] = r & 0xFC;
        line[i * 3 + 1] = g & 0xFC;
        line[i * 3 + 2] = b & 0xFC;
    }
    for (int y = y0; y <= y1; ++y) lcd_write_data(line, (size_t)width * 3);
}
/* Compact 5x7 font keeps the UI independent of external font packages. */
static const uint8_t *glyph(char c)
{
    static const uint8_t blank[5] = {0, 0, 0, 0, 0};
#define G(ch,a,b,c,d,e) case ch: { static const uint8_t v[5] = {a,b,c,d,e}; return v; }
    if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
    switch (c) {
        G('A',0x7E,0x11,0x11,0x11,0x7E) G('B',0x7F,0x49,0x49,0x49,0x36) G('C',0x3E,0x41,0x41,0x41,0x22)
        G('D',0x7F,0x41,0x41,0x22,0x1C) G('E',0x7F,0x49,0x49,0x49,0x41) G('F',0x7F,0x09,0x09,0x09,0x01)
        G('G',0x3E,0x41,0x49,0x49,0x7A) G('H',0x7F,0x08,0x08,0x08,0x7F) G('I',0x00,0x41,0x7F,0x41,0x00)
        G('J',0x20,0x40,0x41,0x3F,0x01) G('K',0x7F,0x08,0x14,0x22,0x41) G('L',0x7F,0x40,0x40,0x40,0x40)
        G('M',0x7F,0x02,0x0C,0x02,0x7F) G('N',0x7F,0x04,0x08,0x10,0x7F) G('O',0x3E,0x41,0x41,0x41,0x3E)
        G('P',0x7F,0x09,0x09,0x09,0x06) G('Q',0x3E,0x41,0x51,0x21,0x5E) G('R',0x7F,0x09,0x19,0x29,0x46)
        G('S',0x46,0x49,0x49,0x49,0x31) G('T',0x01,0x01,0x7F,0x01,0x01) G('U',0x3F,0x40,0x40,0x40,0x3F)
        G('V',0x1F,0x20,0x40,0x20,0x1F) G('W',0x3F,0x40,0x38,0x40,0x3F) G('X',0x63,0x14,0x08,0x14,0x63)
        G('Y',0x07,0x08,0x70,0x08,0x07) G('Z',0x61,0x51,0x49,0x45,0x43)
        G('0',0x3E,0x51,0x49,0x45,0x3E) G('1',0x00,0x42,0x7F,0x40,0x00) G('2',0x42,0x61,0x51,0x49,0x46)
        G('3',0x21,0x41,0x45,0x4B,0x31) G('4',0x18,0x14,0x12,0x7F,0x10) G('5',0x27,0x45,0x45,0x45,0x39)
        G('6',0x3E,0x49,0x49,0x49,0x32) G('7',0x01,0x71,0x09,0x05,0x03) G('8',0x36,0x49,0x49,0x49,0x36)
        G('9',0x26,0x49,0x49,0x49,0x3E) G('-',0x08,0x08,0x08,0x08,0x08) G(':',0,0x36,0x36,0,0)
        G('<',0x08,0x14,0x22,0x41,0x00) G('>',0x41,0x22,0x14,0x08,0x00)
        G('.',0,0x60,0x60,0,0) G('!',0,0,0x5F,0,0) G(' ',0,0,0,0,0)
        default: return blank;
    }
#undef G
}

static const uint8_t *glyph_lowercase(char c)
{
    static const uint8_t blank[5] = {0, 0, 0, 0, 0};
#define G(ch,a,b,c,d,e) case ch: { static const uint8_t v[5] = {a,b,c,d,e}; return v; }
    switch (c) {
        G('a',0x20,0x54,0x54,0x54,0x78) G('b',0x7F,0x48,0x44,0x44,0x38)
        G('c',0x38,0x44,0x44,0x44,0x28) G('d',0x38,0x44,0x44,0x48,0x7F)
        G('e',0x38,0x54,0x54,0x54,0x18) G('f',0x08,0x7E,0x09,0x01,0x02)
        G('g',0x0C,0x52,0x52,0x52,0x3E) G('h',0x7F,0x08,0x04,0x04,0x78)
        G('i',0x00,0x44,0x7D,0x40,0x00) G('j',0x20,0x40,0x44,0x3D,0x00)
        G('k',0x7F,0x10,0x28,0x44,0x00) G('l',0x00,0x41,0x7F,0x40,0x00)
        G('m',0x7C,0x04,0x18,0x04,0x78) G('n',0x7C,0x08,0x04,0x04,0x78)
        G('o',0x38,0x44,0x44,0x44,0x38) G('p',0x7C,0x14,0x14,0x14,0x08)
        G('q',0x08,0x14,0x14,0x18,0x7C) G('r',0x7C,0x08,0x04,0x04,0x08)
        G('s',0x48,0x54,0x54,0x54,0x20) G('t',0x04,0x3F,0x44,0x40,0x20)
        G('u',0x3C,0x40,0x40,0x20,0x7C) G('v',0x1C,0x20,0x40,0x20,0x1C)
        G('w',0x3C,0x40,0x30,0x40,0x3C) G('x',0x44,0x28,0x10,0x28,0x44)
        G('y',0x0C,0x50,0x50,0x50,0x3C) G('z',0x44,0x64,0x54,0x4C,0x44)
        default: return blank;
    }
#undef G
}

static void lcd_text(int x, int y, const char *text, int scale, uint8_t r, uint8_t g, uint8_t b)
{
    for (; *text; ++text, x += 6 * scale) {
        const uint8_t *columns = glyph(*text);
        for (int cx = 0; cx < 5; ++cx) {
            for (int cy = 0; cy < 7; ++cy) {
                if (columns[cx] & (1 << cy)) {
                    lcd_fill_rect(x + cx * scale, y + cy * scale,
                                  x + (cx + 1) * scale - 1, y + (cy + 1) * scale - 1,
                                  r, g, b);
                }
            }
        }
    }
}

static void lcd_text_mixed_case(int x, int y, const char *text, int scale,
                                uint8_t r, uint8_t g, uint8_t b)
{
    for (; *text; ++text, x += 6 * scale) {
        const uint8_t *columns = (*text >= 'a' && *text <= 'z')
            ? glyph_lowercase(*text) : glyph(*text);
        for (int cx = 0; cx < 5; ++cx) {
            for (int cy = 0; cy < 7; ++cy) {
                if (columns[cx] & (1 << cy)) {
                    lcd_fill_rect(x + cx * scale, y + cy * scale,
                                  x + (cx + 1) * scale - 1,
                                  y + (cy + 1) * scale - 1, r, g, b);
                }
            }
        }
    }
}

static void draw_startup_splash(void)
{
    static const char title[] = "Readzio";
    const int title_scale = 5;
    const int title_x = (LCD_WIDTH - (int)(sizeof(title) - 1) * 6 * title_scale) / 2;
    const uint8_t navy_r = 8, navy_g = 18, navy_b = 45;
    const uint8_t cyan_r = 75, cyan_g = 220, cyan_b = 255;

    lcd_fill(navy_r, navy_g, navy_b);
    for (size_t i = 0; i < sizeof(title) - 1; ++i) {
        lcd_text(title_x + (int)i * 6 * title_scale, 92,
                 (char[]){title[i], '\0'}, title_scale,
                 cyan_r, cyan_g, cyan_b);
        vTaskDelay(pdMS_TO_TICKS(80));
    }

    lcd_text((LCD_WIDTH - 14 * 6 * 2) / 2, 158,
             "by Team Adivos", 2, 245, 250, 255);

    lcd_fill_rect(88, 218, 392, 240, 245, 250, 255);
    lcd_fill_rect(92, 222, 388, 236, navy_r, navy_g, navy_b);
    for (int progress = 0; progress <= 12; ++progress) {
        int fill_end = 94 + progress * 24;
        if (progress > 0) {
            lcd_fill_rect(94 + (progress - 1) * 24, 224,
                          fill_end - 1, 234, cyan_r, cyan_g, cyan_b);
        }
        vTaskDelay(pdMS_TO_TICKS(80));
    }
}

static void lcd_line(int x0, int y0, int x1, int y1, uint8_t r, uint8_t g, uint8_t b)
{
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int error = dx + dy;
    while (1) {
        lcd_fill_rect(x0, y0, x0 + 1, y0 + 1, r, g, b);
        if (x0 == x1 && y0 == y1) break;
        int doubled_error = 2 * error;
        if (doubled_error >= dy) { error += dy; x0 += sx; }
        if (doubled_error <= dx) { error += dx; y0 += sy; }
    }
}

static const char *menu_titles[6] = {
    "MENGENAL HURUF", "BELAJAR AIUEO", "LATIHAN FONEM",
    "LATIHAN BENTUK", "REVIEW", "BELAJAR MBP"
};
static const uint8_t card_colors[6][3] = {
    {255, 216, 115}, {122, 207, 191}, {250, 159, 128},
    {137, 184, 232}, {224, 170, 207}, {174, 205, 124}
};

static void draw_mascot(void);
static void draw_book_icon(int center_x, int y, uint8_t r, uint8_t g, uint8_t b);
static void draw_menu_card(int menu_index, int x, int y);

static void draw_header(const char *title, bool show_back)
{
    (void)title;
    lcd_fill_rect(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1, 255, 246, 234);
    if (show_back) {
        lcd_fill_rect(20, 18, 72, 61, 55, 132, 250);
        lcd_line(55, 39, 31, 39, 12, 20, 38);
        lcd_line(31, 39, 47, 24, 12, 20, 38);
        lcd_line(31, 39, 47, 54, 12, 20, 38);
    }
    lcd_fill_rect(415, 20, 468, 62, 220, 220, 220);
    lcd_fill_rect(428, 34, 437, 48, 20, 20, 20);
    lcd_line(437, 34, 450, 25, 20, 20, 20);
    lcd_line(437, 48, 450, 57, 20, 20, 20);
    lcd_line(450, 25, 450, 57, 20, 20, 20);
    lcd_line(455, 33, 461, 41, 20, 20, 20);
    lcd_line(461, 41, 455, 49, 20, 20, 20);
}

static void draw_home(void)
{
    lcd_fill(255, 246, 234);
    int first_menu = home_carousel * 2;
    draw_menu_card(first_menu, 94, 56);
    draw_menu_card(first_menu + 1, 267, 56);
    if (home_carousel > 0) lcd_text(34, 130, "<", 5, 20, 20, 20);
    if (home_carousel < 2) lcd_text(435, 130, ">", 5, 20, 20, 20);
    lcd_fill_rect(339, 270, 466, 310, 91, 197, 183);
    lcd_line(352, 290, 363, 280, 255, 255, 255);
    lcd_line(363, 280, 374, 290, 255, 255, 255);
    lcd_fill_rect(355, 289, 371, 301, 255, 255, 255);
    lcd_fill_rect(361, 294, 365, 301, 91, 197, 183);
    lcd_text(380, 288, "DASHBOARD", 1, 255, 255, 255);
    draw_mascot();
}

static void draw_circle(int center_x, int center_y, int radius, uint8_t r, uint8_t g, uint8_t b)
{
    for (int y = -radius; y <= radius; ++y) {
        int half_width = 0;
        while ((half_width + 1) * (half_width + 1) + y * y <= radius * radius) ++half_width;
        lcd_fill_rect(center_x - half_width, center_y + y, center_x + half_width, center_y + y, r, g, b);
    }
}

static void draw_dashboard(void)
{
    static const uint8_t vowel_colors[5][3] = {
        {239, 88, 91}, {65, 137, 232}, {73, 177, 126},
        {249, 168, 62}, {153, 105, 211}
    };
    lcd_fill(244, 248, 255);

    draw_circle(26, 34, 15, 91, 197, 183);
    lcd_fill_rect(18, 29, 25, 39, 255, 255, 255);
    lcd_fill_rect(27, 29, 34, 39, 255, 255, 255);
    lcd_fill_rect(26, 29, 26, 40, 255, 231, 178);
    lcd_text_mixed_case(53, 20, "Readzio", 3, 34, 89, 103);
    lcd_text(354, 28, "BELAJAR SERU", 1, 84, 117, 129);

    draw_circle(26, 80, 11, 255, 215, 117);
    draw_circle(453, 83, 14, 155, 218, 239);
    draw_circle(438, 218, 8, 255, 183, 149);

    lcd_fill_rect(37, 61, 443, 225, 219, 229, 239);
    lcd_fill_rect(33, 57, 439, 221, 255, 255, 255);
    lcd_fill_rect(33, 57, 439, 65, 91, 197, 183);
    lcd_text(181, 78, "SELAMAT DATANG", 1, 84, 117, 129);
    lcd_text_mixed_case(78, 101, "Belajar Jadi Seru!", 3, 34, 89, 103);
    lcd_text(138, 143, "Kenali huruf dan temukan hal baru", 1,
             103, 128, 139);

    draw_book_icon(76, 155, 249, 192, 112);
    for (int i = 0; i < 5; ++i) {
        int center_x = 135 + i * 52;
        draw_circle(center_x, 195, 14, vowel_colors[i][0],
                    vowel_colors[i][1], vowel_colors[i][2]);
        lcd_text(center_x - 3, 191, (char[]){ "AIUEO"[i], '\0' }, 1,
                 255, 255, 255);
    }

    lcd_fill_rect(101, 247, 379, 298, 30, 114, 104);
    lcd_fill_rect(96, 242, 374, 293, 45, 155, 132);
    lcd_text(187, 260, "KLIK UNTUK MULAI", 1, 255, 255, 255);
    lcd_line(337, 266, 352, 266, 255, 255, 255);
    lcd_line(346, 259, 353, 266, 255, 255, 255);
    lcd_line(353, 266, 346, 273, 255, 255, 255);
    lcd_text(139, 304, "PETUALANGAN HURUF MENUNGGU!", 1,
             84, 117, 129);
}

static void draw_mascot(void)
{
    lcd_fill_rect(36, 245, 76, 251, 101, 47, 30);
    lcd_fill_rect(43, 238, 70, 246, 173, 73, 43);
    lcd_fill_rect(32, 248, 78, 252, 219, 49, 45);
    draw_circle(55, 270, 21, 61, 177, 220);
    draw_circle(55, 252, 14, 90, 196, 232);
    lcd_fill_rect(34, 266, 76, 278, 61, 177, 220);
    draw_circle(47, 251, 5, 255, 255, 255);
    draw_circle(63, 251, 5, 255, 255, 255);
    draw_circle(48, 251, 2, 24, 35, 43);
    draw_circle(64, 251, 2, 24, 35, 43);
    lcd_fill_rect(52, 257, 59, 260, 246, 178, 63);
    lcd_fill_rect(39, 286, 49, 291, 61, 177, 220);
    lcd_fill_rect(62, 286, 72, 291, 61, 177, 220);
}

static void draw_book_icon(int center_x, int y, uint8_t r, uint8_t g, uint8_t b)
{
    lcd_fill_rect(center_x - 39, y + 12, center_x - 2, y + 37, r, g, b);
    lcd_fill_rect(center_x + 2, y + 12, center_x + 39, y + 37, r, g, b);
    lcd_fill_rect(center_x - 36, y + 16, center_x - 3, y + 33, 255, 250, 226);
    lcd_fill_rect(center_x + 3, y + 16, center_x + 36, y + 33, 255, 250, 226);
    lcd_fill_rect(center_x - 2, y + 10, center_x + 2, y + 40, 101, 47, 30);
}

static void draw_menu_card(int menu_index, int x, int y)
{
    static const char *labels[6] = {
        "MENGENAL HURUF", "BELAJAR AIUEO", "LATIHAN FONEM",
        "LATIHAN BENTUK", "REVIEW", "BELAJAR MBP"
    };
    static const uint8_t colors[5][3] = {
        {238, 58, 65}, {50, 128, 246}, {68, 173, 82},
        {255, 185, 35}, {144, 73, 211}
    };

    lcd_fill_rect(x, y, x + 138, y + 198, 221, 221, 221);
    lcd_fill_rect(x + 2, y + 2, x + 136, y + 196, 255, 255, 255);
    lcd_fill_rect(x + 13, y + 15, x + 125, y + 133, 255, 251, 242);
    draw_circle(x + 12, y + 9, 15, 218, 218, 218);
    char number[2] = {(char)('1' + menu_index), '\0'};
    lcd_text(x + 7, y + 1, number, 2, 15, 15, 15);

    if (menu_index == 0) {
        lcd_text(x + 24, y + 38, "A", 4, colors[0][0], colors[0][1], colors[0][2]);
        lcd_text(x + 57, y + 30, "B", 4, colors[1][0], colors[1][1], colors[1][2]);
        lcd_text(x + 90, y + 38, "C", 4, colors[3][0], colors[3][1], colors[3][2]);
        lcd_text(x + 40, y + 79, "D", 4, colors[2][0], colors[2][1], colors[2][2]);
        lcd_text(x + 76, y + 79, "E", 4, colors[4][0], colors[4][1], colors[4][2]);
    } else if (menu_index == 1) {
        const char *vowels = "AIUEO";
        for (int i = 0; i < 5; ++i) {
            lcd_text(x + 15 + i * 20, y + 35 + (i % 2) * 4,
                     (char[]){vowels[i], '\0'}, 2,
                     colors[i][0], colors[i][1], colors[i][2]);
        }
        draw_book_icon(x + 69, y + 76, 177, 127, 67);
    } else if (menu_index == 2) {
        draw_circle(x + 59, y + 72, 37, 255, 205, 215);
        draw_circle(x + 59, y + 72, 30, 255, 238, 242);
        lcd_text(x + 47, y + 54, "A", 5, 221, 78, 111);
        lcd_line(x + 99, y + 54, x + 106, y + 62, 69, 153, 214);
        lcd_line(x + 106, y + 62, x + 99, y + 70, 69, 153, 214);
        lcd_line(x + 106, y + 49, x + 116, y + 62, 69, 153, 214);
        lcd_line(x + 116, y + 62, x + 106, y + 75, 69, 153, 214);
        lcd_line(x + 113, y + 44, x + 126, y + 62, 69, 153, 214);
        lcd_line(x + 126, y + 62, x + 113, y + 80, 69, 153, 214);
    } else if (menu_index == 3) {
        lcd_fill_rect(x + 28, y + 48, x + 68, y + 91, 255, 220, 181);
        lcd_fill_rect(x + 75, y + 48, x + 115, y + 91, 193, 224, 255);
        lcd_text(x + 39, y + 55, "A", 4, 221, 78, 111);
        lcd_text(x + 86, y + 55, "O", 4, 49, 117, 184);
    } else if (menu_index == 4) {
        for (int i = 0; i < 5; ++i) {
            draw_circle(x + 25 + i * 22, y + 65, 13,
                        colors[i][0], colors[i][1], colors[i][2]);
            lcd_text(x + 19 + i * 22, y + 58,
                     (char[]){"AIUEO"[i], '\0'}, 2, 255, 255, 255);
        }
    } else {
        lcd_text(x + 25, y + 35, "MBP", 4, 239, 61, 69);
        draw_book_icon(x + 69, y + 77, 177, 127, 67);
    }

    lcd_fill_rect(x + 4, y + 145, x + 134, y + 169, 218, 218, 218);
    lcd_text(x + 7, y + 153, labels[menu_index], 1, 20, 20, 20);
}

static void draw_step_buttons(const char *left_label, const char *right_label)
{
    lcd_fill_rect(226, 230, 337, 276, 227, 237, 225);
    lcd_fill_rect(350, 230, 468, 276, 27, 105, 96);
    lcd_text(244, 248, left_label, 1, 41, 73, 69);
    lcd_text(376, 248, right_label, 1, 255, 255, 244);
}

static void draw_module_menu(void)
{
    static const char *prompts[6] = {
        "KENALI HURUF A SAMPAI Z", "LATIH LIMA HURUF VOKAL",
        "UCAPKAN BUNYI HURUF", "PILIH HURUF SESUAI BUNYI",
        "ULANGI MATERI YANG SUDAH ADA", "GABUNGKAN SUKU KATA"
    };
    char number[2] = {(char)('1' + active_page), '\0'};

    lcd_fill(255, 246, 234);
    draw_header(menu_titles[active_page], true);
    lcd_fill_rect(57, 80, 423, 211, 255, 255, 255);
    lcd_fill_rect(73, 96, 186, 194,
                  card_colors[active_page][0], card_colors[active_page][1],
                  card_colors[active_page][2]);
    lcd_text(111, 119, number, 7, 20, 20, 20);
    lcd_text(205, 108, "MULAI BELAJAR", 2, 20, 20, 20);
    lcd_text(205, 147, prompts[active_page], 1, 84, 101, 95);
    draw_step_buttons("KEMBALI", "MULAI");
    draw_mascot();
}

static void draw_microphone_icon(int center_x, int center_y)
{
    draw_circle(center_x, center_y, 27, 49, 174, 94);
    lcd_fill_rect(center_x - 5, center_y - 17, center_x + 5, center_y + 9, 20, 54, 36);
    lcd_fill_rect(center_x - 9, center_y - 9, center_x - 5, center_y + 3, 20, 54, 36);
    lcd_fill_rect(center_x + 5, center_y - 9, center_x + 9, center_y + 3, 20, 54, 36);
    lcd_fill_rect(center_x - 12, center_y + 1, center_x - 8, center_y + 9, 20, 54, 36);
    lcd_fill_rect(center_x + 8, center_y + 1, center_x + 12, center_y + 9, 20, 54, 36);
    lcd_fill_rect(center_x - 12, center_y + 7, center_x + 12, center_y + 11, 20, 54, 36);
    lcd_fill_rect(center_x - 2, center_y + 11, center_x + 2, center_y + 19, 20, 54, 36);
    lcd_fill_rect(center_x - 8, center_y + 18, center_x + 8, center_y + 21, 20, 54, 36);
}

static void draw_bottom_nav(bool microphone)
{
    lcd_line(115, 278, 85, 296, 20, 20, 20);
    lcd_line(85, 296, 115, 314, 20, 20, 20);
    lcd_line(365, 278, 395, 296, 20, 20, 20);
    lcd_line(395, 296, 365, 314, 20, 20, 20);
    if (microphone) draw_microphone_icon(240, 292);
    else {
        lcd_fill_rect(222, 286, 231, 302, 22, 22, 22);
        lcd_line(231, 286, 245, 276, 22, 22, 22);
        lcd_line(231, 302, 245, 312, 22, 22, 22);
        lcd_line(245, 276, 245, 312, 22, 22, 22);
        lcd_line(250, 284, 257, 294, 22, 22, 22);
        lcd_line(257, 294, 250, 304, 22, 22, 22);
    }
}

static void draw_phoneme_confirm_button(void)
{
    draw_microphone_icon(240, 292);
}

static void draw_phoneme_star(int center_x, int center_y, bool earned)
{
    static const int star_x[10] = {0, 2, 8, 3, 5, 0, -5, -3, -8, -2};
    static const int star_y[10] = {-8, -2, -2, 2, 8, 4, 8, 2, -2, -2};
    uint8_t fill = earned ? 255 : 225;
    uint8_t star = earned ? 191 : 190;
    draw_circle(center_x, center_y, 13, 190, 190, 190);
    draw_circle(center_x, center_y, 10, fill, earned ? 207 : 225, earned ? 83 : 225);
    if (earned) {
        for (int point = 0; point < 10; ++point) {
            int next = (point + 1) % 10;
            lcd_line(center_x + star_x[point], center_y + star_y[point],
                     center_x + star_x[next], center_y + star_y[next],
                     star, 119, 35);
        }
    }
}

static void draw_vowel_option(int option, char letter, bool correct_option)
{
    static const int centers[2] = {150, 330};
    static const uint8_t card_colors[5][3] = {
        {255, 205, 214}, {190, 222, 255}, {190, 235, 203},
        {255, 220, 168}, {255, 235, 173}
    };
    int x = centers[option] - 55;
    int y = 145;
    bool selected = option == vowel_quiz_selected_option;
    uint8_t border_r = 218, border_g = 211, border_b = 203;
    if (vowel_quiz_feedback == 1 && correct_option) {
        border_r = 66; border_g = 176; border_b = 106;
    } else if (selected && vowel_quiz_feedback == 1) {
        border_r = 235; border_g = 112; border_b = 112;
    } else if (selected && vowel_quiz_feedback == 2) {
        border_r = 66; border_g = 176; border_b = 106;
    }

    lcd_fill_rect(x - 2, y - 2, x + 112, y + 78,
                  border_r, border_g, border_b);
    int color_index = letter - 'A';
    lcd_fill_rect(x, y, x + 108, y + 74,
                  card_colors[color_index][0], card_colors[color_index][1],
                  card_colors[color_index][2]);
    lcd_text(centers[option] - 30, y + 2, (char[]){letter, '\0'}, 10,
             35, 48, 68);
}

static void draw_review_cloud(int center_x, int center_y, char letter,
                              int color_index)
{
    static const uint8_t cloud_colors[5][3] = {
        {255, 193, 92}, {83, 176, 231}, {244, 119, 151},
        {93, 190, 120}, {155, 119, 225}
    };
    uint8_t r = cloud_colors[color_index][0];
    uint8_t g = cloud_colors[color_index][1];
    uint8_t b = cloud_colors[color_index][2];
    draw_circle(center_x - 17, center_y + 4, 20, 193, 212, 227);
    draw_circle(center_x, center_y - 5, 26, 193, 212, 227);
    draw_circle(center_x + 17, center_y + 4, 20, 193, 212, 227);
    lcd_fill_rect(center_x - 37, center_y, center_x + 37, center_y + 23,
                  193, 212, 227);
    draw_circle(center_x - 17, center_y, 20, r, g, b);
    draw_circle(center_x, center_y - 9, 26, r, g, b);
    draw_circle(center_x + 17, center_y, 20, r, g, b);
    lcd_fill_rect(center_x - 37, center_y - 4, center_x + 37, center_y + 19,
                  r, g, b);
    draw_circle(center_x - 15, center_y - 3, 15, 255, 255, 255);
    draw_circle(center_x, center_y - 12, 20, 255, 255, 255);
    draw_circle(center_x + 15, center_y - 3, 15, 255, 255, 255);
    lcd_fill_rect(center_x - 29, center_y - 4, center_x + 29, center_y + 14,
                  255, 255, 255);
    lcd_text(center_x - 12, center_y - 19, (char[]){letter, '\0'}, 5,
             43, 56, 76);
}

static void draw_lesson_page(void)
{
    static const char vowels[] = "AIUEO";
    int step = lesson_step[active_page];
    lcd_fill(255, 246, 234);
    draw_header(menu_titles[active_page], true);

    if (active_page == 0) {
        static const uint8_t card_colors[6][3] = {
            {255, 226, 158}, {186, 229, 255}, {255, 198, 211},
            {198, 237, 195}, {222, 205, 255}, {255, 215, 174}
        };
        static const uint8_t accent_colors[6][3] = {
            {245, 139, 61}, {59, 153, 210}, {224, 86, 117},
            {79, 165, 102}, {140, 102, 207}, {231, 147, 58}
        };
        char pair[3] = {(char)('A' + step % 26), (char)('a' + step % 26), '\0'};
        int color_index = step % 6;
        lcd_text(157, 39, "ALFABET", 4, 15, 15, 15);
        lcd_fill_rect(35, 88, 445, 252, 255, 255, 255);
        lcd_fill_rect(54, 105, 426, 235, accent_colors[color_index][0],
                      accent_colors[color_index][1], accent_colors[color_index][2]);
        lcd_fill_rect(59, 110, 421, 230, card_colors[color_index][0],
                      card_colors[color_index][1], card_colors[color_index][2]);
        draw_circle(88, 137, 8, 255, 255, 255);
        draw_circle(392, 137, 8, 255, 255, 255);
        draw_circle(88, 203, 6, 255, 255, 255);
        draw_circle(392, 203, 6, 255, 255, 255);
        lcd_text(184, 132, (char[]){pair[0], '\0'}, 10, 65, 78, 102);
        lcd_text_mixed_case(264, 150, pair + 1, 7, 65, 78, 102);
        draw_bottom_nav(false);
    } else if (active_page == 1) {
        static const uint8_t vowel_colors[5][3] = {
            {35, 124, 224}, {239, 55, 65}, {48, 173, 82},
            {255, 142, 25}, {126, 66, 211}
        };
        int vowel_index = step % 5;
        uint8_t r = vowel_colors[vowel_index][0];
        uint8_t g = vowel_colors[vowel_index][1];
        uint8_t b = vowel_colors[vowel_index][2];
        char progress[4] = {
            (char)('1' + vowel_index), '/', '5', '\0'
        };

        lcd_fill_rect(48, 76, 432, 266, 243, 248, 255);
        lcd_text(190, 86, "HURUF VOKAL", 1, 65, 78, 102);
        lcd_fill_rect(124, 111, 364, 237, 218, 211, 203);
        lcd_fill_rect(120, 107, 360, 233, 255, 255, 255);
        lcd_fill_rect(120, 107, 360, 116, r, g, b);
        lcd_text(200, 118, (char[]){vowels[vowel_index], '\0'},
                 16, r, g, b);
        lcd_text(230, 254, progress, 1, 65, 78, 102);

        lcd_line(115, 278, 85, 296, 20, 20, 20);
        lcd_line(85, 296, 115, 314, 20, 20, 20);
        lcd_line(365, 278, 395, 296, 20, 20, 20);
        lcd_line(395, 296, 365, 314, 20, 20, 20);
    } else if (active_page == 2) {
        static const char phonemes[] = "AIUEO";
        int phoneme_index = step % PHONEME_CLASS_COUNT;
        char letter[2] = {phonemes[phoneme_index], '\0'};
        lcd_fill_rect(48, 76, 432, 270, 255, 255, 255);
        for (int i = 0; i < 5; ++i)
            draw_phoneme_star(177 + i * 32, 97,
                              phoneme_vowel_passed[i]);
        lcd_text(178, 78, "UCAPKAN BUNYI HURUF", 1, 65, 78, 102);
        lcd_text(178, 122, letter, 11, 0, 0, 0);
        if (phoneme_recording_state == PHONEME_RECORDING_PROMPTING) {
            lcd_text(177, 218, "DENGARKAN CONTOH SUARA", 1, 65, 78, 102);
        } else if (phoneme_recording_state == PHONEME_RECORDING_ACTIVE) {
            lcd_text(177, 218, "KAMI MENDENGARKAN", 1, 65, 78, 102);
            lcd_text(196, 239, "KETUK UNTUK SELESAI", 1, 190, 84, 65);
        } else if (phoneme_recording_state == PHONEME_RECORDING_STOPPING ||
                   phoneme_recording_state == PHONEME_RECORDING_ANALYZING) {
            lcd_text(190, 218, "MENGANALISIS SUARA", 1, 65, 78, 102);
        } else if (phoneme_feedback_state == 1) {
            lcd_text(177, 218, "BENAR! COBA HURUF LAIN", 1, 56, 139, 84);
        } else if (phoneme_feedback_state == 2 &&
                   phoneme_predicted_index >= 0) {
            static const char classes[] = "AEIOU";
            char guess[16] = "TEBAKAN: ";
            size_t length = strlen(guess);
            guess[length++] = classes[phoneme_predicted_index];
            guess[length] = '\0';
            lcd_text(177, 218, guess, 1, 190, 84, 65);
        } else if (phoneme_feedback_state == 3) {
            lcd_text(177, 218, "SUARA BELUM TERDETEKSI", 1, 190, 84, 65);
        } else {
            lcd_text(177, 218, "KETUK MULAI REKAM", 1, 65, 78, 102);
        }
        if ((phoneme_feedback_state == 1 || phoneme_feedback_state == 2) &&
            phoneme_recording_state == PHONEME_RECORDING_IDLE) {
            char confidence_label[20];
            snprintf(confidence_label, sizeof(confidence_label),
                     "AKURASI = %d%%",
                     (int)(phoneme_confidence * 100.0f + 0.5f));
            lcd_text(194, 239, confidence_label, 1, 65, 78, 102);
        }
        draw_mascot();
        draw_bottom_nav(false);
        draw_phoneme_confirm_button();
        lcd_text(204, 263,
                 phoneme_recording_state == PHONEME_RECORDING_ACTIVE
                     ? "STOP REKAM"
                     : phoneme_recording_state == PHONEME_RECORDING_IDLE
                           ? "MULAI REKAM" : "TUNGGU",
                 1, 65, 78, 102);
    } else if (active_page == 3) {
        static const char vowels[] = "AIUEO";
        const int target_index = step % 5;
        const int correct_option = step % 2;
        const char target = vowels[target_index];
        const char distractor = vowels[(target_index + 1) % 5];
        char progress[4] = {
            (char)('1' + target_index), '/', '5', '\0'
        };
        lcd_fill_rect(48, 76, 432, 266, 243, 248, 255);
        lcd_text(159, 90, "PILIH HURUF SESUAI BUNYI", 1, 65, 78, 102);
        draw_vowel_option(0, correct_option == 0 ? target : distractor,
                          correct_option == 0);
        draw_vowel_option(1, correct_option == 1 ? target : distractor,
                          correct_option == 1);
        lcd_text(235, 229, progress, 1, 65, 78, 102);
        if (vowel_quiz_feedback == 1) {
            lcd_text(183, 244, "LIHAT KARTU HIJAU", 1, 56, 139, 84);
        } else if (vowel_quiz_feedback == 2) {
            lcd_text(204, 244, "HEBAT! BENAR", 1, 56, 139, 84);
            lcd_fill_rect(168, 274, 312, 311, 56, 161, 101);
            lcd_text(210, 287, "LANJUT", 1, 255, 255, 255);
        }
    } else if (active_page == 4) {
        static const int cloud_centers_x[5] = {120, 240, 360, 180, 300};
        static const int cloud_centers_y[5] = {166, 166, 166, 226, 226};
        int remaining = 0;
        lcd_fill_rect(48, 76, 432, 266, 255, 255, 255);
        lcd_text(180, 87, "AWAN VOKAL", 2, 65, 78, 102);
        lcd_text(195, 112, "SENTUH HURUFNYA", 1, 111, 126, 145);
        for (int i = 0; i < 5; ++i) {
            if (review_clouds_remaining & (1U << i)) {
                draw_review_cloud(cloud_centers_x[i], cloud_centers_y[i],
                                  vowels[i], i);
                ++remaining;
            }
        }
        lcd_fill_rect(165, 257, 315, 263, 220, 228, 235);
        if (remaining > 0) {
            int progress_width = (5 - remaining) * 30;
            if (progress_width > 0)
                lcd_fill_rect(165, 257, 165 + progress_width, 263,
                              93, 190, 120);
        } else {
            lcd_text(198, 237, "SELESAI! HEBAT", 1, 56, 139, 84);
        }
    } else {
        lcd_fill_rect(58, 76, 422, 269, 255, 255, 255);
        lcd_text(171, 92, "MBP", 7, 239, 55, 65);
        draw_book_icon(240, 171, 121, 79, 47);
        lcd_text(166, 226, "BA + JU", 2, 29, 82, 196);
        draw_bottom_nav(false);
    }
}

static void render_ui(void)
{
    /* LCD commands/data are multiple SPI transactions; keep them atomic relative
       to XPT2046 reads on the same SPI2 bus. */
    if (spi_shared_bus_mutex != NULL) {
        xSemaphoreTake(spi_shared_bus_mutex, portMAX_DELAY);
    }
    if (active_page < 0) {
        if (dashboard_visible) draw_dashboard();
        else draw_home();
    }
    else if (!lesson_started) draw_module_menu();
    else draw_lesson_page();
    if (spi_shared_bus_mutex != NULL) {
        xSemaphoreGive(spi_shared_bus_mutex);
    }
}

static void touch_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << TOUCH_PIN_IRQ,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    spi_device_interface_config_t device = {
        .clock_speed_hz = 2000000,
        .mode = 0,
        .spics_io_num = TOUCH_PIN_CS,
        .queue_size = 1
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &device, &touch_spi));
    ESP_LOGI(TAG, "Touch XPT2046 siap untuk navigasi UI.");
}

static uint16_t touch_read_axis(uint8_t command)
{
    uint8_t tx[3] = {command, 0, 0};
    uint8_t rx[3] = {0};
    spi_transaction_t transaction = {
        .length = 24,
        .tx_buffer = tx,
        .rx_buffer = rx
    };
    if (spi_device_polling_transmit(touch_spi, &transaction) != ESP_OK) return 0;
    return (uint16_t)(((rx[1] << 8) | rx[2]) >> 3) & 0x0FFF;
}

static int touch_map_axis(uint16_t raw, uint16_t raw_min, uint16_t raw_max,
                          int output_max, bool invert)
{
    if (raw_max <= raw_min) return 0;
    int value = raw;
    if (value < raw_min) value = raw_min;
    if (value > raw_max) value = raw_max;
    value = (value - raw_min) * output_max / (raw_max - raw_min);
    return invert ? output_max - value : value;
}

static bool touch_read_point(int *x, int *y, uint16_t *raw_x_out,
                             uint16_t *raw_y_out)
{
    /* LCD and XPT2046 share SPI2. Do both axis reads under the same mutex so
       LCD command/data traffic cannot corrupt a touch conversion. */
    if (spi_shared_bus_mutex != NULL) {
        xSemaphoreTake(spi_shared_bus_mutex, portMAX_DELAY);
    }
    uint16_t raw_x = touch_read_axis(0xD0);
    uint16_t raw_y = touch_read_axis(0x90);
    if (spi_shared_bus_mutex != NULL) {
        xSemaphoreGive(spi_shared_bus_mutex);
    }
    *raw_x_out = raw_x;
    *raw_y_out = raw_y;
    if (raw_x == 0 || raw_x >= 4095 || raw_y == 0 || raw_y >= 4095) return false;
#if TOUCH_SWAP_XY
    *x = touch_map_axis(raw_y, TOUCH_RAW_Y_MIN, TOUCH_RAW_Y_MAX,
                        LCD_WIDTH - 1, TOUCH_INVERT_X);
    *y = touch_map_axis(raw_x, TOUCH_RAW_X_MIN, TOUCH_RAW_X_MAX,
                        LCD_HEIGHT - 1, TOUCH_INVERT_Y);
#else
    *x = touch_map_axis(raw_x, TOUCH_RAW_X_MIN, TOUCH_RAW_X_MAX,
                        LCD_WIDTH - 1, TOUCH_INVERT_X);
    *y = touch_map_axis(raw_y, TOUCH_RAW_Y_MIN, TOUCH_RAW_Y_MAX,
                        LCD_HEIGHT - 1, TOUCH_INVERT_Y);
#endif
    return true;
}

static void request_touch_beep(void);
static void request_menu1_intro(void);
static void request_menu1_letter(int letter_index);
static void request_menu2_intro(void);
static void request_menu2_vowel(int vowel_index);
static void request_menu3_intro(void);
static bool request_menu3_attempt(int vowel_index);
static bool request_menu3_stop(void);
static void request_menu4_intro(void);
static void request_menu4_question(int question_index);
static void request_menu4_feedback(bool correct);
static void request_menu5_intro(void);
static void request_menu5_question(int vowel_index);
static void request_menu5_found(int vowel_index, bool is_final_vowel);
static void request_menu5_incorrect(void);
static void process_phoneme_result(void);

static bool handle_touch(int x, int y)
{
    bool handled = false;
    bool changed = false;
    bool menu1_intro_requested = false;
    bool menu1_audio_changed = false;
    bool menu2_intro_requested = false;
    bool menu2_audio_changed = false;
    bool menu3_intro_requested = false;
    bool menu3_attempt_requested = false;
    int menu3_attempt_vowel = -1;
    bool menu4_intro_requested = false;
    bool menu4_question_requested = false;
    int menu4_feedback_request = 0;
    bool menu5_intro_requested = false;
    bool menu5_question_requested = false;
    int menu5_cloud_found = -1;
    bool menu5_is_final_vowel = false;
    bool menu5_incorrect_requested = false;
    if (active_page < 0) {
        if (dashboard_visible) {
            if (x >= 96 && x <= 374 && y >= 242 && y <= 293) {
                dashboard_visible = false;
                handled = true;
                changed = true;
            }
        } else if (x >= 339 && x <= 466 && y >= 270 && y <= 310) {
            dashboard_visible = true;
            handled = true;
            changed = true;
        } else if (x >= 422 && y >= 112 && y <= 192 && home_carousel < 2) {
            ++home_carousel;
            handled = true;
            changed = true;
        } else if (x >= 25 && x <= 82 && y >= 112 && y <= 192 && home_carousel > 0) {
            --home_carousel;
            handled = true;
            changed = true;
        } else if (y >= 56 && y <= 254) {
            int card_index = -1;
            if (x >= 94 && x <= 232) card_index = 0;
            else if (x >= 267 && x <= 405) card_index = 1;
            if (card_index >= 0) {
                active_page = home_carousel * 2 + card_index;
                lesson_started = false;
                if (active_page == 0) menu1_intro_requested = true;
                if (active_page == 1) menu2_intro_requested = true;
                if (active_page == 2) menu3_intro_requested = true;
                if (active_page == 3) menu4_intro_requested = true;
                if (active_page == 4) menu5_intro_requested = true;
                ESP_LOGI(TAG, "UI menu id=%d dipilih; screen tujuan=%s.",
                         active_page + 1, menu_titles[active_page]);
                handled = true;
                changed = true;
            }
        }
    } else if (!lesson_started) {
        if (((x >= 20 && x <= 72) || (x >= 415 && x <= 468)) &&
            y >= 18 && y <= 61) {
            active_page = -1;
            dashboard_visible = false;
            handled = true;
            changed = true;
        } else if (y >= 230 && y <= 276 && x >= 226 && x <= 337) {
            active_page = -1;
            dashboard_visible = false;
            handled = true;
            changed = true;
        } else if (y >= 230 && y <= 276 && x >= 350 && x <= 468) {
            lesson_started = true;
            lesson_step[active_page] = 0;
            if (active_page == 2) {
                memset(phoneme_vowel_passed, 0, sizeof(phoneme_vowel_passed));
                phoneme_feedback_state = 0;
                phoneme_predicted_index = -1;
            }
            if (active_page == 0) menu1_audio_changed = true;
            if (active_page == 1) menu2_audio_changed = true;
            if (active_page == 4) {
                review_clouds_remaining = 0x1F;
                memset(menu5_attempts, 0, sizeof(menu5_attempts));
                menu5_question_requested = true;
            }
            if (active_page == 3) {
                vowel_quiz_feedback = 0;
                vowel_quiz_selected_option = -1;
                memset(menu4_attempts, 0, sizeof(menu4_attempts));
                menu4_question_requested = true;
            }
            handled = true;
            changed = true;
        } else if (x >= 57 && x <= 423 && y >= 80 && y <= 211) {
            lesson_started = true;
            lesson_step[active_page] = 0;
            if (active_page == 2) {
                memset(phoneme_vowel_passed, 0, sizeof(phoneme_vowel_passed));
                phoneme_feedback_state = 0;
                phoneme_predicted_index = -1;
            }
            if (active_page == 0) menu1_audio_changed = true;
            if (active_page == 1) menu2_audio_changed = true;
            if (active_page == 4) {
                review_clouds_remaining = 0x1F;
                memset(menu5_attempts, 0, sizeof(menu5_attempts));
                menu5_question_requested = true;
            }
            if (active_page == 3) {
                vowel_quiz_feedback = 0;
                vowel_quiz_selected_option = -1;
                memset(menu4_attempts, 0, sizeof(menu4_attempts));
                menu4_question_requested = true;
            }
            handled = true;
            changed = true;
        }
    } else if (((x >= 20 && x <= 72) || (x >= 415 && x <= 468)) &&
               y >= 18 && y <= 61) {
        lesson_started = false;
        handled = true;
        changed = true;
    } else if (active_page == 3 && vowel_quiz_feedback == 2 &&
               x >= 168 && x <= 312 && y >= 274 && y <= 311) {
        lesson_step[3] = (lesson_step[3] + 1) % 5;
        menu4_attempts[lesson_step[3]] = 0;
        vowel_quiz_feedback = 0;
        vowel_quiz_selected_option = -1;
        menu4_question_requested = true;
        handled = true;
        changed = true;
    } else if (active_page == 3 && y >= 143 && y <= 223 &&
               ((x >= 93 && x <= 207) || (x >= 273 && x <= 387))) {
        static const char vowels[] = "AIUEO";
        int target_index = lesson_step[3] % 5;
        int correct_option = lesson_step[3] % 2;
        int selected_option = x < 240 ? 0 : 1;
        char selected_letter = selected_option == correct_option
            ? vowels[target_index] : vowels[(target_index + 1) % 5];
        if (vowel_quiz_feedback != 2) {
            vowel_quiz_selected_option = selected_option;
            vowel_quiz_feedback = selected_letter == vowels[target_index] ? 2 : 1;
            uint32_t attempt = ++menu4_attempts[target_index];
            (void)activity_log_record(ACTIVITY_LOG_VOWEL_CHOICE,
                                      vowels[target_index],
                                      selected_letter == vowels[target_index],
                                      attempt);
            menu4_feedback_request = vowel_quiz_feedback == 2 ? 2 : 1;
            handled = true;
            changed = true;
        }
    } else if (active_page == 4) {
        static const int cloud_centers_x[5] = {120, 240, 360, 180, 300};
        static const int cloud_centers_y[5] = {166, 166, 166, 226, 226};
        for (int i = 0; i < 5; ++i) {
            if ((review_clouds_remaining & (1U << i)) &&
                x >= cloud_centers_x[i] - 42 && x <= cloud_centers_x[i] + 42 &&
                y >= cloud_centers_y[i] - 36 &&
                y <= cloud_centers_y[i] + 24) {
                handled = true;
                static const char vowels[] = "AIUEO";
                int target_index = lesson_step[4];
                uint32_t attempt = ++menu5_attempts[target_index];
                (void)activity_log_record(ACTIVITY_LOG_CLOUD_SEARCH,
                                          vowels[target_index],
                                          i == target_index, attempt);
                if (i == lesson_step[4]) {
                    review_clouds_remaining &= (uint8_t)~(1U << i);
                    ++lesson_step[4];
                    changed = true;
                    menu5_cloud_found = i;
                    menu5_is_final_vowel = review_clouds_remaining == 0;
                } else {
                    menu5_incorrect_requested = true;
                }
                break;
            }
        }
    } else if (active_page == 1 && y >= 76 && y <= 265) {
        handled = true;
    } else if (active_page == 2 &&
               x >= 213 && x <= 267 && y >= 265 && y <= 319) {
        handled = true;
        if (!phoneme_busy) {
            menu3_attempt_requested = true;
            menu3_attempt_vowel = lesson_step[2] % PHONEME_CLASS_COUNT;
        } else if (request_menu3_stop()) {
            changed = true;
        }
    } else if (active_page != 3 && active_page != 4 &&
               y >= 270 && y <= 319 &&
               x >= 75 && x <= 125) {
        int step_count = active_page == 0 ? 26 : active_page == 2 ? PHONEME_CLASS_COUNT :
                         active_page == 3 ? 5 : active_page == 4 ? 5 : 5;
        lesson_step[active_page] = (lesson_step[active_page] + step_count - 1) % step_count;
        if (active_page == 0) menu1_audio_changed = true;
        if (active_page == 1) menu2_audio_changed = true;
        if (active_page == 2) {
            phoneme_feedback_state = 0;
            phoneme_predicted_index = -1;
        }
        handled = true;
        changed = true;
    } else if (active_page != 3 && active_page != 4 &&
               y >= 270 && y <= 319 &&
               x >= 365 && x <= 405) {
        int step_count = active_page == 0 ? 26 : active_page == 2 ? PHONEME_CLASS_COUNT :
                         active_page == 3 ? 5 : active_page == 4 ? 5 : 5;
        lesson_step[active_page] = (lesson_step[active_page] + 1) % step_count;
        if (active_page == 0) menu1_audio_changed = true;
        if (active_page == 1) menu2_audio_changed = true;
        if (active_page == 2) {
            phoneme_feedback_state = 0;
            phoneme_predicted_index = -1;
        }
        handled = true;
        changed = true;
    }
    if (handled) request_touch_beep();
    if (menu3_attempt_requested &&
        request_menu3_attempt(menu3_attempt_vowel)) {
        phoneme_feedback_state = 0;
        changed = true;
    }
    if (changed) render_ui();
    if (menu1_intro_requested) request_menu1_intro();
    if (menu1_audio_changed) request_menu1_letter(lesson_step[0]);
    if (menu2_intro_requested) request_menu2_intro();
    if (menu2_audio_changed) request_menu2_vowel(lesson_step[1] % 5);
    if (menu3_intro_requested) request_menu3_intro();
    if (menu4_intro_requested) request_menu4_intro();
    if (menu4_question_requested) request_menu4_question(lesson_step[3] % 5);
    if (menu4_feedback_request != 0) {
        request_menu4_feedback(menu4_feedback_request == 2);
    }
    if (menu5_intro_requested) request_menu5_intro();
    if (menu5_question_requested) {
        request_menu5_question(lesson_step[4]);
    }
    if (menu5_cloud_found >= 0) {
        request_menu5_found(menu5_cloud_found, menu5_is_final_vowel);
    }
    if (menu5_incorrect_requested) {
        request_menu5_incorrect();
    }
    return handled;
}

static void request_touch_beep(void)
{
    if (speaker_beep_queue == NULL) return;
    const speaker_request_t request = SPEAKER_REQUEST_BEEP;
    if (xQueueSend(speaker_beep_queue, &request, 0) != pdPASS) {
        ESP_LOGW(TAG, "Antrean beep speaker penuh; efek sentuh dilewati.");
    }
}

static void request_menu1_intro(void)
{
    if (speaker_beep_queue == NULL) {
        ESP_LOGW(TAG, "Audio pengantar Menu 1 belum tersedia; permintaan dilewati.");
        return;
    }

    const speaker_request_t request = SPEAKER_REQUEST_MENU1_INTRO;
    if (xQueueSend(speaker_beep_queue, &request, 0) != pdPASS) {
        ESP_LOGW(TAG, "Antrean speaker penuh; audio pengantar Menu 1 dilewati.");
    }
}

static void request_menu2_intro(void)
{
    if (speaker_beep_queue == NULL) {
        ESP_LOGW(TAG, "Audio pengantar Menu 2 belum tersedia; permintaan dilewati.");
        return;
    }

    const speaker_request_t request = SPEAKER_REQUEST_MENU2_INTRO;
    if (xQueueSend(speaker_beep_queue, &request, 0) != pdPASS) {
        ESP_LOGW(TAG, "Antrean speaker penuh; audio pengantar Menu 2 dilewati.");
    } else {
        ESP_LOGI(TAG, "Audio pengantar Menu 2 masuk antrean speaker.");
    }
}

static void request_menu1_letter(int letter_index)
{
    if (letter_index < 0 || letter_index >= 26) {
        ESP_LOGE(TAG, "Indeks audio Menu 1 tidak valid: %d.", letter_index);
        return;
    }
    if (speaker_beep_queue == NULL) {
        ESP_LOGW(TAG, "Audio huruf Menu 1 belum tersedia; permintaan dilewati.");
        return;
    }

    const speaker_request_t request =
        (speaker_request_t)(SPEAKER_REQUEST_MENU1_LETTER_A + letter_index);
    if (xQueueSend(speaker_beep_queue, &request, 0) != pdPASS) {
        ESP_LOGW(TAG, "Antrean speaker penuh; audio huruf %c dilewati.",
                 'A' + letter_index);
    }
}

static void request_menu2_vowel(int vowel_index)
{
    if (vowel_index < 0 || vowel_index >= 5) {
        ESP_LOGE(TAG, "Indeks audio vokal Menu 2 tidak valid: %d.", vowel_index);
        return;
    }
    if (speaker_beep_queue == NULL) {
        ESP_LOGW(TAG, "Audio vokal Menu 2 belum tersedia; permintaan dilewati.");
        return;
    }

    const speaker_request_t request =
        (speaker_request_t)(SPEAKER_REQUEST_MENU2_VOWEL_A + vowel_index);
    if (xQueueSend(speaker_beep_queue, &request, 0) != pdPASS) {
        ESP_LOGW(TAG, "Antrean speaker penuh; audio vokal %c dilewati.",
                 "AIUEO"[vowel_index]);
    } else {
        ESP_LOGI(TAG, "Audio vokal Menu 2 %c masuk antrean speaker.",
                 "AIUEO"[vowel_index]);
    }
}

static void request_menu3_intro(void)
{
    if (speaker_beep_queue == NULL) {
        ESP_LOGW(TAG, "Audio pengantar Menu 3 belum tersedia; permintaan dilewati.");
        return;
    }
    const speaker_request_t request = SPEAKER_REQUEST_MENU3_INTRO;
    if (xQueueSend(speaker_beep_queue, &request, 0) != pdPASS) {
        ESP_LOGW(TAG, "Antrean speaker penuh; pengantar Menu 3 dilewati.");
    }
}

static bool request_menu3_attempt(int vowel_index)
{
    if (vowel_index < 0 || vowel_index >= PHONEME_CLASS_COUNT) {
        ESP_LOGE(TAG, "Indeks fonem Menu 3 tidak valid: %d.", vowel_index);
        return false;
    }
    if (speaker_beep_queue == NULL || phoneme_capture_queue == NULL) {
        ESP_LOGE(TAG, "Audio atau perekam Menu 3 belum siap.");
        return false;
    }
    if (phoneme_busy) return false;

    speaker_request_t request =
        (speaker_request_t)(SPEAKER_REQUEST_MENU3_PROMPT_A + vowel_index);
    if (xQueueSend(speaker_beep_queue, &request, 0) != pdPASS) {
        ESP_LOGW(TAG, "Antrean speaker penuh; percobaan Menu 3 gagal dijadwalkan.");
        return false;
    }
    phoneme_busy = true;
    phoneme_recording_state = PHONEME_RECORDING_PROMPTING;
    ESP_LOGI(TAG, "Contoh fonem %c Menu 3 dijadwalkan sebelum rekaman.",
             "AIUEO"[vowel_index]);
    return true;
}

static bool request_menu3_stop(void)
{
    if (!phoneme_busy ||
        phoneme_recording_state != PHONEME_RECORDING_ACTIVE ||
        phoneme_capture_task_handle == NULL) {
        return false;
    }
    if (xTaskNotify(phoneme_capture_task_handle, 1U, eSetBits) != pdPASS) {
        ESP_LOGE(TAG, "Tidak dapat mengirim perintah berhenti ke perekam Menu 3.");
        return false;
    }
    phoneme_recording_state = PHONEME_RECORDING_STOPPING;
    return true;
}

static void request_menu4_intro(void)
{
    if (speaker_beep_queue == NULL) {
        ESP_LOGW(TAG, "Audio pengantar Menu 4 belum tersedia; permintaan dilewati.");
        return;
    }

    const speaker_request_t request = SPEAKER_REQUEST_MENU4_INTRO;
    if (xQueueSend(speaker_beep_queue, &request, 0) != pdPASS) {
        ESP_LOGW(TAG, "Antrean speaker penuh; pengantar Menu 4 dilewati.");
    }
}

static void request_menu4_question(int question_index)
{
    if (question_index < 0 || question_index >= 5) {
        ESP_LOGE(TAG, "Indeks pertanyaan audio Menu 4 tidak valid: %d.",
                 question_index);
        return;
    }
    if (speaker_beep_queue == NULL) {
        ESP_LOGW(TAG, "Audio pertanyaan Menu 4 belum tersedia; permintaan dilewati.");
        return;
    }

    const speaker_request_t request =
        (speaker_request_t)(SPEAKER_REQUEST_MENU4_QUESTION_A_VS_I +
                            question_index);
    if (xQueueSend(speaker_beep_queue, &request, 0) != pdPASS) {
        ESP_LOGW(TAG, "Antrean speaker penuh; pertanyaan Menu 4 dilewati.");
    }
}

static void request_menu4_feedback(bool correct)
{
    if (speaker_beep_queue == NULL) {
        ESP_LOGW(TAG, "Audio umpan balik Menu 4 belum tersedia; permintaan dilewati.");
        return;
    }

    const speaker_request_t request = correct
        ? SPEAKER_REQUEST_MENU4_CORRECT : SPEAKER_REQUEST_MENU4_INCORRECT;
    if (xQueueSend(speaker_beep_queue, &request, 0) != pdPASS) {
        ESP_LOGW(TAG, "Antrean speaker penuh; umpan balik Menu 4 dilewati.");
    }
}

static void request_menu5_intro(void)
{
    if (speaker_beep_queue == NULL) {
        ESP_LOGW(TAG, "Audio pengantar Menu 5 belum tersedia; permintaan dilewati.");
        return;
    }

    const speaker_request_t request = SPEAKER_REQUEST_MENU5_INTRO;
    if (xQueueSend(speaker_beep_queue, &request, 0) != pdPASS) {
        ESP_LOGW(TAG, "Antrean speaker penuh; pengantar Menu 5 dilewati.");
    }
}

static void request_menu5_question(int vowel_index)
{
    if (vowel_index < 0 || vowel_index >= 5) {
        ESP_LOGE(TAG, "Indeks pertanyaan Menu 5 tidak valid: %d.",
                 vowel_index);
        return;
    }
    if (speaker_beep_queue == NULL) {
        ESP_LOGW(TAG, "Audio pertanyaan Menu 5 belum tersedia; permintaan dilewati.");
        return;
    }

    const speaker_request_t request =
        (speaker_request_t)(SPEAKER_REQUEST_MENU5_QUESTION_A + vowel_index);
    if (xQueueSend(speaker_beep_queue, &request, 0) != pdPASS) {
        ESP_LOGW(TAG, "Antrean speaker penuh; pertanyaan Menu 5 dilewati.");
    }
}

static void request_menu5_found(int vowel_index, bool is_final_vowel)
{
    if (vowel_index < 0 || vowel_index >= 5) {
        ESP_LOGE(TAG, "Indeks vokal ditemukan Menu 5 tidak valid: %d.",
                 vowel_index);
        return;
    }
    if (speaker_beep_queue == NULL) {
        ESP_LOGW(TAG, "Audio jawaban benar Menu 5 belum tersedia; permintaan dilewati.");
        return;
    }

    speaker_request_t request =
        (speaker_request_t)(SPEAKER_REQUEST_MENU5_FOUND_A + vowel_index);
    if (is_final_vowel) {
        request = (speaker_request_t)(request + 8);
    }
    if (xQueueSend(speaker_beep_queue, &request, 0) != pdPASS) {
        ESP_LOGW(TAG, "Antrean speaker penuh; audio jawaban benar Menu 5 dilewati.");
    }
}

static void request_menu5_incorrect(void)
{
    if (speaker_beep_queue == NULL) {
        ESP_LOGW(TAG, "Audio jawaban salah Menu 5 belum tersedia; permintaan dilewati.");
        return;
    }

    const speaker_request_t request = SPEAKER_REQUEST_MENU5_INCORRECT;
    if (xQueueSend(speaker_beep_queue, &request, 0) != pdPASS) {
        ESP_LOGW(TAG, "Antrean speaker penuh; audio jawaban salah Menu 5 dilewati.");
    }
}

static void publish_phoneme_result(uint8_t target_index, int predicted_index,
                                   float confidence, bool valid)
{
    if (phoneme_result_queue == NULL) {
        ESP_LOGE(TAG, "Antrean hasil fonem Menu 3 tidak tersedia.");
        phoneme_busy = false;
        return;
    }
    phoneme_result_event_t result = {
        .target_index = target_index,
        .predicted_index = (int8_t)predicted_index,
        .confidence = confidence,
        .valid = valid
    };
    if (xQueueSend(phoneme_result_queue, &result, pdMS_TO_TICKS(100)) != pdPASS) {
        ESP_LOGE(TAG, "Antrean hasil fonem Menu 3 penuh; hasil tidak dapat ditampilkan.");
        phoneme_busy = false;
    }
}

static void request_welcome_playback(void)
{
    if (welcome_playback_requested) {
        ESP_LOGW(TAG, "Playback sambutan sudah diminta pada boot ini; permintaan ganda diabaikan.");
        return;
    }
    welcome_playback_requested = true;

    if (speaker_beep_queue == NULL) {
        ESP_LOGE(TAG, "Playback sambutan tidak dapat diminta: task speaker belum siap.");
        return;
    }
    const speaker_request_t request = SPEAKER_REQUEST_WELCOME;
    if (xQueueSend(speaker_beep_queue, &request, 0) != pdPASS) {
        ESP_LOGE(TAG, "Antrean speaker penuh; playback sambutan gagal dijadwalkan.");
        return;
    }
    ESP_LOGI(TAG, "Playback sambutan dijadwalkan setelah beranda tampil.");
}

static void microphone_test_task(void *arg)
{
    (void)arg;
    const size_t frame_bytes = 2 * sizeof(int32_t);

    while (1) {
        uint8_t request;
        if (xQueueReceive(mic_test_queue, &request, portMAX_DELAY) != pdPASS) {
            continue;
        }

        i2s_chan_handle_t mic_rx_channel = NULL;
        uint32_t total_bytes = 0;
        uint32_t timeout_reads = 0;
        uint32_t empty_reads = 0;
        uint32_t unaligned_reads = 0;
        uint32_t sample_count = 0;
        uint32_t nonzero_samples = 0;
        int16_t min_sample = 0;
        int16_t max_sample = 0;
        int32_t peak = 0;
        int64_t sum_samples = 0;
        uint64_t sum_squares = 0;
        TickType_t start_ticks = 0;
        bool capture_started = false;
        esp_err_t read_error = ESP_OK;

        i2s_chan_config_t channel_config =
            I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
        esp_err_t err = i2s_new_channel(&channel_config, NULL, &mic_rx_channel);
        if (err != ESP_OK) {
            read_error = err;
            ESP_LOGE(TAG, "INMP441: gagal membuat I2S RX: %s", esp_err_to_name(err));
            goto test_cleanup;
        }

        i2s_std_config_t standard_config = {
            .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(MIC_SAMPLE_RATE),
            .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
            .gpio_cfg = {
                .mclk = I2S_GPIO_UNUSED,
                .bclk = MIC_PIN_BCLK,
                .ws = MIC_PIN_WS,
                .dout = I2S_GPIO_UNUSED,
                .din = MIC_PIN_DOUT,
                .invert_flags = {0}
            }
        };
        err = i2s_channel_init_std_mode(mic_rx_channel, &standard_config);
        if (err != ESP_OK) {
            read_error = err;
            ESP_LOGE(TAG, "INMP441: gagal menginisialisasi I2S: %s",
                     esp_err_to_name(err));
            goto test_cleanup;
        }
        err = i2s_channel_enable(mic_rx_channel);
        if (err != ESP_OK) {
            read_error = err;
            ESP_LOGE(TAG, "INMP441: gagal mengaktifkan I2S RX: %s",
                     esp_err_to_name(err));
            goto test_cleanup;
        }

        start_ticks = xTaskGetTickCount();
        capture_started = true;
        ESP_LOGI(TAG,
                 "INMP441 tes mulai: durasi %d ms, %d Hz, slot kiri dari stereo 32-bit.",
                 MIC_TEST_DURATION_MS, MIC_SAMPLE_RATE);

        while ((TickType_t)(xTaskGetTickCount() - start_ticks) <
               pdMS_TO_TICKS(MIC_TEST_DURATION_MS)) {
            size_t bytes_read = 0;
            err = i2s_channel_read(mic_rx_channel, mic_rx_buffer,
                                   sizeof(mic_rx_buffer), &bytes_read,
                                   MIC_READ_TIMEOUT_MS);
            if (err == ESP_ERR_TIMEOUT) {
                ++timeout_reads;
                continue;
            }
            if (err != ESP_OK) {
                read_error = err;
                ESP_LOGE(TAG, "INMP441: pembacaan I2S gagal: %s",
                         esp_err_to_name(err));
                break;
            }
            if (bytes_read == 0) {
                ++empty_reads;
                vTaskDelay(pdMS_TO_TICKS(1));
                continue;
            }

            total_bytes += (uint32_t)bytes_read;
            size_t word_count = bytes_read / sizeof(mic_rx_buffer[0]);
            if (bytes_read % frame_bytes != 0) ++unaligned_reads;
            for (size_t i = 0; i + 1 < word_count; i += 2) {
                int32_t pcm = mic_rx_buffer[i] >> 16;
                int32_t magnitude = pcm < 0 ? -pcm : pcm;
                if (sample_count == 0 || pcm < min_sample) min_sample = (int16_t)pcm;
                if (sample_count == 0 || pcm > max_sample) max_sample = (int16_t)pcm;
                if (magnitude > peak) peak = magnitude;
                if (pcm != 0) ++nonzero_samples;
                sum_samples += pcm;
                sum_squares += (uint64_t)(magnitude * magnitude);
                ++sample_count;
            }
        }

test_cleanup:
        if (mic_rx_channel != NULL) {
            esp_err_t cleanup_err = i2s_channel_disable(mic_rx_channel);
            if (cleanup_err != ESP_OK && cleanup_err != ESP_ERR_INVALID_STATE) {
                ESP_LOGW(TAG, "INMP441: gagal menonaktifkan I2S RX: %s",
                         esp_err_to_name(cleanup_err));
            }
            cleanup_err = i2s_del_channel(mic_rx_channel);
            if (cleanup_err != ESP_OK) {
                ESP_LOGW(TAG, "INMP441: gagal melepas I2S RX: %s",
                         esp_err_to_name(cleanup_err));
            }
        }

        {
            uint32_t elapsed_ms = capture_started
                ? (uint32_t)(xTaskGetTickCount() - start_ticks) * portTICK_PERIOD_MS
                : 0;
            uint32_t rms = 0;
            if (sample_count > 0) {
                double mean = (double)sum_samples / sample_count;
                double mean_square = (double)sum_squares / sample_count;
                double variance = mean_square - mean * mean;
                rms = (uint32_t)(sqrt(variance > 0.0 ? variance : 0.0) + 0.5);
            }
            bool valid = read_error == ESP_OK &&
                         elapsed_ms >= 4800 &&
                         sample_count >= (uint32_t)(MIC_SAMPLE_RATE * 4) &&
                         nonzero_samples > 0 &&
                         max_sample > min_sample;
            ESP_LOGI(TAG,
                     "INMP441 tes selesai: status=%s durasi=%u ms bytes=%u "
                     "sampel_kiri=%u nonzero=%u peak=%ld rms=%u range=[%d,%d] "
                     "timeout=%u read_kosong=%u read_tidak_selaras=%u error=%s.",
                     valid ? "VALID" : "TIDAK_VALID",
                     (unsigned)elapsed_ms,
                     (unsigned)total_bytes, (unsigned)sample_count,
                     (unsigned)nonzero_samples, (long)peak, (unsigned)rms,
                     (int)min_sample, (int)max_sample,
                     (unsigned)timeout_reads, (unsigned)empty_reads,
                     (unsigned)unaligned_reads,
                     esp_err_to_name(read_error));
        }
        portENTER_CRITICAL(&mic_test_state_lock);
        mic_test_active = false;
        portEXIT_CRITICAL(&mic_test_state_lock);
    }
}

static void phoneme_capture_task(void *arg)
{
    (void)arg;
    phoneme_capture_task_handle = xTaskGetCurrentTaskHandle();
    int16_t *samples = heap_caps_malloc(
        PHONEME_SAMPLE_RATE * (PHONEME_CAPTURE_DURATION_MS / 1000) *
            sizeof(*samples),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    bool model_ready = readzio_phoneme_model_init();
    if (samples == NULL) {
        ESP_LOGE(TAG, "Tidak dapat mengalokasikan buffer rekaman Menu 3 di PSRAM.");
    }

    while (1) {
        uint8_t target_index;
        if (xQueueReceive(phoneme_capture_queue, &target_index,
                          portMAX_DELAY) != pdPASS) {
            continue;
        }
        if (target_index >= PHONEME_CLASS_COUNT) {
            ESP_LOGE(TAG, "Indeks target fonem dari antrean tidak valid: %u.",
                     (unsigned)target_index);
            publish_phoneme_result(target_index, -1, 0.0f, false);
            continue;
        }
        if (!model_ready || samples == NULL) {
            publish_phoneme_result(target_index, -1, 0.0f, false);
            continue;
        }

        i2s_chan_handle_t mic_rx_channel = NULL;
        esp_err_t err;
        i2s_chan_config_t channel_config =
            I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
        err = i2s_new_channel(&channel_config, NULL, &mic_rx_channel);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Menu 3: gagal membuat I2S RX: %s",
                     esp_err_to_name(err));
            publish_phoneme_result(target_index, -1, 0.0f, false);
            continue;
        }

        i2s_std_config_t standard_config = {
            .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(MIC_SAMPLE_RATE),
            .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
            .gpio_cfg = {
                .mclk = I2S_GPIO_UNUSED,
                .bclk = MIC_PIN_BCLK,
                .ws = MIC_PIN_WS,
                .dout = I2S_GPIO_UNUSED,
                .din = MIC_PIN_DOUT,
                .invert_flags = {0}
            }
        };
        err = i2s_channel_init_std_mode(mic_rx_channel, &standard_config);
        if (err == ESP_OK) err = i2s_channel_enable(mic_rx_channel);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Menu 3: gagal mengaktifkan mikrofon: %s",
                     esp_err_to_name(err));
            (void)i2s_del_channel(mic_rx_channel);
            publish_phoneme_result(target_index, -1, 0.0f, false);
            continue;
        }

        const size_t target_samples =
            PHONEME_SAMPLE_RATE * (PHONEME_CAPTURE_DURATION_MS / 1000);
        size_t captured_samples = 0;
        uint32_t timeout_reads = 0;
        TickType_t capture_started = xTaskGetTickCount();
        phoneme_recording_state = PHONEME_RECORDING_ACTIVE;
        render_ui();
        ESP_LOGI(TAG, "Menu 3 mulai merekam %c; ketuk STOP REKAM untuk "
                      "mengakhiri, batas otomatis %d ms.",
                 "AIUEO"[target_index], PHONEME_CAPTURE_DURATION_MS);
        while (captured_samples < target_samples &&
               (TickType_t)(xTaskGetTickCount() - capture_started) <
                   pdMS_TO_TICKS(PHONEME_CAPTURE_DURATION_MS + 500)) {
            uint32_t notifications = 0;
            if (xTaskNotifyWait(0, UINT32_MAX, &notifications, 0) == pdTRUE &&
                (notifications & 1U) != 0) {
                break;
            }
            size_t bytes_read = 0;
            err = i2s_channel_read(mic_rx_channel, mic_rx_buffer,
                                   sizeof(mic_rx_buffer), &bytes_read,
                                   MIC_READ_TIMEOUT_MS);
            if (err == ESP_ERR_TIMEOUT) {
                ++timeout_reads;
                continue;
            }
            if (err != ESP_OK) break;
            size_t words = bytes_read / sizeof(mic_rx_buffer[0]);
            for (size_t word = 0; word + 1 < words &&
                                  captured_samples < target_samples;
                 word += 2) {
                samples[captured_samples++] =
                    (int16_t)(mic_rx_buffer[word] >> 16);
            }
        }
        phoneme_recording_state = PHONEME_RECORDING_ANALYZING;
        render_ui();

        esp_err_t cleanup_err = i2s_channel_disable(mic_rx_channel);
        if (cleanup_err != ESP_OK && cleanup_err != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "Menu 3: gagal menonaktifkan mikrofon: %s",
                     esp_err_to_name(cleanup_err));
        }
        cleanup_err = i2s_del_channel(mic_rx_channel);
        if (cleanup_err != ESP_OK) {
            ESP_LOGW(TAG, "Menu 3: gagal melepas kanal mikrofon: %s",
                     esp_err_to_name(cleanup_err));
        }

        if (err != ESP_OK || captured_samples < PHONEME_SAMPLE_RATE) {
            ESP_LOGE(TAG, "Menu 3 rekaman gagal/tidak cukup: %u sampel, %s, "
                     "timeout=%u.",
                     (unsigned)captured_samples, esp_err_to_name(err),
                     (unsigned)timeout_reads);
            publish_phoneme_result(target_index, -1, 0.0f, false);
            continue;
        }

        if (!readzio_extract_phoneme_features(samples, captured_samples,
                                              phoneme_features)) {
            ESP_LOGW(TAG, "Menu 3 tidak menemukan suara yang cukup jelas.");
            publish_phoneme_result(target_index, -1, 0.0f, false);
            continue;
        }

        int predicted_index = -1;
        float probabilities[PHONEME_CLASS_COUNT] = {0};
        if (!readzio_phoneme_model_predict(phoneme_features,
                                           &predicted_index, probabilities)) {
            ESP_LOGE(TAG, "Menu 3 gagal mengklasifikasikan rekaman.");
            publish_phoneme_result(target_index, -1, 0.0f, false);
            continue;
        }
        ESP_LOGI(TAG, "Menu 3 target=%c prediksi=%c keyakinan=%.1f%%.",
                 "AIUEO"[target_index], "AEIOU"[predicted_index],
                 probabilities[predicted_index] * 100.0f);
        publish_phoneme_result(target_index, predicted_index,
                               probabilities[predicted_index], true);
    }
}

static void process_phoneme_result(void)
{
    if (phoneme_result_queue == NULL) return;
    phoneme_result_event_t result;
    while (xQueueReceive(phoneme_result_queue, &result, 0) == pdPASS) {
        phoneme_busy = false;
        phoneme_recording_state = PHONEME_RECORDING_IDLE;
        if (active_page != 2 || !lesson_started ||
            result.target_index != lesson_step[2] % PHONEME_CLASS_COUNT) {
            if (active_page == 2 && lesson_started) render_ui();
            continue;
        }

        phoneme_predicted_index = result.predicted_index;
        phoneme_confidence = result.confidence;
        bool correct = result.valid && result.predicted_index >= 0 &&
                       result.predicted_index < PHONEME_CLASS_COUNT &&
                       "AEIOU"[result.predicted_index] ==
                           "AIUEO"[result.target_index];
        phoneme_feedback_state = !result.valid ? 3 : correct ? 1 : 2;
        if (correct) phoneme_vowel_passed[result.target_index] = true;
        render_ui();

        if (result.valid && speaker_beep_queue != NULL) {
            speaker_request_t request = correct
                ? SPEAKER_REQUEST_MENU3_CORRECT
                : SPEAKER_REQUEST_MENU3_INCORRECT;
            if (xQueueSend(speaker_beep_queue, &request, 0) != pdPASS) {
                ESP_LOGW(TAG, "Antrean speaker penuh; umpan balik Menu 3 dilewati.");
            }
        }
    }
}

static void ui_event_task(void *arg)
{
    (void)arg;
    touch_event_t event;
    while (1) {
        if (xQueueReceive(ui_event_queue, &event, pdMS_TO_TICKS(20)) == pdPASS) {
            bool handled = handle_touch(event.x, event.y);
            ESP_LOGI(TAG,
                     "Touch raw=(%u,%u) screen=(%d,%d) target=%s page=%d.",
                     (unsigned)event.raw_x, (unsigned)event.raw_y,
                     event.x, event.y, handled ? "yes" : "no", active_page);
        }
        process_phoneme_result();
    }
}

extern const uint8_t _binary_welcome_readzio_wav_start[];
extern const uint8_t _binary_welcome_readzio_wav_end[];
extern const uint8_t _binary_pengantar_menu1_clean_wav_start[];
extern const uint8_t _binary_pengantar_menu1_clean_wav_end[];
extern const uint8_t _binary_pengantar_menu2_clean_ima4_start[];
extern const uint8_t _binary_pengantar_menu2_clean_ima4_end[];
extern const uint8_t _binary_A_clean_ima4_start[];
extern const uint8_t _binary_A_clean_ima4_end[];
extern const uint8_t _binary_I_clean_ima4_start[];
extern const uint8_t _binary_I_clean_ima4_end[];
extern const uint8_t _binary_U_clean_ima4_start[];
extern const uint8_t _binary_U_clean_ima4_end[];
extern const uint8_t _binary_E_clean_ima4_start[];
extern const uint8_t _binary_E_clean_ima4_end[];
extern const uint8_t _binary_O_clean_ima4_start[];
extern const uint8_t _binary_O_clean_ima4_end[];
extern const uint8_t _binary_pengantar_menu3_clean_ima4_start[];
extern const uint8_t _binary_pengantar_menu3_clean_ima4_end[];
extern const uint8_t _binary_ucapkan_A_clean_ima4_start[];
extern const uint8_t _binary_ucapkan_A_clean_ima4_end[];
extern const uint8_t _binary_ucapkan_I_clean_ima4_start[];
extern const uint8_t _binary_ucapkan_I_clean_ima4_end[];
extern const uint8_t _binary_ucapkan_U_clean_ima4_start[];
extern const uint8_t _binary_ucapkan_U_clean_ima4_end[];
extern const uint8_t _binary_ucapkan_E_clean_ima4_start[];
extern const uint8_t _binary_ucapkan_E_clean_ima4_end[];
extern const uint8_t _binary_ucapkan_O_clean_ima4_start[];
extern const uint8_t _binary_ucapkan_O_clean_ima4_end[];
extern const uint8_t _binary_jika_benar_clean_ima4_start[];
extern const uint8_t _binary_jika_benar_clean_ima4_end[];
extern const uint8_t _binary_jika_kurangtepat_clean_ima4_start[];
extern const uint8_t _binary_jika_kurangtepat_clean_ima4_end[];
extern const uint8_t _binary_pengantar_menu4_clean_ima4_start[];
extern const uint8_t _binary_pengantar_menu4_clean_ima4_end[];
extern const uint8_t _binary_A_vs_I_clean_ima4_start[];
extern const uint8_t _binary_A_vs_I_clean_ima4_end[];
extern const uint8_t _binary_U_vs_I_clean_ima4_start[];
extern const uint8_t _binary_U_vs_I_clean_ima4_end[];
extern const uint8_t _binary_U_vs_E_clean_ima4_start[];
extern const uint8_t _binary_U_vs_E_clean_ima4_end[];
extern const uint8_t _binary_O_vs_E_clean_ima4_start[];
extern const uint8_t _binary_O_vs_E_clean_ima4_end[];
extern const uint8_t _binary_O_vs_A_clean_ima4_start[];
extern const uint8_t _binary_O_vs_A_clean_ima4_end[];
extern const uint8_t _binary_Hebat_clean_ima4_start[];
extern const uint8_t _binary_Hebat_clean_ima4_end[];
extern const uint8_t _binary_Jika_salah_clean_ima4_start[];
extern const uint8_t _binary_Jika_salah_clean_ima4_end[];
extern const uint8_t _binary_pengantar_menu5_clean_ima4_start[];
extern const uint8_t _binary_pengantar_menu5_clean_ima4_end[];
extern const uint8_t _binary_ketemu_clean_ima4_start[];
extern const uint8_t _binary_ketemu_clean_ima4_end[];
extern const uint8_t _binary_Finish_awan_clean_ima4_start[];
extern const uint8_t _binary_Finish_awan_clean_ima4_end[];
extern const uint8_t _binary_awan_A_clean_ima4_start[];
extern const uint8_t _binary_awan_A_clean_ima4_end[];
extern const uint8_t _binary_awan_I_clean_ima4_start[];
extern const uint8_t _binary_awan_I_clean_ima4_end[];
extern const uint8_t _binary_awan_U_clean_ima4_start[];
extern const uint8_t _binary_awan_U_clean_ima4_end[];
extern const uint8_t _binary_awan_E_clean_ima4_start[];
extern const uint8_t _binary_awan_E_clean_ima4_end[];
extern const uint8_t _binary_awan_O_clean_ima4_start[];
extern const uint8_t _binary_awan_O_clean_ima4_end[];
extern const uint8_t _binary_Aa_clean_ima4_start[];
extern const uint8_t _binary_Aa_clean_ima4_end[];
extern const uint8_t _binary_Bb_clean_ima4_start[];
extern const uint8_t _binary_Bb_clean_ima4_end[];
extern const uint8_t _binary_Cc_clean_ima4_start[];
extern const uint8_t _binary_Cc_clean_ima4_end[];
extern const uint8_t _binary_Dd_clean_ima4_start[];
extern const uint8_t _binary_Dd_clean_ima4_end[];
extern const uint8_t _binary_Ee_clean_ima4_start[];
extern const uint8_t _binary_Ee_clean_ima4_end[];
extern const uint8_t _binary_Ff_clean_ima4_start[];
extern const uint8_t _binary_Ff_clean_ima4_end[];
extern const uint8_t _binary_Gg_clean_ima4_start[];
extern const uint8_t _binary_Gg_clean_ima4_end[];
extern const uint8_t _binary_Hh_clean_ima4_start[];
extern const uint8_t _binary_Hh_clean_ima4_end[];
extern const uint8_t _binary_Ii_clean_ima4_start[];
extern const uint8_t _binary_Ii_clean_ima4_end[];
extern const uint8_t _binary_Jj_clean_ima4_start[];
extern const uint8_t _binary_Jj_clean_ima4_end[];
extern const uint8_t _binary_Kk_clean_ima4_start[];
extern const uint8_t _binary_Kk_clean_ima4_end[];
extern const uint8_t _binary_Ll_clean_ima4_start[];
extern const uint8_t _binary_Ll_clean_ima4_end[];
extern const uint8_t _binary_Mm_clean_ima4_start[];
extern const uint8_t _binary_Mm_clean_ima4_end[];
extern const uint8_t _binary_Nn_clean_ima4_start[];
extern const uint8_t _binary_Nn_clean_ima4_end[];
extern const uint8_t _binary_Oo_clean_ima4_start[];
extern const uint8_t _binary_Oo_clean_ima4_end[];
extern const uint8_t _binary_Pp_clean_ima4_start[];
extern const uint8_t _binary_Pp_clean_ima4_end[];
extern const uint8_t _binary_Qq_clean_ima4_start[];
extern const uint8_t _binary_Qq_clean_ima4_end[];
extern const uint8_t _binary_Rr_clean_ima4_start[];
extern const uint8_t _binary_Rr_clean_ima4_end[];
extern const uint8_t _binary_Ss_clean_ima4_start[];
extern const uint8_t _binary_Ss_clean_ima4_end[];
extern const uint8_t _binary_Tt_clean_ima4_start[];
extern const uint8_t _binary_Tt_clean_ima4_end[];
extern const uint8_t _binary_Uu_clean_ima4_start[];
extern const uint8_t _binary_Uu_clean_ima4_end[];
extern const uint8_t _binary_Vv_clean_ima4_start[];
extern const uint8_t _binary_Vv_clean_ima4_end[];
extern const uint8_t _binary_Ww_clean_ima4_start[];
extern const uint8_t _binary_Ww_clean_ima4_end[];
extern const uint8_t _binary_Xx_clean_ima4_start[];
extern const uint8_t _binary_Xx_clean_ima4_end[];
extern const uint8_t _binary_Yy_clean_ima4_start[];
extern const uint8_t _binary_Yy_clean_ima4_end[];
extern const uint8_t _binary_Zz_clean_ima4_start[];
extern const uint8_t _binary_Zz_clean_ima4_end[];

typedef struct {
    const uint8_t *start;
    const uint8_t *end;
} embedded_ima4_asset_t;

static const embedded_ima4_asset_t menu1_audio_assets[26] = {
    {_binary_Aa_clean_ima4_start, _binary_Aa_clean_ima4_end},
    {_binary_Bb_clean_ima4_start, _binary_Bb_clean_ima4_end},
    {_binary_Cc_clean_ima4_start, _binary_Cc_clean_ima4_end},
    {_binary_Dd_clean_ima4_start, _binary_Dd_clean_ima4_end},
    {_binary_Ee_clean_ima4_start, _binary_Ee_clean_ima4_end},
    {_binary_Ff_clean_ima4_start, _binary_Ff_clean_ima4_end},
    {_binary_Gg_clean_ima4_start, _binary_Gg_clean_ima4_end},
    {_binary_Hh_clean_ima4_start, _binary_Hh_clean_ima4_end},
    {_binary_Ii_clean_ima4_start, _binary_Ii_clean_ima4_end},
    {_binary_Jj_clean_ima4_start, _binary_Jj_clean_ima4_end},
    {_binary_Kk_clean_ima4_start, _binary_Kk_clean_ima4_end},
    {_binary_Ll_clean_ima4_start, _binary_Ll_clean_ima4_end},
    {_binary_Mm_clean_ima4_start, _binary_Mm_clean_ima4_end},
    {_binary_Nn_clean_ima4_start, _binary_Nn_clean_ima4_end},
    {_binary_Oo_clean_ima4_start, _binary_Oo_clean_ima4_end},
    {_binary_Pp_clean_ima4_start, _binary_Pp_clean_ima4_end},
    {_binary_Qq_clean_ima4_start, _binary_Qq_clean_ima4_end},
    {_binary_Rr_clean_ima4_start, _binary_Rr_clean_ima4_end},
    {_binary_Ss_clean_ima4_start, _binary_Ss_clean_ima4_end},
    {_binary_Tt_clean_ima4_start, _binary_Tt_clean_ima4_end},
    {_binary_Uu_clean_ima4_start, _binary_Uu_clean_ima4_end},
    {_binary_Vv_clean_ima4_start, _binary_Vv_clean_ima4_end},
    {_binary_Ww_clean_ima4_start, _binary_Ww_clean_ima4_end},
    {_binary_Xx_clean_ima4_start, _binary_Xx_clean_ima4_end},
    {_binary_Yy_clean_ima4_start, _binary_Yy_clean_ima4_end},
    {_binary_Zz_clean_ima4_start, _binary_Zz_clean_ima4_end}
};

static const embedded_ima4_asset_t menu2_intro_audio = {
    _binary_pengantar_menu2_clean_ima4_start,
    _binary_pengantar_menu2_clean_ima4_end
};

static const embedded_ima4_asset_t menu2_vowel_audio[5] = {
    {_binary_A_clean_ima4_start, _binary_A_clean_ima4_end},
    {_binary_I_clean_ima4_start, _binary_I_clean_ima4_end},
    {_binary_U_clean_ima4_start, _binary_U_clean_ima4_end},
    {_binary_E_clean_ima4_start, _binary_E_clean_ima4_end},
    {_binary_O_clean_ima4_start, _binary_O_clean_ima4_end}
};

static const embedded_ima4_asset_t menu3_intro_audio = {
    _binary_pengantar_menu3_clean_ima4_start,
    _binary_pengantar_menu3_clean_ima4_end
};

static const embedded_ima4_asset_t menu3_prompt_audio[5] = {
    {_binary_ucapkan_A_clean_ima4_start, _binary_ucapkan_A_clean_ima4_end},
    {_binary_ucapkan_I_clean_ima4_start, _binary_ucapkan_I_clean_ima4_end},
    {_binary_ucapkan_U_clean_ima4_start, _binary_ucapkan_U_clean_ima4_end},
    {_binary_ucapkan_E_clean_ima4_start, _binary_ucapkan_E_clean_ima4_end},
    {_binary_ucapkan_O_clean_ima4_start, _binary_ucapkan_O_clean_ima4_end}
};

static const embedded_ima4_asset_t menu3_correct_audio = {
    _binary_jika_benar_clean_ima4_start,
    _binary_jika_benar_clean_ima4_end
};

static const embedded_ima4_asset_t menu3_incorrect_audio = {
    _binary_jika_kurangtepat_clean_ima4_start,
    _binary_jika_kurangtepat_clean_ima4_end
};

static const embedded_ima4_asset_t menu4_intro_audio = {
    _binary_pengantar_menu4_clean_ima4_start,
    _binary_pengantar_menu4_clean_ima4_end
};

static const embedded_ima4_asset_t menu4_question_audio[5] = {
    {_binary_A_vs_I_clean_ima4_start, _binary_A_vs_I_clean_ima4_end},
    {_binary_U_vs_I_clean_ima4_start, _binary_U_vs_I_clean_ima4_end},
    {_binary_U_vs_E_clean_ima4_start, _binary_U_vs_E_clean_ima4_end},
    {_binary_O_vs_E_clean_ima4_start, _binary_O_vs_E_clean_ima4_end},
    {_binary_O_vs_A_clean_ima4_start, _binary_O_vs_A_clean_ima4_end}
};

static const embedded_ima4_asset_t menu4_correct_audio = {
    _binary_Hebat_clean_ima4_start, _binary_Hebat_clean_ima4_end
};

static const embedded_ima4_asset_t menu4_incorrect_audio = {
    _binary_Jika_salah_clean_ima4_start,
    _binary_Jika_salah_clean_ima4_end
};

static const embedded_ima4_asset_t menu5_intro_audio = {
    _binary_pengantar_menu5_clean_ima4_start,
    _binary_pengantar_menu5_clean_ima4_end
};

static const embedded_ima4_asset_t menu5_found_audio = {
    _binary_ketemu_clean_ima4_start, _binary_ketemu_clean_ima4_end
};

static const embedded_ima4_asset_t menu5_finish_audio = {
    _binary_Finish_awan_clean_ima4_start,
    _binary_Finish_awan_clean_ima4_end
};

static const embedded_ima4_asset_t menu5_cloud_audio[5] = {
    {_binary_awan_A_clean_ima4_start, _binary_awan_A_clean_ima4_end},
    {_binary_awan_I_clean_ima4_start, _binary_awan_I_clean_ima4_end},
    {_binary_awan_U_clean_ima4_start, _binary_awan_U_clean_ima4_end},
    {_binary_awan_E_clean_ima4_start, _binary_awan_E_clean_ima4_end},
    {_binary_awan_O_clean_ima4_start, _binary_awan_O_clean_ima4_end}
};

typedef struct {
    const uint8_t *pcm_data;
    size_t pcm_size;
    uint32_t sample_rate;
    uint32_t byte_rate;
    uint16_t channels;
    uint16_t block_align;
    uint16_t bits_per_sample;
} embedded_wav_t;

static uint16_t wav_read_u16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static uint32_t wav_read_u32(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

static bool parse_embedded_wav(const uint8_t *wav, size_t wav_size,
                               embedded_wav_t *audio)
{
    if (wav_size < 12 || memcmp(wav, "RIFF", 4) != 0 ||
        memcmp(wav + 8, "WAVE", 4) != 0) {
        return false;
    }

    uint32_t riff_size = wav_read_u32(wav + 4);
    if (riff_size < 4 || (size_t)riff_size > wav_size - 8) return false;
    size_t riff_end = (size_t)riff_size + 8;
    bool format_found = false;
    bool data_found = false;

    for (size_t offset = 12; offset <= riff_end && riff_end - offset >= 8;) {
        const uint8_t *chunk = wav + offset;
        uint32_t chunk_size = wav_read_u32(chunk + 4);
        size_t available = riff_end - offset - 8;
        if ((size_t)chunk_size > available) return false;

        const uint8_t *chunk_data = chunk + 8;
        if (memcmp(chunk, "fmt ", 4) == 0) {
            if (format_found || chunk_size < 16 ||
                wav_read_u16(chunk_data) != 1) {
                return false;
            }
            audio->channels = wav_read_u16(chunk_data + 2);
            audio->sample_rate = wav_read_u32(chunk_data + 4);
            audio->byte_rate = wav_read_u32(chunk_data + 8);
            audio->block_align = wav_read_u16(chunk_data + 12);
            audio->bits_per_sample = wav_read_u16(chunk_data + 14);
            format_found = true;
        } else if (memcmp(chunk, "data", 4) == 0) {
            if (data_found) return false;
            audio->pcm_data = chunk_data;
            audio->pcm_size = chunk_size;
            data_found = true;
        }

        size_t padded_size = (size_t)chunk_size + (chunk_size & 1U);
        if (padded_size > available) return false;
        offset += 8 + padded_size;
    }

    return format_found && data_found;
}

static int16_t wav_read_sample(const uint8_t *data)
{
    uint16_t sample = wav_read_u16(data);
    int32_t signed_sample = sample < 0x8000U
        ? (int32_t)sample : (int32_t)sample - 0x10000;
    return (int16_t)signed_sample;
}

static bool speaker_menu_context_matches(int expected_page,
                                         bool expected_lesson_started)
{
    return expected_page < 0 ||
           (active_page == expected_page &&
            lesson_started == expected_lesson_started);
}

typedef struct {
    const uint8_t *data;
    size_t sample_count;
    size_t sample_index;
    int32_t predictor;
    int index;
} ima4_decoder_t;

static const int ima4_step_table[89] = {
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31,
    34, 37, 41, 45, 50, 55, 60, 66, 73, 80, 88, 97, 107, 118, 130,
    143, 157, 173, 190, 209, 230, 253, 279, 307, 337, 371, 408, 449,
    494, 544, 598, 658, 724, 796, 876, 963, 1060, 1166, 1282, 1411,
    1552, 1707, 1878, 2066, 2272, 2499, 2749, 3024, 3327, 3660, 4026,
    4428, 4871, 5358, 5894, 6484, 7132, 7845, 8630, 9493, 10442,
    11487, 12635, 13899, 15289, 16818, 18500, 20350, 22385, 24623,
    27086, 29794, 32767
};

static const int ima4_index_table[16] = {
    -1, -1, -1, -1, 2, 4, 6, 8,
    -1, -1, -1, -1, 2, 4, 6, 8
};

static bool ima4_decoder_init(ima4_decoder_t *decoder,
                              const uint8_t *data, size_t data_size)
{
    if (data_size < 8) return false;
    size_t sample_count = wav_read_u32(data);
    if (sample_count == 0 ||
        (sample_count - 1 + 1) / 2 != data_size - 8 ||
        data[6] > 88 || data[7] != 0) {
        return false;
    }

    decoder->data = data + 8;
    decoder->sample_count = sample_count;
    decoder->sample_index = 0;
    decoder->predictor = wav_read_sample(data + 4);
    decoder->index = data[6];
    return true;
}

static int16_t ima4_decoder_next(ima4_decoder_t *decoder)
{
    if (decoder->sample_index == 0) {
        ++decoder->sample_index;
        return (int16_t)decoder->predictor;
    }

    size_t nibble_index = decoder->sample_index - 1;
    uint8_t packed = decoder->data[nibble_index / 2];
    int nibble = nibble_index & 1U ? packed >> 4 : packed & 0x0F;
    int step = ima4_step_table[decoder->index];
    int delta = step >> 3;
    if (nibble & 4) delta += step;
    if (nibble & 2) delta += step >> 1;
    if (nibble & 1) delta += step >> 2;
    decoder->predictor += nibble & 8 ? -delta : delta;
    if (decoder->predictor > 32767) decoder->predictor = 32767;
    if (decoder->predictor < -32768) decoder->predictor = -32768;
    decoder->index += ima4_index_table[nibble];
    if (decoder->index > 88) decoder->index = 88;
    if (decoder->index < 0) decoder->index = 0;
    ++decoder->sample_index;
    return (int16_t)decoder->predictor;
}

static bool speaker_play_embedded_wav(const char *description,
                                      const uint8_t *wav, size_t wav_size,
                                      int expected_page,
                                      bool expected_lesson_started)
{
    if (!speaker_menu_context_matches(expected_page,
                                     expected_lesson_started)) {
        ESP_LOGI(TAG, "Playback WAV %s dilewati; halaman sudah berubah.",
                 description);
        return true;
    }

    embedded_wav_t audio = {0};
    if (!parse_embedded_wav(wav, wav_size, &audio) ||
        audio.channels != 1 || audio.sample_rate != 16000 ||
        audio.bits_per_sample != 16 || audio.block_align != 2 ||
        audio.byte_rate != audio.sample_rate * audio.block_align ||
        audio.pcm_size == 0 || audio.pcm_size % audio.block_align != 0) {
        ESP_LOGE(TAG, "WAV %s tidak valid/tidak didukung (embedded=%u byte); playback dibatalkan.",
                 description, (unsigned)wav_size);
        return false;
    }

    const size_t source_samples = audio.pcm_size / audio.block_align;
    const size_t source_chunk_samples = 1024;
    const int64_t target_rms = 4125;  /* approximately -18 dBFS */
    const int64_t target_peak = 8192; /* 25% of signed 16-bit full scale */
    static int16_t output[1024 * 4];
    int64_t sum_squares = 0;
    int32_t source_peak = 0;
    size_t total_output_bytes = 0;

    for (size_t i = 0; i < source_samples; ++i) {
        int32_t sample = wav_read_sample(audio.pcm_data + i * audio.block_align);
        int32_t magnitude = sample < 0 ? -sample : sample;
        if (magnitude > source_peak) source_peak = magnitude;
        sum_squares += (int64_t)sample * sample;
    }
    if (source_peak == 0) {
        ESP_LOGE(TAG, "WAV %s senyap; playback dibatalkan.", description);
        return false;
    }
    uint32_t source_rms = (uint32_t)sqrt((double)sum_squares / source_samples);
    if (source_rms == 0) {
        ESP_LOGE(TAG, "RMS WAV %s tidak valid; playback dibatalkan.", description);
        return false;
    }
    int64_t gain_q15 = target_rms * 32768 / source_rms;
    int64_t peak_limited_gain_q15 = target_peak * 32768 / source_peak;
    if (gain_q15 > peak_limited_gain_q15) {
        gain_q15 = peak_limited_gain_q15;
    }

    ESP_LOGI(TAG,
             "WAV %s PCM mono %lu Hz/16-bit, %u frame, %u data byte; "
             "peak=%ld RMS=%u, peak_limit=%ld gain Q15=%ld, TX stereo %u Hz.",
             description, (unsigned long)audio.sample_rate, (unsigned)source_samples,
             (unsigned)audio.pcm_size, (long)source_peak,
             (unsigned)source_rms, (long)target_peak, (long)gain_q15,
             SPEAKER_SAMPLE_RATE);
    TickType_t playback_started = xTaskGetTickCount();
    size_t write_count = 0;
    ESP_LOGI(TAG, "WAV %s playback mulai.", description);

    for (size_t source_offset = 0; source_offset < source_samples;) {
        if (!speaker_menu_context_matches(expected_page,
                                          expected_lesson_started)) {
            ESP_LOGI(TAG, "Playback WAV %s dihentikan; halaman sudah berubah.",
                     description);
            return true;
        }

        size_t sample_count = source_samples - source_offset;
        if (sample_count > source_chunk_samples) {
            sample_count = source_chunk_samples;
        }

        for (size_t i = 0; i < sample_count; ++i) {
            size_t sample_index = source_offset + i;
            int32_t current = wav_read_sample(
                audio.pcm_data + sample_index * audio.block_align);
            int32_t next = current;
            if (sample_index + 1 < source_samples) {
                next = wav_read_sample(
                    audio.pcm_data + (sample_index + 1) * audio.block_align);
            }
            current = (int32_t)((int64_t)current * gain_q15 / 32768);
            next = (int32_t)((int64_t)next * gain_q15 / 32768);
            int32_t midpoint = (current + next) / 2;
            size_t output_index = i * 4;
            output[output_index] = (int16_t)current;
            output[output_index + 1] = (int16_t)current;
            output[output_index + 2] = (int16_t)midpoint;
            output[output_index + 3] = (int16_t)midpoint;
        }

        size_t bytes_expected = sample_count * 4 * sizeof(output[0]);
        size_t bytes_written = 0;
        esp_err_t err = i2s_channel_write(speaker_tx_channel, output,
                                          bytes_expected, &bytes_written,
                                          pdMS_TO_TICKS(1000));
        if (err != ESP_OK || bytes_written != bytes_expected) {
            ESP_LOGE(TAG, "Playback WAV %s gagal pada sampel %u: %s (%u/%u byte).",
                     description, (unsigned)source_offset, esp_err_to_name(err),
                     (unsigned)bytes_written, (unsigned)bytes_expected);
            return false;
        }
        total_output_bytes += bytes_written;
        ++write_count;
        source_offset += sample_count;
    }

    uint32_t elapsed_ms = (uint32_t)(xTaskGetTickCount() - playback_started) *
                          portTICK_PERIOD_MS;
    ESP_LOGI(TAG,
             "WAV %s playback selesai: sumber=%u frame, I2S=%u byte, "
             "writes=%u elapsed=%u ms, status=OK.",
             description, (unsigned)source_samples, (unsigned)total_output_bytes,
             (unsigned)write_count, (unsigned)elapsed_ms);
    return true;
}

static bool speaker_play_welcome(void)
{
    size_t wav_size = (size_t)(_binary_welcome_readzio_wav_end -
                               _binary_welcome_readzio_wav_start);
    return speaker_play_embedded_wav("sambutan", _binary_welcome_readzio_wav_start,
                                     wav_size, -1, false);
}

static bool speaker_play_menu1_intro(void)
{
    size_t wav_size = (size_t)(_binary_pengantar_menu1_clean_wav_end -
                               _binary_pengantar_menu1_clean_wav_start);
    return speaker_play_embedded_wav(
        "pengantar Menu 1",
        _binary_pengantar_menu1_clean_wav_start, wav_size, 0, false);
}

static bool speaker_play_ima4_asset(const char *description,
                                    const embedded_ima4_asset_t *asset,
                                    int expected_page,
                                    bool expected_lesson_started)
{
    if (!speaker_menu_context_matches(expected_page,
                                     expected_lesson_started)) {
        ESP_LOGI(TAG, "Playback IMA ADPCM %s dilewati; halaman sudah berubah.",
                 description);
        return true;
    }

    size_t encoded_size = (size_t)(asset->end - asset->start);
    ima4_decoder_t decoder;
    if (!ima4_decoder_init(&decoder, asset->start, encoded_size)) {
        ESP_LOGE(TAG, "Audio IMA ADPCM %s tidak valid (%u byte).",
                 description, (unsigned)encoded_size);
        return false;
    }

    const size_t source_samples = decoder.sample_count;
    const size_t source_chunk_samples = 1024;
    const int64_t target_rms = 4125;
    const int64_t target_peak = 8192;
    static int16_t output[1024 * 4];
    int64_t sum_squares = 0;
    int32_t source_peak = 0;
    size_t total_output_bytes = 0;

    for (size_t i = 0; i < source_samples; ++i) {
        int32_t sample = ima4_decoder_next(&decoder);
        int32_t magnitude = sample < 0 ? -sample : sample;
        if (magnitude > source_peak) source_peak = magnitude;
        sum_squares += (int64_t)sample * sample;
    }
    if (source_peak == 0) {
        ESP_LOGE(TAG, "Audio IMA ADPCM %s senyap; playback dibatalkan.", description);
        return false;
    }
    uint32_t source_rms = (uint32_t)sqrt((double)sum_squares / source_samples);
    if (source_rms == 0) {
        ESP_LOGE(TAG, "RMS audio IMA ADPCM %s tidak valid.", description);
        return false;
    }

    int64_t gain_q15 = target_rms * 32768 / source_rms;
    int64_t peak_limited_gain_q15 = target_peak * 32768 / source_peak;
    if (gain_q15 > peak_limited_gain_q15) gain_q15 = peak_limited_gain_q15;

    ESP_LOGI(TAG,
             "IMA ADPCM %s mono 16 kHz, %u frame, %u byte; "
             "peak=%ld RMS=%u gain Q15=%ld.",
             description, (unsigned)source_samples, (unsigned)encoded_size,
             (long)source_peak, (unsigned)source_rms, (long)gain_q15);
    TickType_t playback_started = xTaskGetTickCount();
    size_t write_count = 0;
    if (!ima4_decoder_init(&decoder, asset->start, encoded_size)) {
        ESP_LOGE(TAG, "Gagal menginisialisasi ulang decoder IMA ADPCM %s.", description);
        return false;
    }

    int32_t current_source = ima4_decoder_next(&decoder);
    for (size_t source_offset = 0; source_offset < source_samples;) {
        if (!speaker_menu_context_matches(expected_page,
                                          expected_lesson_started)) {
            ESP_LOGI(TAG, "Playback IMA ADPCM %s dihentikan; halaman sudah berubah.",
                     description);
            return true;
        }

        size_t sample_count = source_samples - source_offset;
        if (sample_count > source_chunk_samples) {
            sample_count = source_chunk_samples;
        }

        for (size_t i = 0; i < sample_count; ++i) {
            int32_t next_source = current_source;
            if (source_offset + i + 1 < source_samples) {
                next_source = ima4_decoder_next(&decoder);
            }
            int32_t current = (int32_t)((int64_t)current_source * gain_q15 / 32768);
            int32_t next = (int32_t)((int64_t)next_source * gain_q15 / 32768);
            int32_t midpoint = (current + next) / 2;
            size_t output_index = i * 4;
            output[output_index] = (int16_t)current;
            output[output_index + 1] = (int16_t)current;
            output[output_index + 2] = (int16_t)midpoint;
            output[output_index + 3] = (int16_t)midpoint;
            current_source = next_source;
        }

        size_t bytes_expected = sample_count * 4 * sizeof(output[0]);
        size_t bytes_written = 0;
        esp_err_t err = i2s_channel_write(speaker_tx_channel, output,
                                          bytes_expected, &bytes_written,
                                          pdMS_TO_TICKS(1000));
        if (err != ESP_OK || bytes_written != bytes_expected) {
            ESP_LOGE(TAG, "Playback IMA ADPCM %s gagal pada sampel %u: %s (%u/%u byte).",
                     description, (unsigned)source_offset, esp_err_to_name(err),
                     (unsigned)bytes_written, (unsigned)bytes_expected);
            return false;
        }
        total_output_bytes += bytes_written;
        ++write_count;
        source_offset += sample_count;
    }

    uint32_t elapsed_ms = (uint32_t)(xTaskGetTickCount() - playback_started) *
                          portTICK_PERIOD_MS;
    ESP_LOGI(TAG,
             "IMA ADPCM %s playback selesai: sumber=%u frame, I2S=%u byte, "
             "writes=%u elapsed=%u ms, status=OK.",
             description, (unsigned)source_samples, (unsigned)total_output_bytes,
             (unsigned)write_count, (unsigned)elapsed_ms);
    return true;
}

static bool speaker_play_menu1_letter(unsigned letter_index)
{
    static const char *words[26] = {
        "ayam", "bebek", "ceri", "delman", "elang", "foto", "gelang",
        "hadiah", "ikan", "jeruk", "kakek", "langit", "mama", "nenek",
        "obor", "pensil", "quen", "roti", "sosis", "terong", "udang",
        "venus", "website", "x-rey", "yoyo", "zebra"
    };
    if (letter_index >= 26) {
        ESP_LOGE(TAG, "Indeks audio Menu 1 di luar rentang: %u.", letter_index);
        return false;
    }

    char description[32];
    snprintf(description, sizeof(description), "Menu 1 huruf %c (%s)",
             'A' + letter_index, words[letter_index]);
    return speaker_play_ima4_asset(description,
                                    &menu1_audio_assets[letter_index],
                                    0, true);
}

static bool speaker_play_menu2_intro(void)
{
    return speaker_play_ima4_asset("pengantar Menu 2", &menu2_intro_audio,
                                   1, false);
}

static bool speaker_play_menu2_vowel(unsigned vowel_index)
{
    static const char *vowel_names[5] = {
        "A", "I", "U", "E", "O"
    };
    if (vowel_index >= 5) {
        ESP_LOGE(TAG, "Indeks audio vokal Menu 2 di luar rentang: %u.",
                 vowel_index);
        return false;
    }

    char description[32];
    snprintf(description, sizeof(description), "Menu 2 vokal %s",
             vowel_names[vowel_index]);
    return speaker_play_ima4_asset(description,
                                   &menu2_vowel_audio[vowel_index], 1, true);
}

static bool speaker_play_menu3_intro(void)
{
    return speaker_play_ima4_asset("pengantar Menu 3", &menu3_intro_audio,
                                   2, false);
}

static bool speaker_play_menu3_prompt(unsigned vowel_index)
{
    if (vowel_index >= PHONEME_CLASS_COUNT) {
        ESP_LOGE(TAG, "Indeks audio fonem Menu 3 tidak valid: %u.", vowel_index);
        return false;
    }

    char description[32];
    snprintf(description, sizeof(description), "Menu 3 ucapkan %c",
             "AIUEO"[vowel_index]);
    if (!speaker_play_ima4_asset(description, &menu3_prompt_audio[vowel_index],
                                 2, true)) {
        publish_phoneme_result((uint8_t)vowel_index, -1, 0.0f, false);
        return false;
    }
    if (!speaker_menu_context_matches(2, true)) {
        publish_phoneme_result((uint8_t)vowel_index, -1, 0.0f, false);
        return true;
    }

    uint8_t target = (uint8_t)vowel_index;
    if (phoneme_capture_queue == NULL ||
        xQueueSend(phoneme_capture_queue, &target, 0) != pdPASS) {
        ESP_LOGE(TAG, "Antrean perekaman fonem Menu 3 penuh/tidak tersedia.");
        publish_phoneme_result(target, -1, 0.0f, false);
        return false;
    }
    return true;
}

static bool speaker_play_menu3_feedback(bool correct)
{
    return speaker_play_ima4_asset(
        correct ? "jawaban benar Menu 3" : "jawaban kurang tepat Menu 3",
        correct ? &menu3_correct_audio : &menu3_incorrect_audio, 2, true);
}

static bool speaker_play_menu4_intro(void)
{
    return speaker_play_ima4_asset("pengantar Menu 4", &menu4_intro_audio,
                                   3, false);
}

static bool speaker_play_menu4_question(unsigned question_index)
{
    static const char *question_names[5] = {
        "A vs I", "U vs I", "U vs E", "O vs E", "O vs A"
    };
    if (question_index >= 5) {
        ESP_LOGE(TAG, "Indeks audio pertanyaan Menu 4 di luar rentang: %u.",
                 question_index);
        return false;
    }

    char description[40];
    snprintf(description, sizeof(description), "Menu 4 %s",
             question_names[question_index]);
    return speaker_play_ima4_asset(description,
                                   &menu4_question_audio[question_index],
                                   3, true);
}

static bool speaker_play_menu4_feedback(bool correct)
{
    return speaker_play_ima4_asset(
        correct ? "umpan balik benar Menu 4" : "umpan balik salah Menu 4",
        correct ? &menu4_correct_audio : &menu4_incorrect_audio, 3, true);
}

static bool speaker_play_menu5_intro(void)
{
    return speaker_play_ima4_asset("pengantar Menu 5", &menu5_intro_audio,
                                   4, false);
}

static bool speaker_play_menu5_question(unsigned vowel_index)
{
    static const char *vowel_names[5] = {"A", "I", "U", "E", "O"};
    if (vowel_index >= 5) {
        ESP_LOGE(TAG, "Indeks pertanyaan Menu 5 di luar rentang: %u.",
                 vowel_index);
        return false;
    }

    char description[32];
    snprintf(description, sizeof(description), "awan vokal %s",
             vowel_names[vowel_index]);
    return speaker_play_ima4_asset(description,
                                   &menu5_cloud_audio[vowel_index], 4, true);
}

static bool speaker_play_menu5_found(unsigned vowel_index, bool is_final_vowel)
{
    if (vowel_index >= 5) {
        ESP_LOGE(TAG, "Indeks vokal ditemukan Menu 5 di luar rentang: %u.",
                 vowel_index);
        return false;
    }
    if (is_final_vowel) {
        return speaker_play_ima4_asset("penutup awan Menu 5",
                                       &menu5_finish_audio, 4, true);
    }
    if (!speaker_play_ima4_asset("ketemu", &menu5_found_audio, 4, true)) {
        return false;
    }
    return speaker_play_menu5_question(vowel_index + 1);
}

static bool speaker_play_menu5_incorrect(void)
{
    return speaker_play_ima4_asset("umpan balik salah Menu 5",
                                   &menu4_incorrect_audio, 4, true);
}

static void speaker_beep_task(void *arg)
{
    (void)arg;
    static int16_t samples[BEEP_BUFFER_SAMPLES * SPEAKER_CHANNELS];

    while (1) {
        speaker_request_t request;
        if (xQueueReceive(speaker_beep_queue, &request, portMAX_DELAY) != pdPASS) {
            continue;
        }

        if (request == SPEAKER_REQUEST_WELCOME) {
            (void)speaker_play_welcome();
            continue;
        } else if (request == SPEAKER_REQUEST_MENU1_INTRO) {
            (void)speaker_play_menu1_intro();
            continue;
        } else if (request == SPEAKER_REQUEST_MENU2_INTRO) {
            (void)speaker_play_menu2_intro();
            continue;
        } else if (request == SPEAKER_REQUEST_MENU3_INTRO) {
            (void)speaker_play_menu3_intro();
            continue;
        } else if (request == SPEAKER_REQUEST_MENU3_CORRECT ||
                   request == SPEAKER_REQUEST_MENU3_INCORRECT) {
            (void)speaker_play_menu3_feedback(
                request == SPEAKER_REQUEST_MENU3_CORRECT);
            continue;
        } else if (request == SPEAKER_REQUEST_MENU4_INTRO) {
            (void)speaker_play_menu4_intro();
            continue;
        } else if (request == SPEAKER_REQUEST_MENU4_CORRECT ||
                   request == SPEAKER_REQUEST_MENU4_INCORRECT) {
            (void)speaker_play_menu4_feedback(
                request == SPEAKER_REQUEST_MENU4_CORRECT);
            continue;
        } else if (request == SPEAKER_REQUEST_MENU5_INTRO) {
            (void)speaker_play_menu5_intro();
            continue;
        } else if (request == SPEAKER_REQUEST_MENU5_INCORRECT) {
            (void)speaker_play_menu5_incorrect();
            continue;
        } else if (request >= SPEAKER_REQUEST_MENU1_LETTER_A &&
                   request < SPEAKER_REQUEST_MENU1_LETTER_A + 26) {
            (void)speaker_play_menu1_letter(
                (unsigned)(request - SPEAKER_REQUEST_MENU1_LETTER_A));
            continue;
        } else if (request >= SPEAKER_REQUEST_MENU2_VOWEL_A &&
                   request < SPEAKER_REQUEST_MENU2_VOWEL_A + 5) {
            (void)speaker_play_menu2_vowel(
                (unsigned)(request - SPEAKER_REQUEST_MENU2_VOWEL_A));
            continue;
        } else if (request >= SPEAKER_REQUEST_MENU3_PROMPT_A &&
                   request < SPEAKER_REQUEST_MENU3_PROMPT_A +
                                 PHONEME_CLASS_COUNT) {
            (void)speaker_play_menu3_prompt(
                (unsigned)(request - SPEAKER_REQUEST_MENU3_PROMPT_A));
            continue;
        } else if (request >= SPEAKER_REQUEST_MENU4_QUESTION_A_VS_I &&
                   request < SPEAKER_REQUEST_MENU4_QUESTION_A_VS_I + 5) {
            (void)speaker_play_menu4_question(
                (unsigned)(request - SPEAKER_REQUEST_MENU4_QUESTION_A_VS_I));
            continue;
        } else if (request >= SPEAKER_REQUEST_MENU5_QUESTION_A &&
                   request < SPEAKER_REQUEST_MENU5_QUESTION_A + 5) {
            (void)speaker_play_menu5_question(
                (unsigned)(request - SPEAKER_REQUEST_MENU5_QUESTION_A));
            continue;
        } else if ((request >= SPEAKER_REQUEST_MENU5_FOUND_A &&
                    request < SPEAKER_REQUEST_MENU5_FOUND_A + 5) ||
                   (request >= SPEAKER_REQUEST_MENU5_FOUND_A + 8 &&
                    request < SPEAKER_REQUEST_MENU5_FOUND_A + 13)) {
            bool is_final_vowel =
                request >= SPEAKER_REQUEST_MENU5_FOUND_A + 8;
            unsigned vowel_index = is_final_vowel
                ? (unsigned)(request - SPEAKER_REQUEST_MENU5_FOUND_A - 8)
                : (unsigned)(request - SPEAKER_REQUEST_MENU5_FOUND_A);
            (void)speaker_play_menu5_found(vowel_index, is_final_vowel);
            continue;
        }

        for (size_t i = 0; i < BEEP_BUFFER_SAMPLES; ++i) {
            int16_t output = 0;
            if (i < BEEP_TONE_SAMPLES) {
                float phase = 6.28318530718f * 1800.0f * (float)i /
                              (float)SPEAKER_SAMPLE_RATE;
                int32_t sample = (int32_t)(sinf(phase) * (float)BEEP_AMPLITUDE);
                int remaining = BEEP_TONE_SAMPLES - 1 - (int)i;
                int fade = 32768;
                if (i < 96) {
                    fade = (int)i * 32768 / 96;
                }
                if (remaining < 160) {
                    int fade_out = remaining > 0
                        ? remaining * 32768 / 160 : 0;
                    if (fade_out < fade) fade = fade_out;
                }
                output = (int16_t)(sample * fade / 32768);
            }
            samples[i * SPEAKER_CHANNELS] = output;
            samples[i * SPEAKER_CHANNELS + 1] = output;
        }
        size_t bytes_written = 0;
        esp_err_t err = i2s_channel_write(speaker_tx_channel, samples,
                                          BEEP_BUFFER_SAMPLES * SPEAKER_CHANNELS *
                                              sizeof(samples[0]),
                                          &bytes_written,
                                          pdMS_TO_TICKS(500));
        size_t bytes_expected = BEEP_BUFFER_SAMPLES * SPEAKER_CHANNELS *
                                sizeof(samples[0]);
        if (err != ESP_OK || bytes_written != bytes_expected) {
            ESP_LOGW(TAG, "Beep speaker gagal: %s (%u/%u byte)",
                     esp_err_to_name(err), (unsigned)bytes_written,
                     (unsigned)bytes_expected);
        } else {
            ESP_LOGI(TAG, "Touch beep terkirim lewat I2S: %u/%u byte, status=OK.",
                     (unsigned)bytes_written,
                     (unsigned)bytes_expected);
        }
    }
}

static bool speaker_init(void)
{
    if (!speaker_enable_gpio_ready) {
        ESP_LOGE(TAG, "Speaker tetap disable: GPIO enable belum siap.");
        return false;
    }

    esp_err_t err;
    i2s_chan_config_t channel_config =
        I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    channel_config.dma_desc_num = 8;
    channel_config.dma_frame_num = 512;
    channel_config.auto_clear_after_cb = true;
    err = i2s_new_channel(&channel_config, &speaker_tx_channel, NULL);
    if (err != ESP_OK) goto fail;

    i2s_std_config_t standard_config = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SPEAKER_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = SPEAKER_PIN_BCLK,
            .ws = SPEAKER_PIN_WS,
            .dout = SPEAKER_PIN_DOUT,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {0}
        }
    };
    err = i2s_channel_init_std_mode(speaker_tx_channel, &standard_config);
    if (err != ESP_OK) goto fail;
    err = i2s_channel_enable(speaker_tx_channel);
    if (err != ESP_OK) goto fail;

    {
        static int16_t silence[512 * SPEAKER_CHANNELS];
        const size_t silence_bytes = sizeof(silence);
        for (int i = 0; i < 8; ++i) {
            size_t bytes_written = 0;
            err = i2s_channel_write(speaker_tx_channel, silence, silence_bytes,
                                   &bytes_written, pdMS_TO_TICKS(500));
            if (err != ESP_OK || bytes_written != silence_bytes) {
                if (err == ESP_OK) err = ESP_FAIL;
                ESP_LOGE(TAG,
                         "Priming I2S senyap gagal (%u/%u byte): %s.",
                         (unsigned)bytes_written, (unsigned)silence_bytes,
                         esp_err_to_name(err));
                goto fail;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(150));
    }

    speaker_beep_queue = xQueueCreate(8, sizeof(speaker_request_t));
    if (speaker_beep_queue == NULL) {
        err = ESP_ERR_NO_MEM;
        goto fail;
    }
    if (xTaskCreate(speaker_beep_task, "speaker_beep", 3072, NULL, 4,
                    &speaker_beep_task_handle) != pdPASS) {
        err = ESP_ERR_NO_MEM;
        goto fail;
    }

    gpio_set_level(SPEAKER_PIN_ENABLE, 1);
    ESP_LOGI(TAG, "MAX98357A I2S siap (BCLK=15 WS=16 DOUT=17 EN=38).");
    return true;

fail:
    ESP_LOGW(TAG, "Speaker I2S tidak tersedia: %s", esp_err_to_name(err));
    if (speaker_beep_task_handle != NULL) {
        vTaskDelete(speaker_beep_task_handle);
        speaker_beep_task_handle = NULL;
    }
    if (speaker_tx_channel != NULL) {
        i2s_channel_disable(speaker_tx_channel);
        i2s_del_channel(speaker_tx_channel);
        speaker_tx_channel = NULL;
    }
    if (speaker_beep_queue != NULL) {
        vQueueDelete(speaker_beep_queue);
        speaker_beep_queue = NULL;
    }
    gpio_set_level(SPEAKER_PIN_ENABLE, 0);
    return false;
}

static bool speaker_enable_prepare(void)
{
    gpio_config_t enable_gpio = {
        .pin_bit_mask = 1ULL << SPEAKER_PIN_ENABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    esp_err_t err = gpio_config(&enable_gpio);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Gagal mengatur GPIO enable speaker LOW: %s",
                 esp_err_to_name(err));
        return false;
    }
    err = gpio_set_level(SPEAKER_PIN_ENABLE, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Gagal menahan MAX98357A dalam kondisi disable: %s",
                 esp_err_to_name(err));
        return false;
    }

    speaker_enable_gpio_ready = true;
    ESP_LOGI(TAG, "GPIO enable speaker ditahan LOW selama startup.");
    return true;
}

static void touch_task(void *arg)
{
    (void)arg;
    bool pressed = false;
    bool release_pending = false;
    TickType_t press_started = 0;
    TickType_t release_started = 0;
    TickType_t last_idle_log = 0;
    TickType_t last_invalid_log = 0;
    int last_x = 0, last_y = 0;
    int sum_x = 0, sum_y = 0, samples = 0;
    uint32_t sum_raw_x = 0, sum_raw_y = 0;

    ESP_LOGI(TAG, "Touch polling aktif (IRQ GPIO%d awal=%d).",
             TOUCH_PIN_IRQ, gpio_get_level(TOUCH_PIN_IRQ));

    while (1) {
        TickType_t now = xTaskGetTickCount();
        bool irq_low = gpio_get_level(TOUCH_PIN_IRQ) == 0;
        int x = 0, y = 0;
        uint16_t raw_x = 0, raw_y = 0;
        bool point_valid = touch_read_point(&x, &y, &raw_x, &raw_y);

        if (point_valid) {
            if (!pressed) {
                if (samples == 0) {
                    press_started = now;
                    sum_x = 0;
                    sum_y = 0;
                    sum_raw_x = 0;
                    sum_raw_y = 0;
                } else if (abs(x - last_x) > 60 || abs(y - last_y) > 60) {
                    press_started = now;
                    sum_x = 0;
                    sum_y = 0;
                    sum_raw_x = 0;
                    sum_raw_y = 0;
                    samples = 0;
                }
                last_x = x;
                last_y = y;
                sum_x += x;
                sum_y += y;
                sum_raw_x += raw_x;
                sum_raw_y += raw_y;
                ++samples;

                if (samples >= 2 &&
                    now - press_started >= pdMS_TO_TICKS(TOUCH_PRESS_DEBOUNCE_MS)) {
                    pressed = true;
                    release_pending = false;
                    int screen_x = sum_x / samples;
                    int screen_y = sum_y / samples;
                    touch_event_t event = {
                        .x = screen_x,
                        .y = screen_y,
                        .raw_x = (uint16_t)(sum_raw_x / samples),
                        .raw_y = (uint16_t)(sum_raw_y / samples)
                    };
                    if (ui_event_queue != NULL) {
                        if (xQueueSend(ui_event_queue, &event, 0) != pdPASS)
                            ESP_LOGW(TAG, "Antrean UI penuh; event touch dilewati.");
                    } else {
                        bool handled = handle_touch(screen_x, screen_y);
                        ESP_LOGI(TAG,
                                 "Touch raw=(%u,%u) screen=(%d,%d) target=%s page=%d.",
                                 (unsigned)event.raw_x, (unsigned)event.raw_y,
                                 screen_x, screen_y, handled ? "yes" : "no", active_page);
                    }
                }
            } else {
                release_pending = false;
            }
        } else if (pressed) {
            if (!release_pending) {
                release_pending = true;
                release_started = now;
            } else if (now - release_started >= pdMS_TO_TICKS(TOUCH_RELEASE_DEBOUNCE_MS)) {
                pressed = false;
                release_pending = false;
                samples = 0;
            }
        } else {
            samples = 0;
            if (now - last_idle_log >= pdMS_TO_TICKS(TOUCH_DIAGNOSTIC_INTERVAL_MS)) {
                ESP_LOGI(TAG,
                         "Touch probe IRQ %s raw=(%u,%u) valid=%d.",
                         irq_low ? "LOW" : "HIGH",
                         (unsigned)raw_x, (unsigned)raw_y, point_valid);
                last_idle_log = now;
            }
            if (irq_low && now - last_invalid_log >= pdMS_TO_TICKS(1000)) {
                ESP_LOGW(TAG, "Touch IRQ LOW tetapi data XPT2046 tidak valid (raw=%u,%u).",
                         (unsigned)raw_x, (unsigned)raw_y);
                last_invalid_log = now;
            }
        }

        if (ui_event_queue == NULL) process_phoneme_result();
        vTaskDelay(pdMS_TO_TICKS(TOUCH_POLL_INTERVAL_MS));
    }
}

#if CONFIG_READZIO_ENABLE_NETWORK
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "Menghubungkan ke Wi-Fi..."); esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (retry_count < WIFI_MAX_RETRY) { retry_count++; ESP_LOGW(TAG, "Wi-Fi terputus, mencoba lagi (%d/%d)", retry_count, WIFI_MAX_RETRY); esp_wifi_connect(); }
        else { xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT); }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Wi-Fi terhubung. IP ESP32: " IPSTR, IP2STR(&event->ip_info.ip));
        retry_count = 0; xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static bool wifi_init_sta(void)
{
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
    wifi_config_t wifi_config = {0};
    snprintf((char *)wifi_config.sta.ssid, sizeof(wifi_config.sta.ssid), "%s", WIFI_SSID);
    snprintf((char *)wifi_config.sta.password, sizeof(wifi_config.sta.password), "%s", WIFI_PASSWORD);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true; wifi_config.sta.pmf_cfg.required = false;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT) { ESP_LOGI(TAG, "Wi-Fi siap."); return true; }
    ESP_LOGE(TAG, "Gagal terhubung ke Wi-Fi setelah beberapa percobaan."); return false;
}

static void test_backend(void)
{
    esp_http_client_config_t config = { .url = BACKEND_HEALTH_URL, .method = HTTP_METHOD_GET, .timeout_ms = 10000 };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) { ESP_LOGE(TAG, "Gagal membuat HTTP client"); return; }
    ESP_LOGI(TAG, "Menguji endpoint backend Readzio: %s", BACKEND_HEALTH_URL);
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        int status = esp_http_client_get_status_code(client);
        ESP_LOGI(TAG, "Backend HTTP status: %d", status);
        if (status >= 200 && status < 300) ESP_LOGI(TAG, "Koneksi backend berhasil.");
        else ESP_LOGW(TAG, "Backend merespons status HTTP %d", status);
    } else ESP_LOGE(TAG, "Request backend gagal: %s", esp_err_to_name(err));
    esp_http_client_cleanup(client);
}
#endif

static void log_heap_status(void)
{
    const size_t internal_total = heap_caps_get_total_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    const size_t internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    const size_t internal_min_free = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    const size_t internal_largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    const size_t psram_total = heap_caps_get_total_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    const size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    const size_t psram_min_free = heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    const size_t psram_largest = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    ESP_LOGI(TAG, "Heap internal: free %u/%u KiB (%u%%), min-free %u KiB, largest block %u KiB",
             (unsigned)(internal_free / 1024), (unsigned)(internal_total / 1024),
             internal_total ? (unsigned)((internal_free * 100) / internal_total) : 0,
             (unsigned)(internal_min_free / 1024), (unsigned)(internal_largest / 1024));
    ESP_LOGI(TAG, "Heap PSRAM: free %u/%u KiB (%u%%), min-free %u KiB, largest block %u KiB",
             (unsigned)(psram_free / 1024), (unsigned)(psram_total / 1024),
             psram_total ? (unsigned)((psram_free * 100) / psram_total) : 0,
             (unsigned)(psram_min_free / 1024), (unsigned)(psram_largest / 1024));
}

void app_main(void)
{
    (void)speaker_enable_prepare();
    esp_err_t ret = activity_log_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Penyimpanan riwayat tidak tersedia: %s; "
                 "aplikasi tetap berjalan tanpa pencatatan.",
                 esp_err_to_name(ret));
    }
    ESP_LOGI(TAG,"Readzio hardware test: TFT + touch + speaker; backend intentionally disabled.");
    spi_shared_bus_mutex = xSemaphoreCreateMutex();
    if (spi_shared_bus_mutex == NULL) {
        ESP_LOGE(TAG, "Gagal membuat mutex SPI bersama; hentikan agar bus tidak rusak.");
        return;
    }
    lcd_init();
    draw_startup_splash();
    render_ui();
    touch_init();
    if (!speaker_init()) ESP_LOGW(TAG, "UI tetap berjalan tanpa beep speaker.");
    mic_test_queue = xQueueCreate(1, sizeof(uint8_t));
    if (mic_test_queue != NULL &&
        xTaskCreate(microphone_test_task, "mic_test", 4096, NULL, 3, NULL) != pdPASS) {
        vQueueDelete(mic_test_queue);
        mic_test_queue = NULL;
        ESP_LOGW(TAG, "Task tes INMP441 gagal dibuat; tes mikrofon tidak tersedia.");
    } else if (mic_test_queue == NULL) {
        ESP_LOGW(TAG, "Antrean tes INMP441 gagal dibuat; tes mikrofon tidak tersedia.");
    }
    phoneme_capture_queue = xQueueCreate(1, sizeof(uint8_t));
    phoneme_result_queue = xQueueCreate(1, sizeof(phoneme_result_event_t));
    if (phoneme_capture_queue == NULL || phoneme_result_queue == NULL) {
        ESP_LOGE(TAG, "Antrean pengenalan Menu 3 gagal dibuat.");
        if (phoneme_capture_queue != NULL) vQueueDelete(phoneme_capture_queue);
        if (phoneme_result_queue != NULL) vQueueDelete(phoneme_result_queue);
        phoneme_capture_queue = NULL;
        phoneme_result_queue = NULL;
    } else if (xTaskCreate(phoneme_capture_task, "phoneme_capture", 8192,
                           NULL, 3, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Task pengenalan Menu 3 gagal dibuat.");
        vQueueDelete(phoneme_capture_queue);
        vQueueDelete(phoneme_result_queue);
        phoneme_capture_queue = NULL;
        phoneme_result_queue = NULL;
    }
    ui_event_queue = xQueueCreate(4, sizeof(touch_event_t));
    if (ui_event_queue != NULL &&
        xTaskCreate(ui_event_task, "ui_events", 4096, NULL, 2, NULL) != pdPASS) {
        vQueueDelete(ui_event_queue);
        ui_event_queue = NULL;
        ESP_LOGW(TAG, "Task UI event gagal dibuat; fallback ke task touch.");
    }
    if (xTaskCreate(touch_task, "touch_ui", 4096, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Gagal membuat task touch; hentikan inisialisasi UI.");
        return;
    }
    request_welcome_playback();
    /* Networking is optional so hardware tests do not wait on Wi-Fi. */
#if CONFIG_READZIO_ENABLE_NETWORK
    if (wifi_init_sta()) test_backend();
    else ESP_LOGW(TAG, "Wi-Fi belum terhubung; UI tetap berjalan.");
#else
    ESP_LOGI(TAG, "Mode tes komponen aktif: tidak ada koneksi Wi-Fi/backend.");
#endif
    log_heap_status();
    while(1) vTaskDelay(pdMS_TO_TICKS(1000));
}
