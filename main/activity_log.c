#include "activity_log.h"

#include <inttypes.h>
#include <fcntl.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "driver/usb_serial_jtag.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

#define ACTIVITY_NAMESPACE "readzio_log"
#define ACTIVITY_QUEUE_LENGTH 16
#define ACTIVITY_TASK_STACK 6144
#define SERIAL_LINE_MAX 40
#define RECORD_MAGIC 0x415A494FUL
#define RECORD_VERSION 1

typedef struct {
    uint32_t magic;
    uint32_t sequence;
    uint64_t timestamp_ms;
    uint64_t session_id;
    uint32_t attempt;
    uint8_t activity;
    char target;
    uint8_t correct;
    uint8_t version;
    uint32_t crc;
} activity_record_t;

typedef struct {
    activity_log_kind_t activity;
    char target;
    bool correct;
    uint32_t attempt;
    uint64_t timestamp_ms;
    uint64_t session_id;
} activity_request_t;

static const char *TAG = "activity_log";
static nvs_handle_t s_nvs_handle;
static QueueHandle_t s_activity_queue;
static uint64_t s_session_id;
static uint32_t s_next_sequence = 1;
static bool s_storage_writable = true;
static uint32_t s_queue_dropped;
static portMUX_TYPE s_drop_lock = portMUX_INITIALIZER_UNLOCKED;
static int s_uart_fd = -1;

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t length)
{
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
        }
    }
    return crc;
}

static uint32_t record_crc(const activity_record_t *record)
{
    return crc32_update(0xFFFFFFFFU, (const uint8_t *)record,
                        offsetof(activity_record_t, crc)) ^ 0xFFFFFFFFU;
}

static bool record_valid(const activity_record_t *record)
{
    return record->magic == RECORD_MAGIC &&
           record->version == RECORD_VERSION &&
           (record->activity == ACTIVITY_LOG_VOWEL_CHOICE ||
            record->activity == ACTIVITY_LOG_CLOUD_SEARCH) &&
           record->target >= 'A' && record->target <= 'Z' &&
           record->correct <= 1 &&
           record->attempt > 0 &&
           record->crc == record_crc(record);
}

static bool record_key_sequence(const char *key, uint32_t *sequence)
{
    if (key[0] != 'r') {
        return false;
    }
    if (strlen(key) != 9) {
        return false;
    }

    char *end = NULL;
    unsigned long value = strtoul(key + 1, &end, 16);
    if (end == key + 1 || *end != '\0' || value > UINT32_MAX) {
        return false;
    }
    *sequence = (uint32_t)value;
    return true;
}

static esp_err_t scan_max_sequence(uint32_t *max_sequence)
{
    nvs_iterator_t iterator = NULL;
    esp_err_t err = nvs_entry_find_in_handle(s_nvs_handle, NVS_TYPE_BLOB,
                                              &iterator);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        *max_sequence = 0;
        return ESP_OK;
    }
    if (err != ESP_OK) {
        return err;
    }

    *max_sequence = 0;
    while (err == ESP_OK) {
        nvs_entry_info_t info = {0};
        err = nvs_entry_info(iterator, &info);
        if (err != ESP_OK) {
            break;
        }
        if (info.key[0] == 'r') {
            uint32_t key_sequence = 0;
            activity_record_t record;
            size_t record_size = sizeof(record);
            if (!record_key_sequence(info.key, &key_sequence)) {
                err = ESP_ERR_INVALID_STATE;
                break;
            }
            err = nvs_get_blob(s_nvs_handle, info.key, &record, &record_size);
            if (err != ESP_OK) {
                break;
            }
            if (record_size != sizeof(record) || !record_valid(&record) ||
                record.sequence != key_sequence) {
                err = ESP_ERR_INVALID_CRC;
                break;
            }
            if (record.sequence > *max_sequence) {
                *max_sequence = record.sequence;
            }
        }
        err = nvs_entry_next(&iterator);
    }
    nvs_release_iterator(iterator);

    return err == ESP_ERR_NVS_NOT_FOUND ? ESP_OK : err;
}

static void report_queue_drops(void)
{
    uint32_t dropped = 0;
    portENTER_CRITICAL(&s_drop_lock);
    dropped = s_queue_dropped;
    s_queue_dropped = 0;
    portEXIT_CRITICAL(&s_drop_lock);
    if (dropped > 0) {
        ESP_LOGE(TAG, "%u aktivitas tidak masuk antrean; periksa beban task.",
                 (unsigned)dropped);
    }
}

static void process_activity(const activity_request_t *request)
{
    if (!s_storage_writable) {
        return;
    }

    if (s_next_sequence == 0) {
        s_storage_writable = false;
        ESP_LOGE(TAG, "Nomor sequence habis; riwayat lama tetap dipertahankan.");
        return;
    }

    activity_record_t record = {0};
    record.magic = RECORD_MAGIC;
    record.sequence = s_next_sequence;
    record.timestamp_ms = request->timestamp_ms;
    record.session_id = request->session_id;
    record.attempt = request->attempt;
    record.activity = (uint8_t)request->activity;
    record.target = request->target;
    record.correct = request->correct ? 1 : 0;
    record.version = RECORD_VERSION;
    record.crc = record_crc(&record);

    char key[10];
    (void)snprintf(key, sizeof(key), "r%08" PRIx32, record.sequence);
    esp_err_t err = nvs_set_blob(s_nvs_handle, key, &record, sizeof(record));
    if (err == ESP_OK) {
        err = nvs_commit(s_nvs_handle);
    }
    if (err != ESP_OK) {
        s_storage_writable = false;
        ESP_LOGE(TAG,
                 "Aktivitas sequence=%" PRIu32 " gagal disimpan (%s); "
                 "penulisan dihentikan, riwayat lama tidak dihapus.",
                 record.sequence, esp_err_to_name(err));
        return;
    }

    if (s_uart_fd >= 0) {
        char payload[112];
        int payload_length = snprintf(
            payload, sizeof(payload),
            "READZIO_LIVE,%" PRIu32 ",%" PRIu64 ",%016" PRIx64
            ",%s,%c,%s,%" PRIu32,
            record.sequence, record.timestamp_ms, record.session_id,
            record.activity == ACTIVITY_LOG_VOWEL_CHOICE
                ? "vowel_choice" : "cloud_search",
            record.target, record.correct ? "true" : "false",
            record.attempt);
        if (payload_length <= 0 || (size_t)payload_length >= sizeof(payload)) {
            ESP_LOGE(TAG, "Record tersimpan tetapi gagal diformat untuk COM3.");
        } else {
            uint32_t crc = crc32_update(0xFFFFFFFFU,
                                        (const uint8_t *)payload,
                                        (size_t)payload_length) ^ 0xFFFFFFFFU;
            char line[128];
            int line_length = snprintf(line, sizeof(line), "%s,%08" PRIx32 "\n",
                                       payload, crc);
            if (line_length <= 0 || (size_t)line_length >= sizeof(line) ||
                write(s_uart_fd, line, (size_t)line_length) !=
                    (ssize_t)line_length) {
                ESP_LOGE(TAG,
                         "Record sequence=%" PRIu32
                         " tersimpan tetapi gagal dikirim live ke COM3.",
                         record.sequence);
            }
        }
    }

    if (s_next_sequence == UINT32_MAX) {
        s_next_sequence = 0;
    } else {
        ++s_next_sequence;
    }
}

typedef struct {
    activity_record_t *records;
    size_t count;
    size_t capacity;
} record_list_t;

static esp_err_t append_record(record_list_t *list,
                               const activity_record_t *record)
{
    if (list->count == list->capacity) {
        size_t next_capacity = list->capacity == 0 ? 16 : list->capacity * 2;
        if (next_capacity > 1024) {
            return ESP_ERR_NO_MEM;
        }
        activity_record_t *records = realloc(
            list->records, next_capacity * sizeof(*list->records));
        if (records == NULL) {
            return ESP_ERR_NO_MEM;
        }
        list->records = records;
        list->capacity = next_capacity;
    }
    list->records[list->count++] = *record;
    return ESP_OK;
}

static int compare_records(const void *left, const void *right)
{
    const activity_record_t *a = left;
    const activity_record_t *b = right;
    if (a->sequence < b->sequence) return -1;
    if (a->sequence > b->sequence) return 1;
    return 0;
}

static esp_err_t load_records(record_list_t *list)
{
    nvs_iterator_t iterator = NULL;
    esp_err_t err = nvs_entry_find_in_handle(s_nvs_handle, NVS_TYPE_BLOB,
                                              &iterator);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    if (err != ESP_OK) {
        return err;
    }

    while (err == ESP_OK) {
        nvs_entry_info_t info = {0};
        err = nvs_entry_info(iterator, &info);
        if (err != ESP_OK) {
            break;
        }
        if (info.key[0] == 'r') {
            uint32_t key_sequence = 0;
            activity_record_t record;
            size_t record_size = sizeof(record);
            if (!record_key_sequence(info.key, &key_sequence)) {
                err = ESP_ERR_INVALID_STATE;
                break;
            }
            err = nvs_get_blob(s_nvs_handle, info.key, &record, &record_size);
            if (err != ESP_OK) {
                break;
            }
            if (record_size != sizeof(record) || !record_valid(&record) ||
                record.sequence != key_sequence) {
                err = ESP_ERR_INVALID_CRC;
                break;
            }
            err = append_record(list, &record);
            if (err != ESP_OK) {
                break;
            }
        }
        err = nvs_entry_next(&iterator);
    }
    nvs_release_iterator(iterator);
    if (err != ESP_ERR_NVS_NOT_FOUND) {
        return err;
    }

    if (list->count > 1) {
        qsort(list->records, list->count, sizeof(*list->records), compare_records);
    }
    for (size_t i = 1; i < list->count; ++i) {
        if (list->records[i - 1].sequence == list->records[i].sequence) {
            return ESP_ERR_INVALID_STATE;
        }
    }
    return ESP_OK;
}

typedef enum {
    EXPORT_USB_SERIAL_JTAG,
    EXPORT_UART0
} export_transport_t;

static bool export_send_line(export_transport_t transport, const char *line,
                             size_t length)
{
    int written = transport == EXPORT_USB_SERIAL_JTAG
        ? usb_serial_jtag_write_bytes(line, length, pdMS_TO_TICKS(1000))
        : s_uart_fd >= 0 ? (int)write(s_uart_fd, line, length) : -1;
    return written == (int)length;
}

static void send_export_error(export_transport_t transport, const char *nonce,
                              const char *code)
{
    char line[80];
    int length = snprintf(line, sizeof(line), "READZIO_ERROR,%s,%s\n",
                          nonce, code);
    if (length > 0 && (size_t)length < sizeof(line)) {
        (void)export_send_line(transport, line, (size_t)length);
    }
}

static void export_records(const char *nonce, export_transport_t transport)
{
    activity_request_t pending_request;
    while (xQueueReceive(s_activity_queue, &pending_request, 0) == pdPASS) {
        process_activity(&pending_request);
    }
    report_queue_drops();

    record_list_t list = {0};
    esp_err_t err = load_records(&list);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Ekspor dibatalkan karena riwayat gagal dibaca: %s",
                 esp_err_to_name(err));
        send_export_error(transport, nonce, "storage_read");
        free(list.records);
        return;
    }

    char line[128];
    int length = snprintf(line, sizeof(line),
                          "READZIO_BEGIN,%s,1,%u\n", nonce,
                          (unsigned)list.count);
    if (length <= 0 || (size_t)length >= sizeof(line) ||
        !export_send_line(transport, line, (size_t)length)) {
        ESP_LOGE(TAG, "Gagal mengirim awal ekspor melalui serial.");
        free(list.records);
        return;
    }

    uint32_t crc = 0xFFFFFFFFU;
    for (size_t i = 0; i < list.count; ++i) {
        const activity_record_t *record = &list.records[i];
        const char *activity = record->activity == ACTIVITY_LOG_VOWEL_CHOICE
            ? "vowel_choice" : "cloud_search";
        length = snprintf(
            line, sizeof(line),
            "READZIO_DATA,%s,%" PRIu32 ",%" PRIu64 ",%016" PRIx64
            ",%s,%c,%s,%" PRIu32 "\n",
            nonce, record->sequence, record->timestamp_ms, record->session_id,
            activity, record->target, record->correct ? "true" : "false",
            record->attempt);
        if (length <= 0 || (size_t)length >= sizeof(line)) {
            ESP_LOGE(TAG, "Baris ekspor terlalu panjang; transfer dihentikan.");
            free(list.records);
            return;
        }
        crc = crc32_update(crc, (const uint8_t *)line, (size_t)length);
        if (!export_send_line(transport, line, (size_t)length)) {
            ESP_LOGE(TAG, "Ekspor terputus saat mengirim sequence=%" PRIu32 ".",
                     record->sequence);
            free(list.records);
            return;
        }
    }

    crc ^= 0xFFFFFFFFU;
    length = snprintf(line, sizeof(line),
                      "READZIO_END,%s,%u,%08" PRIx32 "\n",
                      nonce, (unsigned)list.count, crc);
    if (length <= 0 || (size_t)length >= sizeof(line) ||
        !export_send_line(transport, line, (size_t)length)) {
        ESP_LOGE(TAG, "Gagal mengirim penanda akhir ekspor.");
        free(list.records);
        return;
    }
    if (transport == EXPORT_USB_SERIAL_JTAG) {
        err = usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(3000));
    } else {
        err = ESP_OK;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "USB Serial/JTAG belum menyelesaikan transfer: %s",
                 esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Ekspor selesai: %u record; data flash tetap tersimpan.",
                 (unsigned)list.count);
    }
    free(list.records);
}

static bool valid_nonce(const char *nonce)
{
    if (strlen(nonce) != 16) {
        return false;
    }
    for (size_t i = 0; i < 16; ++i) {
        char c = nonce[i];
        if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') ||
              (c >= 'A' && c <= 'F'))) {
            return false;
        }
    }
    return true;
}

static void handle_serial_command(const char *command, const char *prefix,
                                  export_transport_t transport,
                                  char last_nonce[17])
{
    size_t prefix_length = strlen(prefix);
    if (strncmp(command, prefix, prefix_length) != 0) {
        return;
    }
    const char *nonce = command + prefix_length;
    if (!valid_nonce(nonce)) {
        ESP_LOGW(TAG, "Perintah ekspor ditolak: nonce tidak valid.");
        return;
    }
    if (strcmp(nonce, last_nonce) == 0) {
        return;
    }
    memcpy(last_nonce, nonce, 16);
    last_nonce[16] = '\0';
    export_records(nonce, transport);
}

static void activity_log_task(void *arg)
{
    (void)arg;
    s_uart_fd = open("/dev/uart/0", O_RDWR | O_NONBLOCK);
    if (s_uart_fd < 0) {
        ESP_LOGE(TAG, "UART0 COM3 tidak tersedia untuk progres live.");
    }

    usb_serial_jtag_driver_config_t usb_config = {
        .tx_buffer_size = 2048,
        .rx_buffer_size = 2048
    };
    esp_err_t usb_err = usb_serial_jtag_driver_install(&usb_config);
    bool usb_ready = usb_err == ESP_OK;
    if (!usb_ready) {
        ESP_LOGE(TAG, "USB Serial/JTAG ekspor tidak tersedia: %s",
                 esp_err_to_name(usb_err));
    } else {
        static const char ready[] = "READZIO_READY,1\n";
        (void)usb_serial_jtag_write_bytes(ready, sizeof(ready) - 1,
                                          pdMS_TO_TICKS(1000));
    }

    char usb_command[SERIAL_LINE_MAX];
    size_t usb_command_length = 0;
    bool discard_usb_command = false;
    char uart_command[SERIAL_LINE_MAX];
    size_t uart_command_length = 0;
    bool discard_uart_command = false;
    char last_nonce[17] = {0};
    uint8_t input[64];
    uint8_t uart_input[64];

    while (true) {
        if (usb_ready) {
            int bytes_read = usb_serial_jtag_read_bytes(input, sizeof(input), 0);
            for (int i = 0; i < bytes_read; ++i) {
                char value = (char)input[i];
                if (value == '\r') {
                    continue;
                }
                if (value == '\n') {
                    if (!discard_usb_command && usb_command_length > 0) {
                        usb_command[usb_command_length] = '\0';
                        handle_serial_command(usb_command, "EXPORT,",
                                              EXPORT_USB_SERIAL_JTAG,
                                              last_nonce);
                    }
                    usb_command_length = 0;
                    discard_usb_command = false;
                    continue;
                }
                if (!discard_usb_command) {
                    if (usb_command_length + 1 < sizeof(usb_command)) {
                        usb_command[usb_command_length++] = value;
                    } else {
                        usb_command_length = 0;
                        discard_usb_command = true;
                    }
                }
            }
        }

        if (s_uart_fd >= 0) {
            ssize_t uart_bytes = read(s_uart_fd, uart_input,
                                      sizeof(uart_input));
            for (ssize_t i = 0; i < uart_bytes; ++i) {
                char value = (char)uart_input[i];
                if (value == '\r') {
                    continue;
                }
                if (value == '\n') {
                    if (!discard_uart_command && uart_command_length > 0) {
                        uart_command[uart_command_length] = '\0';
                        handle_serial_command(uart_command, "SYNC,",
                                              EXPORT_UART0, last_nonce);
                    }
                    uart_command_length = 0;
                    discard_uart_command = false;
                    continue;
                }
                if (!discard_uart_command) {
                    if (uart_command_length + 1 < sizeof(uart_command)) {
                        uart_command[uart_command_length++] = value;
                    } else {
                        uart_command_length = 0;
                        discard_uart_command = true;
                    }
                }
            }
        }

        activity_request_t request;
        if (xQueueReceive(s_activity_queue, &request, pdMS_TO_TICKS(10)) == pdPASS) {
            process_activity(&request);
        }
        report_queue_drops();
    }
}

esp_err_t activity_log_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_open(ACTIVITY_NAMESPACE, NVS_READWRITE, &s_nvs_handle);
    if (err != ESP_OK) {
        return err;
    }

    uint32_t max_sequence = 0;
    err = scan_max_sequence(&max_sequence);
    if (err != ESP_OK) {
        nvs_close(s_nvs_handle);
        return err;
    }
    s_next_sequence = max_sequence == UINT32_MAX ? 0 : max_sequence + 1;
    s_session_id = ((uint64_t)esp_random() << 32) | esp_random();
    s_storage_writable = s_next_sequence != 0;

    s_activity_queue = xQueueCreate(ACTIVITY_QUEUE_LENGTH,
                                    sizeof(activity_request_t));
    if (s_activity_queue == NULL) {
        nvs_close(s_nvs_handle);
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreate(activity_log_task, "activity_log", ACTIVITY_TASK_STACK,
                    NULL, 1, NULL) != pdPASS) {
        vQueueDelete(s_activity_queue);
        s_activity_queue = NULL;
        nvs_close(s_nvs_handle);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

bool activity_log_record(activity_log_kind_t activity, char target,
                         bool correct, uint32_t attempt)
{
    if (s_activity_queue == NULL || attempt == 0 ||
        (activity != ACTIVITY_LOG_VOWEL_CHOICE &&
         activity != ACTIVITY_LOG_CLOUD_SEARCH) ||
        target < 'A' || target > 'Z') {
        return false;
    }

    activity_request_t request = {
        .activity = activity,
        .target = target,
        .correct = correct,
        .attempt = attempt,
        .timestamp_ms = (uint64_t)(esp_timer_get_time() / 1000),
        .session_id = s_session_id
    };
    if (xQueueSend(s_activity_queue, &request, 0) != pdPASS) {
        portENTER_CRITICAL(&s_drop_lock);
        if (s_queue_dropped < UINT32_MAX) {
            ++s_queue_dropped;
        }
        portEXIT_CRITICAL(&s_drop_lock);
        return false;
    }
    return true;
}
