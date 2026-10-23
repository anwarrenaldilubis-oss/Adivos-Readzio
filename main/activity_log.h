#ifndef ACTIVITY_LOG_H
#define ACTIVITY_LOG_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef enum {
    ACTIVITY_LOG_VOWEL_CHOICE = 1,
    ACTIVITY_LOG_CLOUD_SEARCH = 2
} activity_log_kind_t;

esp_err_t activity_log_init(void);
bool activity_log_record(activity_log_kind_t activity, char target, bool correct,
                         uint32_t attempt);

#endif
