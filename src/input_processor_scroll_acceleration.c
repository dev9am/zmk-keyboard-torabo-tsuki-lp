/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: MIT
 *
 * Sigmoid-based scroll acceleration input processor for ZMK.
 * Inspired by kloir-z/zmk-pmw3610-driver scroll-acceleration-feature.
 *
 * param1: sensitivity (1-10, higher = more acceleration)
 * param2: threshold (speed value at which acceleration reaches 50%, 0 = default 500)
 */

#define DT_DRV_COMPAT zmk_input_processor_scroll_acceleration

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <drivers/input_processor.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define DEFAULT_THRESHOLD 500
#define MAX_DELTA_TIME_MS 100
#define SENSITIVITY_MIN 1
#define SENSITIVITY_MAX 10

struct scroll_accel_config {
    uint8_t type;
    size_t codes_len;
    uint16_t codes[];
};

struct scroll_accel_data {
    int64_t last_event_time;
};

static bool event_matches(const struct scroll_accel_config *cfg, struct input_event *event) {
    if (event->type != cfg->type) {
        return false;
    }

    for (int i = 0; i < cfg->codes_len; i++) {
        if (cfg->codes[i] == event->code) {
            return true;
        }
    }

    return false;
}

static int scroll_accel_handle_event(const struct device *dev, struct input_event *event,
                                     uint32_t param1, uint32_t param2,
                                     struct zmk_input_processor_state *state) {
    const struct scroll_accel_config *cfg = dev->config;
    struct scroll_accel_data *data = dev->data;

    if (!event_matches(cfg, event)) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    int32_t value = event->value;
    int64_t current_time = k_uptime_get();
    int64_t delta_time = data->last_event_time > 0 ?
                         current_time - data->last_event_time : 0;

    data->last_event_time = current_time;

    if (delta_time <= 0 || delta_time >= MAX_DELTA_TIME_MS) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    int32_t sensitivity = CLAMP((int32_t)param1, SENSITIVITY_MIN, SENSITIVITY_MAX);
    int32_t threshold = param2 > 0 ? (int32_t)param2 : DEFAULT_THRESHOLD;

    /* Speed scaled by 100 to avoid floating point */
    int32_t speed_x100 = (abs(value) * 100) / (int32_t)delta_time;

    /* Integer sigmoid approximation: speed / (speed + threshold) → 0-100% */
    int32_t sigmoid_pct = (speed_x100 * 100) / (speed_x100 + threshold);

    /* Multiplier in percentage: 100% at rest, up to (sensitivity * 100)% at max speed */
    int32_t accel_pct = 100 + (sensitivity - 1) * sigmoid_pct;

    int32_t new_value = (value * accel_pct) / 100;

    /* Preserve minimum movement direction */
    if (new_value == 0 && value != 0) {
        new_value = value;
    }

    LOG_DBG("scroll accel: value=%d speed=%d sigmoid=%d%% accel=%d%% result=%d",
            value, speed_x100, sigmoid_pct, accel_pct, new_value);

    event->value = new_value;

    return ZMK_INPUT_PROC_CONTINUE;
}

static struct zmk_input_processor_driver_api scroll_accel_driver_api = {
    .handle_event = scroll_accel_handle_event,
};

#define SCROLL_ACCEL_INST(n)                                                                       \
    static struct scroll_accel_data scroll_accel_data_##n = {};                                    \
    static const struct scroll_accel_config scroll_accel_config_##n = {                            \
        .type = DT_INST_PROP_OR(n, type, INPUT_EV_REL),                                            \
        .codes_len = DT_INST_PROP_LEN(n, codes),                                                   \
        .codes = DT_INST_PROP(n, codes),                                                           \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, NULL, NULL, &scroll_accel_data_##n, &scroll_accel_config_##n,         \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                        \
                          &scroll_accel_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SCROLL_ACCEL_INST)
