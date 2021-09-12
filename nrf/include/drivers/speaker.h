/**
 * @file speaker.h
 *
 * @brief Public API for driving a speaker using the nRF PWM peripheral
 */

/*
 * Copyright (c) 2021 Daniel Veilleux
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */
#ifndef ZEPHYR_INCLUDE_SPEAKER_H_
#define ZEPHYR_INCLUDE_SPEAKER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr.h>
#include <device.h>
#include <stdbool.h>

#define SPEAKER_MIN_FREQ_HZ     100
#define SPEAKER_MAX_FREQ_HZ     20000
#define SPEAKER_MIN_VOLUME      0
#define SPEAKER_MAX_VOLUME      100
#define SPEAKER_MAX_DURATION_MS 30000

typedef enum
{
    SPEAKER_EVENT_FINISHED,
    SPEAKER_EVENT_COUNT
} speaker_event_t;

typedef void (*speaker_callback_t) (speaker_event_t event);

typedef int (*speaker_init_t)      (const struct device *dev);
typedef int (*speaker_tone_play_t) (const struct device *dev,
                                       uint16_t freq_hz, uint32_t duration_ms, uint8_t volume,
                                       speaker_callback_t cb);
typedef int (*speaker_pcm_play_t)  (const struct device *dev,
                                       uint8_t const * const sound, size_t len,
                                       speaker_callback_t cb);

/**
 * @brief Speaker driver API
 */
struct speaker_driver_api {
    speaker_init_t      init;
    speaker_tone_play_t tone_play;
    speaker_pcm_play_t  pcm_play;
};

static inline int speaker_init(const struct device *dev)
{
    struct speaker_driver_api *api;

    if (dev == NULL) {
        return -EINVAL;
    }

    api = (struct speaker_driver_api*)dev->api;

    if (api->init == NULL) {
        return -ENOTSUP;
    }
    return api->init(dev);
}

static inline int speaker_tone_play(const struct device *dev,
                                   uint16_t freq_hz, uint32_t duration_ms, uint8_t volume,
                                   speaker_callback_t cb)
{
    struct speaker_driver_api *api;

    if (dev == NULL) {
        return -EINVAL;
    }

    api = (struct speaker_driver_api*)dev->api;

    if (api->tone_play == NULL) {
        return -ENOTSUP;
    }
    return api->tone_play(dev, freq_hz, duration_ms, volume, cb);
}

/**
 * @brief Play the sound. Set to the callback to NULL if the app doesn't need to know
 *        when the sound has finished.
 */
static inline int speaker_pcm_play(const struct device *dev,
                                       uint8_t const * const sound, size_t len,
                                       speaker_callback_t cb)
{
    struct speaker_driver_api *api;

    if (dev == NULL) {
        return -EINVAL;
    }

    api = (struct speaker_driver_api*)dev->api;

    if (api->pcm_play == NULL) {
        return -ENOTSUP;
    }
    return api->pcm_play(dev, sound, len, cb);
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_SPEAKER_H_ */
