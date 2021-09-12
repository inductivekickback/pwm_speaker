/*
 * Copyright (c) 2021 Daniel Veilleux
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#define DT_DRV_COMPAT nordic_nrf_sw_speaker

#include <kernel.h>
#include <device.h>
#include <devicetree.h>
#include <irq.h>

#include "math.h"
#include "arm_math.h"

#include <nrfx_pwm.h>
#include <hal/nrf_gpio.h>
#include <logging/log.h>

#include <drivers/speaker.h>

#define SEQ_REPEATS        7

#define TONE_BUF_SIZE      625 // Max required buffer is for 100Hz tone
#define SEQ_BUF_SIZE       20
#define INVALID_ENABLE_PIN 0xff

LOG_MODULE_REGISTER(nordic_nrf_sw_speaker, CONFIG_SPEAKER_LOG_LEVEL);

struct pwm_pcm_t
{
    uint16_t           seq0_buf[SEQ_BUF_SIZE];
    uint16_t           seq1_buf[SEQ_BUF_SIZE];
    uint8_t           *data;
    uint32_t           data_head;
    uint32_t           data_tail;
    uint32_t           buf_size;
    nrf_pwm_sequence_t seq0;
    nrf_pwm_sequence_t seq1;
    bool               ramp_down_started;
};

typedef struct
{
    unsigned int irq_p;
    unsigned int priority_p;
    nrfx_pwm_t   pwm_instance;
    bool         ready;
    void (*isr_p)(void);
} pwm_periph_t;

static pwm_periph_t m_avail_pwms[] = {
#if CONFIG_SPEAKER_ALLOW_PWM0
    {
        .pwm_instance = NRFX_PWM_INSTANCE(0),
        .irq_p        = DT_IRQN(DT_NODELABEL(pwm0)),
        .isr_p        = nrfx_pwm_0_irq_handler,
        .priority_p   = DT_IRQ(DT_NODELABEL(pwm0), priority),
        .ready        = false,
    },
#endif
#if CONFIG_SPEAKER_ALLOW_PWM1
    {
        .pwm_instance = NRFX_PWM_INSTANCE(1),
        .irq_p        = DT_IRQN(DT_NODELABEL(pwm1)),
        .isr_p        = nrfx_pwm_1_irq_handler,
        .priority_p   = DT_IRQ(DT_NODELABEL(pwm1), priority),
        .ready        = false,
    },
#endif
#if CONFIG_SPEAKER_ALLOW_PWM2
    {
        .pwm_instance = NRFX_PWM_INSTANCE(2),
        .irq_p        = DT_IRQN(DT_NODELABEL(pwm2)),
        .isr_p        = nrfx_pwm_2_irq_handler,
        .priority_p   = DT_IRQ(DT_NODELABEL(pwm2), priority),
        .ready        = false,
    },
#endif
#if CONFIG_SPEAKER_ALLOW_PWM3
    {
        .pwm_instance = NRFX_PWM_INSTANCE(3),
        .irq_p        = DT_IRQN(DT_NODELABEL(pwm3)),
        .isr_p        = nrfx_pwm_3_irq_handler,
        .priority_p   = DT_IRQ(DT_NODELABEL(pwm3), priority),
        .ready        = false,
    },
#endif
};

#define NUM_AVAIL_PWMS (sizeof(m_avail_pwms)/sizeof(pwm_periph_t))

static const uint8_t ramp_down_seq[] = { 0x7d, 0x76, 0x6f, 0x69, 0x62,
                                         0x5c, 0x55, 0x4e, 0x48, 0x41, 
                                         0x3b, 0x34, 0x2e, 0x27, 0x20,
                                         0x1a, 0x13, 0x0d, 0x06, 0x00 };

struct speaker_cfg {
    const uint32_t output_pin;
    const uint32_t enable_pin;
    const uint8_t  pwm_index;
};

struct speaker_data {
    struct k_sem              sem;
    struct k_work             work;
    struct pwm_pcm_t          pcm;
    uint16_t                  tone_buf[TONE_BUF_SIZE];
    speaker_callback_t        cb;
    nrfx_pwm_t               *pwm_instance;
    const struct speaker_cfg *cfg;
    bool                      ready;
};

static void callback(struct k_work *item)
{
    struct speaker_data *p_data = CONTAINER_OF(item, struct speaker_data, work);
    if (p_data->cb) {
        p_data->cb(SPEAKER_EVENT_FINISHED);
    }
}

static void speaker_disable(uint32_t pin)
{
    if (INVALID_ENABLE_PIN != pin) {
        nrf_gpio_pin_clear(pin);
    }
}

static void speaker_enable(uint32_t pin)
{
    if (INVALID_ENABLE_PIN != pin) {
        nrf_gpio_pin_set(pin);
    }
}

static void ramp_down(nrfx_pwm_t *pwm, uint16_t *seq_buf, nrf_pwm_sequence_t *seq)
{
    /* NOTE: EasyDMA buffers must be in RAM. */
    memcpy(seq_buf, ramp_down_seq, sizeof(ramp_down_seq));

    seq->values.p_common = seq_buf;
    seq->length          = sizeof(ramp_down_seq);
    seq->repeats         = SEQ_REPEATS;
    seq->end_delay       = 0;

    (void) nrfx_pwm_simple_playback(pwm, seq, 1, NRFX_PWM_FLAG_STOP);
}

static uint32_t seq0_buffer_update(struct speaker_data *p_data)
{
    struct pwm_pcm_t *p_pcm = &p_data->pcm;

    if (p_pcm->data_head == p_pcm->data_tail)
    {
        if(!p_pcm->ramp_down_started)
        {
            ramp_down(p_data->pwm_instance, p_pcm->seq0_buf, &p_pcm->seq0);
            p_pcm->ramp_down_started = true;
        }
        return 0;
    }

    uint16_t buf_size = SEQ_BUF_SIZE;
    uint32_t pwm_flag = (NRFX_PWM_FLAG_SIGNAL_END_SEQ0 | NRFX_PWM_FLAG_NO_EVT_FINISHED);

    for (uint32_t i = 0; i < SEQ_BUF_SIZE; i++)
    {
        if (p_pcm->data_head == p_pcm->data_tail)
        {
            buf_size = i;
            pwm_flag = NRFX_PWM_FLAG_STOP;
            break;
        }
        p_pcm->seq0_buf[i] = (uint16_t)p_pcm->data[p_pcm->data_head++];
    }

    p_pcm->seq0.values.p_common = p_pcm->seq0_buf;
    p_pcm->seq0.length          = buf_size;
    p_pcm->seq0.repeats         = SEQ_REPEATS;
    p_pcm->seq0.end_delay       = 0;

    return pwm_flag;
}

static uint32_t seq1_buffer_update(struct speaker_data *p_data)
{
    struct pwm_pcm_t *p_pcm = &p_data->pcm;

    if (p_pcm->data_head == p_pcm->data_tail)
    {
        if(!p_pcm->ramp_down_started)
        {
            ramp_down(p_data->pwm_instance, p_pcm->seq1_buf, &p_pcm->seq1);
            p_pcm->ramp_down_started = true;
        }
        return 0;
    }

    uint16_t buf_size = SEQ_BUF_SIZE;
    uint32_t pwm_flag = (NRFX_PWM_FLAG_SIGNAL_END_SEQ1 | NRFX_PWM_FLAG_NO_EVT_FINISHED);

    for (uint32_t i = 0; i < SEQ_BUF_SIZE; i++)
    {
        if (p_pcm->data_head == p_pcm->data_tail)
        {
            buf_size = i;
            pwm_flag = NRFX_PWM_FLAG_STOP;
            break;
        }
        p_pcm->seq1_buf[i] = (uint16_t)p_pcm->data[p_pcm->data_head++];
    }

    p_pcm->seq1.values.p_common = p_pcm->seq1_buf;
    p_pcm->seq1.length          = buf_size;
    p_pcm->seq1.repeats         = SEQ_REPEATS;
    p_pcm->seq1.end_delay       = 0;

    return pwm_flag;
}

static void pwm_handler(nrfx_pwm_evt_type_t event_type, void *p_context)
{
    struct speaker_data *p_data = (struct speaker_data*)p_context;

    switch (event_type) {
    case NRFX_PWM_EVT_STOPPED:
        speaker_disable(p_data->cfg->enable_pin);
        k_sem_give(&p_data->sem);
        k_work_submit(&p_data->work);
        break;
    case NRFX_PWM_EVT_FINISHED:
        p_data->pcm.ramp_down_started = false;
        break;
    case NRFX_PWM_EVT_END_SEQ0:
        (void)seq0_buffer_update(p_data);
        break;
    case NRFX_PWM_EVT_END_SEQ1:
        (void)seq1_buffer_update(p_data);
        break;
    default:
        break;
    }
}

static int nrf_sw_speaker_tone_play(const struct device *dev,
                                   uint16_t freq_hz, uint32_t duration_ms, uint8_t volume,
                                   speaker_callback_t cb)
{
    const struct speaker_cfg *p_cfg  = dev->config;
    struct speaker_data      *p_data = dev->data;

    if (unlikely(!p_data->ready)) {
        LOG_ERR("Driver is not initialized");
        return -EBUSY;
    }

    int err = k_sem_take(&p_data->sem, K_FOREVER);
    if (0 != err) {
        return err;
    }

    if ((0 == duration_ms) || (0 == volume)) {
        speaker_disable(p_cfg->enable_pin);
        (void) nrfx_pwm_stop(p_data->pwm_instance, true);
        return -1;
    }

    if ((SPEAKER_MIN_FREQ_HZ > freq_hz) || (SPEAKER_MAX_FREQ_HZ < freq_hz))
    {
        return -2;
    }

    if (SPEAKER_MAX_VOLUME < volume)
    {
        return -2;
    }

    if (SPEAKER_MAX_DURATION_MS < duration_ms)
    {
        return -2;
    }

    p_data->cb = cb;

    float    period         = (1.0f / freq_hz);
    uint32_t num            = MIN(TONE_BUF_SIZE, (uint32_t)(period / 16.0e-6f));
    uint16_t playback_count = (uint32_t)((duration_ms / 1000.0f) / period);

    for (uint32_t i = 0; i < num; i++)
    {
        float temp = (float)volume * 125.0f;
        temp *= arm_sin_f32(2.0f * PI * freq_hz * i * 16.0e-6f - PI/2);
        p_data->tone_buf[i]  = (uint16_t)((temp / 100.0f) + 125.0f);
    }

    nrf_pwm_sequence_t const seq =
    {
        .values.p_common = p_data->tone_buf,
        .length          = num,
        .repeats         = 0,
        .end_delay       = 0
    };

    speaker_enable(p_cfg->enable_pin);
    nrfx_pwm_simple_playback(p_data->pwm_instance, &seq, playback_count, NRFX_PWM_FLAG_STOP);
    return 0;
}

static int nrf_sw_speaker_pcm_play(const struct device *dev,
                                   uint8_t const * const sound, size_t len,
                                   speaker_callback_t cb)
{
    const struct speaker_cfg *p_cfg  = dev->config;
    struct speaker_data      *p_data = dev->data;

    if (unlikely(!p_data->ready)) {
        LOG_ERR("Driver is not initialized");
        return -EBUSY;
    }

    int err = k_sem_take(&p_data->sem, K_FOREVER);
    if (0 != err) {
        return err;
    }

    p_data->cb            = cb;
    p_data->pcm.data      = (uint8_t *)sound;
    p_data->pcm.data_head = 0;
    p_data->pcm.data_tail = (len - 1);
    p_data->pcm.buf_size  = len;

    uint32_t pwm_flags = (seq0_buffer_update(p_data) | seq1_buffer_update(p_data));

    speaker_enable(p_cfg->enable_pin);

    (void)nrfx_pwm_complex_playback(p_data->pwm_instance, &p_data->pcm.seq0, &p_data->pcm.seq1,
                                        1, (pwm_flags | NRFX_PWM_FLAG_LOOP));
    return 0;
}

static int nrf_sw_speaker_init(const struct device *dev)
{
    pwm_periph_t *p_periph;
    nrfx_pwm_t   *p_inst;

    const struct speaker_cfg *p_cfg  = dev->config;
    struct speaker_data      *p_data = dev->data;

    if (unlikely(p_data->ready)) {
        /* Already initialized */
        return 0;
    }

    if (NUM_AVAIL_PWMS <= p_cfg->pwm_index) {
        goto ERR_EXIT;
    }

    if (0 != k_sem_init(&p_data->sem, 1, 1)) {
        goto ERR_EXIT;
    }

    p_periph = &m_avail_pwms[p_cfg->pwm_index];
    p_inst   = &p_periph->pwm_instance;

    if (!p_periph->ready) {
        nrfx_pwm_config_t config = {
          .output_pins = { p_cfg->output_pin,
                             NRFX_PWM_PIN_NOT_USED,
                             NRFX_PWM_PIN_NOT_USED,
                             NRFX_PWM_PIN_NOT_USED },
          .base_clock  = NRF_PWM_CLK_16MHz,
          .load_mode   = NRF_PWM_LOAD_COMMON,
          .count_mode  = NRF_PWM_MODE_UP,
          .top_value   = 0xff,
          .step_mode   = NRF_PWM_STEP_AUTO };

        /* NOTE: irq_connect_dynamic returns a vector index instead of an error code. */
        irq_connect_dynamic(p_periph->irq_p,
                              p_periph->priority_p,
                              nrfx_isr,
                              p_periph->isr_p,
                              0);

        nrfx_err_t err = nrfx_pwm_init(p_inst,
                                         &config,
                                         pwm_handler,
                                         p_data);
        if (NRFX_SUCCESS != err) {
            goto ERR_EXIT;
        }

        p_periph->ready = true;
    }

    if (INVALID_ENABLE_PIN != p_cfg->enable_pin) {
        nrf_gpio_pin_clear(p_cfg->enable_pin);
        nrf_gpio_cfg_output(p_cfg->enable_pin);
    }

    nrf_gpio_pin_clear(p_cfg->output_pin);
    nrf_gpio_cfg_output(p_cfg->output_pin);

    k_work_init(&p_data->work, callback);

    p_data->pwm_instance = p_inst;
    p_data->cfg          = p_cfg;
    p_data->ready        = true;
    return 0;

ERR_EXIT:
    return -ENXIO;
}

static const struct speaker_driver_api speaker_driver_api = {
    .init      = nrf_sw_speaker_init,
    .pcm_play  = nrf_sw_speaker_pcm_play,
    .tone_play = nrf_sw_speaker_tone_play,
};

#define INST(num) DT_INST(num, nordic_nrf_sw_speaker)

#define SPEAKER_DEVICE(n) \
    static const struct speaker_cfg speaker_cfg_##n = { \
        .output_pin = DT_PROP(INST(n), output_pin), \
        .enable_pin = DT_PROP(INST(n), enable_pin), \
        .pwm_index  = (n) \
    }; \
    static struct speaker_data speaker_data_##n; \
    DEVICE_DEFINE(speaker_##n, \
                DT_LABEL(INST(n)), \
                nrf_sw_speaker_init, \
                NULL, \
                &speaker_data_##n, \
                &speaker_cfg_##n, \
                POST_KERNEL, \
                CONFIG_SPEAKER_INIT_PRIORITY, \
                &speaker_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SPEAKER_DEVICE)

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "Speaker driver enabled without any devices"
#endif
