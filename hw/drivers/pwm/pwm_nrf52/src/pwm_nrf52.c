/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <hal/hal_bsp.h>
#include <assert.h>
#include <os/os.h>

#include <pwm/pwm.h>
#include <string.h>

/* Nordic headers */
#include <bsp.h>
#include <nrf.h>
#include <nrf_pwm.h>
#include <nrf_drv_pwm.h>
#include <nrf_drv_clock.h>
#include <bsp/cmsis_nvic.h>
#include <app_timer.h>

//#include <app_error.h>
#include <app_util_platform.h>
#include <nrf_drv_ppi.h>
#include <nrf_drv_timer.h>

extern void PWM0_IRQHandler(void);

/* Mynewt Nordic driver */
#include "pwm_nrf52/pwm_nrf52.h"

#define PWM_MAX_INSTANCES 3
#define PWM_NO_INSTANCE -1
//static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);

struct nrf53_pwm_dev_global {
    bool in_use;
    nrf_drv_pwm_t *drv_instance;
    nrf_drv_pwm_config_t *config;
    void *duty_cycles;
};

static struct nrf53_pwm_dev_global instances[PWM_MAX_INSTANCES];

/* /\** */
/*  * Validate the contents of a given nrf_drv_pwm_config_t structure. */
/*  *\/ */
/* static int */
/* validate_config(nrf_drv_pwm_config_t* config) */
/* { */
/*     return 0; */
/* } */

/**
 * Add a driver instance.
 */
static int
add_instance(int inst_id, nrf_drv_pwm_config_t* init_conf)
{
    nrf_drv_pwm_config_t * config;
    nrf_drv_pwm_t instance = NRF_DRV_PWM_INSTANCE(0);

    /* assert(instances[inst_id].drv_instance == NULL && */
    /*        instances[inst_id].config == NULL); */

    instances[inst_id].drv_instance =
        (nrf_drv_pwm_t *) os_malloc(sizeof(nrf_drv_pwm_t));
    assert(instances[inst_id].drv_instance);

    instances[inst_id].config =
        (nrf_drv_pwm_config_t *) os_malloc(sizeof(nrf_drv_pwm_config_t));
    assert(instances[inst_id].config);

    memcpy(instances[inst_id].drv_instance, &instance, sizeof(nrf_drv_pwm_t));
    instances[inst_id].duty_cycles = NULL;

    //what should the defaults be?
    config = instances[inst_id].config;
    if (!init_conf) {
        config->output_pins[0] = LED_2 | NRF_DRV_PWM_PIN_INVERTED;
        config->output_pins[1] = LED_1;
        config->output_pins[2] = NRF_DRV_PWM_PIN_NOT_USED;
        config->output_pins[3] = NRF_DRV_PWM_PIN_NOT_USED;
        config->irq_priority = APP_IRQ_PRIORITY_LOW;
        config->base_clock   = NRF_PWM_CLK_1MHz;
        config->count_mode   = NRF_PWM_MODE_UP;
        config->top_value    = 10000;
        config->load_mode    = NRF_PWM_LOAD_INDIVIDUAL;
        config->step_mode    = NRF_PWM_STEP_AUTO;
    } else {
        memcpy(config, init_conf, sizeof(nrf_drv_pwm_config_t));
    }

    return 0;
}

/**
 * Remove a driver instance.
 */
static void
remove_instance(int inst_id)
{
    free(instances[inst_id].drv_instance);
    instances[inst_id].drv_instance = NULL;
    free(instances[inst_id].config);
    instances[inst_id].config = NULL;
    free(instances[inst_id].duty_cycles);
    instances[inst_id].duty_cycles = NULL;
}

/**
 * Open the NRF52 PWM device
 *
 * This function locks the device for access from other tasks.
 *
 * @param odev The OS device to open
 * @param wait The time in MS to wait.  If 0 specified, returns immediately
 *             if resource unavailable.  If OS_WAIT_FOREVER specified, blocks
 *             until resource is available.
 * @param arg  Argument provided by higher layer to open, in this case
 *             it can be a nrf_drv_pwm_config_t, to override the default
 *             configuration.
 *
 * @return 0 on success, non-zero on failure.
 */
static int
nrf52_pwm_open(struct os_dev *odev, uint32_t wait, void *arg)
{
    struct pwm_dev *dev;
    int stat = 0;
    int inst_id;
    dev = (struct pwm_dev *) odev;
    inst_id = dev->pwm_instance_id;

    if (os_started()) {
        stat = os_mutex_pend(&dev->pwm_lock, wait);
        if (stat != OS_OK) {
            return stat;
        }
    }

    if (odev->od_flags & OS_DEV_F_STATUS_OPEN) {
        os_mutex_release(&dev->pwm_lock);
        stat = OS_EBUSY;
        return stat;
    }

    add_instance(inst_id, arg);

    stat = nrf_drv_pwm_init(instances[inst_id].drv_instance,
                            instances[inst_id].config,
                            NULL);
    if (stat != NRF_SUCCESS) {
        return -stat;
    }

    return 0;
}

/**
 * Close the NRF52 PWM device.
 *
 * This function unlocks the device.
 *
 * @param odev The device to close.
 */
static int
nrf52_pwm_close(struct os_dev *odev)
{
    struct pwm_dev *dev;
    int inst_id;

    dev = (struct pwm_dev *) odev;
    inst_id = dev->pwm_instance_id;

    nrf_drv_pwm_uninit(instances[inst_id].drv_instance);

    remove_instance(inst_id);

    if (os_started()) {
        os_mutex_release(&dev->pwm_lock);
    }

    return 0;
}


/**
 * Configure a channel on the PWM device.
 *
 * @param dev The device to configure.
 * @param cnum The channel number to configure.
 * @param data Driver specific configuration data for this channel.
 *
 * @return 0 on success, non-zero error code on failure.
 */
static int
nrf52_pwm_configure_channel(struct pwm_dev *dev, uint8_t cnum, void *data)
{
    int inst_id = dev->pwm_instance_id;
    nrf_drv_pwm_uninit(instances[inst_id].drv_instance);

    return 0;
}

/**
 * Enable the PWM with specified duty cycle.
 *
 * This duty cycle is a fractional duty cycle where 0 == off, 65535=on,
 * and any value in between is on for fraction clocks and off
 * for 65535-fraction clocks.
 *
 * @param dev The device to configure.
 * @param cnum The channel number. The channel should be in use.
 * @param fraction The fraction value.
 *
 * @return 0 on success, negative on error.
 */
static int
nrf52_pwm_enable_duty_cycle(struct pwm_dev *dev, uint8_t cnum, uint16_t fraction)
{
    int inst_id = dev->pwm_instance_id;
    nrf_drv_pwm_config_t *config;

    //instances[dev->pwm_instance_id].in_use
    assert(instances[inst_id].config != NULL);
    config = instances[inst_id].config;
    assert (config->output_pins[cnum] != NRF_DRV_PWM_PIN_NOT_USED);

    if (instances[inst_id].duty_cycles == NULL) {
        instances[inst_id].duty_cycles = calloc(4, sizeof(nrf_pwm_sequence_t));
        assert(instances[inst_id].duty_cycles);
    }

    nrf_pwm_sequence_t const seq =
        {
            .values.p_individual = instances[inst_id].duty_cycles,
            .length              = 4,
            .repeats             = 0,
            .end_delay           = 0
        };

    ((uint16_t *) seq.values.p_individual)[cnum] = fraction;
    nrf_drv_pwm_simple_playback(instances[inst_id].drv_instance,
                                &seq,
                                1,
                                NRF_DRV_PWM_FLAG_LOOP);
    return 0;
}

/**
 * Disable the PWM channel, it will be marked as unconfigured.
 *
 * @param dev The device to configure.
 * @param cnum The channel number.
 *
 * @return 0 success, negative on error.
 */
static int
nrf52_pwm_disable(struct pwm_dev *dev, uint8_t cnum)
{
    return 0;
}

/**
 * This frequency must be between 1/2 the clock frequency and
 * the clock divided by the resolution. NOTE: This may affect
 * other PWM channels.
 *
 * @param dev The device to configure.
 * @param freq_hz The frequency value in Hz.
 *
 * @return A value is in Hz on success, negative on error.
 */
static int
nrf52_pwm_set_frequency(struct pwm_dev *dev, uint32_t freq_hz)
{
    //check if the freq is acceptable
    //set it
    /* int frq; */
    /* int inst_id = dev->pwm_instance_id; */

    /* switch (freq_hz) { */
    /* case 16000000: */
    /*     frq = NRF_PWM_CLK_16MHz; */
    /*     break; */
    /* case 8000000: */
    /*     frq = NRF_PWM_CLK_8MHz; */
    /*     break; */
    /* case 4000000: */
    /*     frq = NRF_PWM_CLK_4MHz; */
    /*     break; */
    /* case 2000000: */
    /*     frq = NRF_PWM_CLK_2MHz; */
    /*     break; */
    /* case 1000000: */
    /*     frq = NRF_PWM_CLK_1MHz; */
    /*     break; */
    /* case 500000: */
    /*     frq = NRF_PWM_CLK_500kHz; */
    /*     break; */
    /* case 250000: */
    /*     frq = NRF_PWM_CLK_250kHz; */
    /*     break; */
    /* case 125000: */
    /*     frq = NRF_PWM_CLK_125kHz; */
    /*     break; */
    /* default: */
    /*     return (OS_EINVAL); */
    /* } */

    /* nrf_drv_pwm_uninit(instances[inst_id].drv_instance); */
    /* instances[inst_id].config.base_clock = frq; */
    //Needs work
    return 0;
}

/**
 * Get the underlying clock driving the PWM device.
 *
 * @param dev
 *
 * @return value is in Hz on success, negative on error.
 */
static int
nrf52_pwm_get_clock_freq(struct pwm_dev *dev)
{
    int inst_id = dev->pwm_instance_id;

    if (instances[inst_id].config == NULL) {
        return (OS_EINVAL);
    }

    switch (instances[inst_id].config->base_clock) {
    case NRF_PWM_CLK_16MHz:
        return (16000000);
    case NRF_PWM_CLK_8MHz:
        return (8000000);
    case NRF_PWM_CLK_4MHz:
        return (4000000);
    case NRF_PWM_CLK_2MHz:
        return (2000000);
    case NRF_PWM_CLK_1MHz:
        return (1000000);
    case NRF_PWM_CLK_500kHz:
        return (500000);
    case NRF_PWM_CLK_250kHz:
        return (250000);
    case NRF_PWM_CLK_125kHz:
        return (125000);
    }

    return (OS_EINVAL);
}

/**
 * Get the resolution of the PWM in bits.
 *
 * @param dev The device to query.
 *
 * @return The value in bits on success, negative on error.
 */
static int
nrf52_pwm_get_resolution_bits(struct pwm_dev *dev)
{
    //resolution_bits = log2(PWM_Frequency / Timer_Clock_Frequency)
    // http://ww1.microchip.com/downloads/en/DeviceDoc/70209A.pdf page16
    /* float pwm_freq = (float) nrf52_pwm_get_clock_freq(dev); */

    return 0;
}

/**
 * Callback to initialize an adc_dev structure from the os device
 * initialization callback.  This sets up a nrf52_pwm_device(), so
 * that subsequent lookups to this device allow us to manipulate it.
 */
int
nrf52_pwm_dev_init(struct os_dev *odev, void *arg)
{

    struct pwm_dev *dev;
    struct pwm_driver_funcs *pwm_funcs;

    dev = (struct pwm_dev *) odev;

    if (arg) {
        dev->pwm_instance_id = *((int*) arg);
    } else {
        /* dev->pwm_instance_id = 0; */
        dev->pwm_instance_id = 0;
        while (dev->pwm_instance_id < PWM_MAX_INSTANCES) {
            if (!instances[dev->pwm_instance_id].in_use) {
                break;
            }
            dev->pwm_instance_id++;
        }
    }

    dev->pwm_chan_count = NRF_PWM_CHANNEL_COUNT;
    os_mutex_init(&dev->pwm_lock);

    OS_DEV_SETHANDLERS(odev, nrf52_pwm_open, nrf52_pwm_close);

    pwm_funcs = &dev->pwm_funcs;
    pwm_funcs->pwm_configure_channel = nrf52_pwm_configure_channel;
    pwm_funcs->pwm_enable_duty_cycle = nrf52_pwm_enable_duty_cycle;
    pwm_funcs->pwm_set_frequency = nrf52_pwm_set_frequency;
    pwm_funcs->pwm_get_clock_freq = nrf52_pwm_get_clock_freq;
    pwm_funcs->pwm_get_resolution_bits = nrf52_pwm_get_resolution_bits;
    pwm_funcs->pwm_disable = nrf52_pwm_disable;

    NVIC_SetVector(PWM0_IRQn, (uint32_t) PWM0_IRQHandler);
    return (0);
}
