/*
 * Copyright (c) 2020 Daniel Veilleux
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

/*
 * NOTE: Invalid measurements manifest as a 128600us pulse followed by a second pulse of ~6us
 *       about 145us later. This pulse can't be truncated so it effectively reduces the sensor's
 *       working rate.
 */
 
 
#define DT_DRV_COMPAT hc_sr04

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/devicetree.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(hc_sr04, CONFIG_SENSOR_LOG_LEVEL);

/* Timings defined by spec */
#define T_TRIG_PULSE_US       11
#define T_INVALID_PULSE_US    25000
#define T_MAX_WAIT_MS         130
#define T_SPURIOS_WAIT_US     145
#define METERS_PER_SEC        340

enum hc_sr04_state {
    HC_SR04_STATE_IDLE,
    HC_SR04_STATE_RISING_EDGE,
    HC_SR04_STATE_FALLING_EDGE,
    HC_SR04_STATE_FINISHED,
    HC_SR04_STATE_ERROR,
    HC_SR04_STATE_COUNT
};

static struct hc_sr04_shared_resources {
    struct k_sem         fetch_sem;
    struct k_mutex       mutex;
    bool                 ready; /* The module has been initialized */
    enum hc_sr04_state   state;
    uint32_t             start_time;
    uint32_t             end_time;
} m_shared_resources;

struct hc_sr04_data {
    struct sensor_value   sensor_value;
    const struct device  *trig_dev;
    const struct device  *echo_dev;
    struct gpio_callback  echo_cb_data;
};

struct hc_sr04_config {
	const struct gpio_dt_spec gpio_trig;
	const struct gpio_dt_spec gpio_echo;
};

static void input_changed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    switch (m_shared_resources.state) {
    case HC_SR04_STATE_RISING_EDGE:
        m_shared_resources.start_time = k_cycle_get_32();
        m_shared_resources.state = HC_SR04_STATE_FALLING_EDGE;
        break;
    case HC_SR04_STATE_FALLING_EDGE:
        m_shared_resources.end_time = k_cycle_get_32();
        (void) gpio_remove_callback(dev, cb);
        m_shared_resources.state = HC_SR04_STATE_FINISHED;
        k_sem_give(&m_shared_resources.fetch_sem);
        break;
    default:
        (void) gpio_remove_callback(dev, cb);
        m_shared_resources.state = HC_SR04_STATE_ERROR;
        break;
    }
}

static int hc_sr04_init(const struct device *dev)
{
    int err;

    struct hc_sr04_data      *p_data = dev->data;
    const struct hc_sr04_config *p_cfg  = dev->config;

    p_data->sensor_value.val1 = 0;
    p_data->sensor_value.val2 = 0;

    p_data->trig_dev = p_cfg->gpio_trig.port;
    if (!p_data->trig_dev) {
        return -ENODEV;
    }
    p_data->echo_dev = p_cfg->gpio_echo.port;
    if (!p_data->echo_dev) {
        return -ENODEV;
    }
    err = gpio_pin_configure(p_data->trig_dev, p_cfg->gpio_trig.pin, (GPIO_OUTPUT | p_cfg->gpio_trig.dt_flags));
    if (err != 0) {
        return err;
    }
    err = gpio_pin_configure(p_data->echo_dev, p_cfg->gpio_echo.pin, (GPIO_INPUT | p_cfg->gpio_echo.dt_flags));
    if (err != 0) {
        return err;
    }
    err = gpio_pin_interrupt_configure(p_data->echo_dev,
                                       p_cfg->gpio_echo.pin,
                                       GPIO_INT_EDGE_BOTH);
    if (err != 0) {
        return err;
    }
    gpio_init_callback(&p_data->echo_cb_data, input_changed, BIT(p_cfg->gpio_echo.pin));

    if (m_shared_resources.ready) {
        /* Already initialized */
        return 0;
    }

    err = k_sem_init(&m_shared_resources.fetch_sem, 0, 1);
    if (0 != err) {
        return err;
    }
    err = k_mutex_init(&m_shared_resources.mutex);
    if (0 != err) {
        return err;
    }

    m_shared_resources.state = HC_SR04_STATE_IDLE;
    m_shared_resources.ready = true;
    return 0;
}

static int hc_sr04_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    int      err;
    uint32_t count;

    struct hc_sr04_data      *p_data = dev->data;
    const struct hc_sr04_config *p_cfg  = dev->config;

    if (unlikely((SENSOR_CHAN_ALL != chan) && (SENSOR_CHAN_DISTANCE != chan))) {
        return -ENOTSUP;
    }

    if (unlikely(!m_shared_resources.ready)) {
        LOG_ERR("Driver is not initialized yet");
        return -EBUSY;
    }

    err = k_mutex_lock(&m_shared_resources.mutex, K_FOREVER);
    if (0 != err) {
        return err;
    }

    err = gpio_add_callback(p_data->echo_dev, &p_data->echo_cb_data);
    if (0 != err) {
        LOG_DBG("Failed to add HC-SR04 echo callback");
        (void) k_mutex_unlock(&m_shared_resources.mutex);
        return -EIO;
    }

    m_shared_resources.state = HC_SR04_STATE_RISING_EDGE;
    gpio_pin_set(p_data->trig_dev, p_cfg->gpio_trig.pin, 1);
    k_busy_wait(T_TRIG_PULSE_US);
    gpio_pin_set(p_data->trig_dev, p_cfg->gpio_trig.pin, 0);

    if (0 != k_sem_take(&m_shared_resources.fetch_sem, K_MSEC(T_MAX_WAIT_MS))) {
        LOG_DBG("No response from HC-SR04");
        (void) k_mutex_unlock(&m_shared_resources.mutex);
        err = gpio_remove_callback(p_data->echo_dev, &p_data->echo_cb_data);
        if (0 != err) {
            return err;
        }
        return -EIO;
    }

    __ASSERT_NO_MSG(HC_SR04_STATE_FINISHED == m_shared_resources.state);

    if (m_shared_resources.start_time <= m_shared_resources.end_time) {
        count = (m_shared_resources.end_time - m_shared_resources.start_time);
    } else {
        count =  (0xFFFFFFFF - m_shared_resources.start_time);
        count += m_shared_resources.end_time;
    }
    /* Convert from ticks to nanoseconds and then to microseconds */
    count = k_cyc_to_us_near32(count);
    if ((T_INVALID_PULSE_US > count) && (T_TRIG_PULSE_US < count)) {
        /* Convert to meters and divide round-trip distance by two */
        count = (count * METERS_PER_SEC / 2);
        p_data->sensor_value.val2 = (count % 1000000);
        p_data->sensor_value.val1 = (count / 1000000);
    } else {
        LOG_INF("Invalid measurement");
        p_data->sensor_value.val1 = 0;
        p_data->sensor_value.val2 = 0;
        k_usleep(T_SPURIOS_WAIT_US);
    }

    err = k_mutex_unlock(&m_shared_resources.mutex);
    if (0 != err) {
        return err;
    }
    return 0;
}

static int hc_sr04_channel_get(const struct device *dev,
                    enum sensor_channel chan,
                    struct sensor_value *val)
{
    const struct hc_sr04_data *p_data = dev->data;

    if (unlikely(!m_shared_resources.ready)) {
        LOG_WRN("Device is not initialized yet");
        return -EBUSY;
    }

    switch (chan) {
    case SENSOR_CHAN_DISTANCE:
        val->val2 = p_data->sensor_value.val2;
        val->val1 = p_data->sensor_value.val1;
        break;
    default:
        return -ENOTSUP;
    }
    return 0;
}

#ifdef CONFIG_PM_DEVICE

static int hc_sr04_pm_action(const struct device *dev, enum pm_device_action action)
{
	switch (action) {
	case PM_DEVICE_ACTION_TURN_ON:
	case PM_DEVICE_ACTION_RESUME:
	case PM_DEVICE_ACTION_TURN_OFF:
	case PM_DEVICE_ACTION_SUSPEND:
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

#endif

static const struct sensor_driver_api hc_sr04_driver_api = {
    .sample_fetch = hc_sr04_sample_fetch,
    .channel_get  = hc_sr04_channel_get,
};

#define HC_SR04_INST_DEFINE(inst)                                       \
    static const struct hc_sr04_config hc_sr04_config_##inst = {        \
        .gpio_trig = GPIO_DT_SPEC_INST_GET(inst, trig_gpios),           \
        .gpio_echo = GPIO_DT_SPEC_INST_GET(inst, echo_gpios),           \
    };                                                                  \
    static struct hc_sr04_data hc_sr04_data_##inst;                     \
    SENSOR_DEVICE_DT_INST_DEFINE(inst,                                  \
                hc_sr04_init,                                           \
                NULL,                                                   \
                &hc_sr04_data_##inst,	                                \
		        &hc_sr04_config_##inst,                                 \
                POST_KERNEL,                                            \
		        CONFIG_SENSOR_INIT_PRIORITY,                            \
                &hc_sr04_driver_api);

// #if DT_NUM_INST_STATUS_OKAY(HC_SR04_INST_DEFINE) == 0
// #warning "HC_SR04 driver enabled without any devices"
// #endif

DT_INST_FOREACH_STATUS_OKAY(HC_SR04_INST_DEFINE)
