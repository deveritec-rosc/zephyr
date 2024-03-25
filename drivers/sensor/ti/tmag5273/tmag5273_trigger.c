/*
 * Copyright (c) 2023 deveritec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "tmag5273.h"

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include <zephyr/drivers/sensor/tmag5273.h>
#include <zephyr/dt-bindings/sensor/tmag5273.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/check.h>
#include <zephyr/sys/util.h>

LOG_MODULE_DECLARE(TMAG5273, CONFIG_SENSOR_LOG_LEVEL);

#define TMAG5273_ON_INTERRUPT_BIT 0

static inline int8_t tmag5273_trigger_calc_magn_threshold_regdata_from_value(
	const struct device *dev, enum tmag5273_drv_data_threshold_axis axis, bool make_abs)
{
	struct tmag5273_data *drv_data = dev->data;
	int32_t value = drv_data->int_threshold[axis];

	if (value == 0) {
		return 0;
	}

	const int xyz_range_gauss = drv_data->xyz_range * TMAG5273_CONV_FACTOR_MT_TO_GS;

	if (make_abs) {
		value = abs(value);
	}

	if (value > xyz_range_gauss) {
		value = xyz_range_gauss;
	} else if (value < -xyz_range_gauss) {
		value = -xyz_range_gauss;
	}

	return (value * 128 * TMAG5273_CONV_FACTOR_GS_TO_MT) / drv_data->xyz_range;
}

static inline int32_t
tmag5273_trigger_calc_magn_threshold_value_from_regdata(const struct device *dev, int8_t regdata)
{
	struct tmag5273_data *drv_data = dev->data;
	return ((int32_t)regdata * drv_data->xyz_range * TMAG5273_CONV_FACTOR_MT_TO_GS) / 128;
}

static inline int tmag527_trigger_attr_write_threshold(const struct device *dev,
						       enum sensor_channel chan)
{
	const struct tmag5273_config *drv_cfg = dev->config;

	uint8_t data[3] = {0};
	uint8_t addr_start = TMAG5273_REG_X_THR_CONFIG;
	size_t num_bytes;

	int retval;

	BUILD_ASSERT(sizeof(data) == TMAG5273_DRV_DATA_THRESHOLD_AXIS_SIZE,
		     "size of data must be equal TMAG5273_DRV_DATA_THRESHOLD_AXIS_SIZE");

	retval = i2c_reg_read_byte_dt(&drv_cfg->i2c, TMAG5273_REG_DEVICE_CONFIG_2, &data[0]);
	if (retval < 0) {
		LOG_ERR("error reading magnet threshold %d", retval);
		return retval;
	}

	const bool make_abs = (data[0] & TMAG5273_THR_HYST_MSK) == TMAG5273_THR_HYST_LSB_ONLY;

	if (chan == SENSOR_CHAN_MAGN_XYZ) {
		num_bytes = 3;
		data[0] = tmag5273_trigger_calc_magn_threshold_regdata_from_value(
			dev, TMAG5273_DRV_DATA_THRESHOLD_AXIS_X, make_abs);
		data[1] = tmag5273_trigger_calc_magn_threshold_regdata_from_value(
			dev, TMAG5273_DRV_DATA_THRESHOLD_AXIS_Y, make_abs);
		data[2] = tmag5273_trigger_calc_magn_threshold_regdata_from_value(
			dev, TMAG5273_DRV_DATA_THRESHOLD_AXIS_Z, make_abs);
	} else {
		num_bytes = 1;

		switch (chan) {
		case SENSOR_CHAN_MAGN_X:
			data[0] = tmag5273_trigger_calc_magn_threshold_regdata_from_value(
				dev, TMAG5273_DRV_DATA_THRESHOLD_AXIS_X, make_abs);
			addr_start = TMAG5273_REG_X_THR_CONFIG;
			break;
		case SENSOR_CHAN_MAGN_Y:
			data[0] = tmag5273_trigger_calc_magn_threshold_regdata_from_value(
				dev, TMAG5273_DRV_DATA_THRESHOLD_AXIS_Y, make_abs);
			addr_start = TMAG5273_REG_Y_THR_CONFIG;
			break;
		case SENSOR_CHAN_MAGN_Z:
			data[0] = tmag5273_trigger_calc_magn_threshold_regdata_from_value(
				dev, TMAG5273_DRV_DATA_THRESHOLD_AXIS_Z, make_abs);
			addr_start = TMAG5273_REG_Z_THR_CONFIG;
			break;
		default:
			LOG_ERR("unsupported channel %d", chan);
			return -ENOTSUP;
		}
	}

	retval = i2c_burst_write_dt(&drv_cfg->i2c, addr_start, &data[0], num_bytes);
	if (retval < 0) {
		LOG_ERR("error writing threshold value %d", retval);
		return retval;
	}

	return 0;
}

static inline int tmag5273_trigger_attr_set_threshold(const struct device *dev,
						      enum sensor_channel chan,
						      const struct sensor_value *val)
{
	const struct tmag5273_config *drv_cfg = dev->config;
	struct tmag5273_data *drv_data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_MAGN_XYZ:
		drv_data->int_threshold[TMAG5273_DRV_DATA_THRESHOLD_AXIS_X] =
			drv_data->int_threshold[TMAG5273_DRV_DATA_THRESHOLD_AXIS_Y] =
				drv_data->int_threshold[TMAG5273_DRV_DATA_THRESHOLD_AXIS_Z] =
					val->val1;
		break;
	case SENSOR_CHAN_MAGN_X:
		if ((drv_cfg->axis & TMAG5273_MAG_CH_EN_X) != TMAG5273_MAG_CH_EN_X) {
			return -ENOTSUP;
		}

		drv_data->int_threshold[TMAG5273_DRV_DATA_THRESHOLD_AXIS_X] = val->val1;
		break;
	case SENSOR_CHAN_MAGN_Y:
		if ((drv_cfg->axis & TMAG5273_MAG_CH_EN_Y) != TMAG5273_MAG_CH_EN_Y) {
			return -ENOTSUP;
		}

		drv_data->int_threshold[TMAG5273_DRV_DATA_THRESHOLD_AXIS_Y] = val->val1;
		break;
	case SENSOR_CHAN_MAGN_Z:
		if ((drv_cfg->axis & TMAG5273_MAG_CH_EN_Z) != TMAG5273_MAG_CH_EN_Z) {
			return -ENOTSUP;
		}

		drv_data->int_threshold[TMAG5273_DRV_DATA_THRESHOLD_AXIS_Z] = val->val1;
		break;
	default:
		LOG_ERR("unsupported channel %d", chan);
		return -ENOTSUP;
	}

	return tmag527_trigger_attr_write_threshold(dev, chan);
}

static inline int tmag5273_trigger_attr_get_threshold(const struct device *dev,
						      enum sensor_channel chan,
						      struct sensor_value *val)
{
	const struct tmag5273_config *drv_cfg = dev->config;

	uint8_t data[3] = {0};
	uint8_t reg_mask = 0xFF;
	int retval;

	retval = i2c_reg_read_byte_dt(&drv_cfg->i2c, TMAG5273_REG_DEVICE_CONFIG_2, &data[0]);
	if (retval < 0) {
		LOG_ERR("error reading magnet threshold %d", retval);
		return retval;
	}

	if ((data[0] & TMAG5273_THR_HYST_MSK) == TMAG5273_THR_HYST_LSB_ONLY) {
		/* in LSB mode, get rid of MSB if present (ignored by the sensor) */
		reg_mask = 0x7F;
	}

	retval =
		i2c_burst_read_dt(&drv_cfg->i2c, TMAG5273_REG_X_THR_CONFIG, &data[0], sizeof(data));
	if (retval < 0) {
		return retval;
	}

	switch (chan) {
	case SENSOR_CHAN_MAGN_XYZ:
		for (size_t i = 0; i < sizeof(data); ++i) {
			val[i].val1 = tmag5273_trigger_calc_magn_threshold_value_from_regdata(
				dev, data[i] & reg_mask);
			val[i].val2 = 0;
		}
		break;
	case SENSOR_CHAN_MAGN_X:
		val->val1 = tmag5273_trigger_calc_magn_threshold_value_from_regdata(
			dev, data[0] & reg_mask);
		val->val2 = 0;
		break;
	case SENSOR_CHAN_MAGN_Y:
		val->val1 = tmag5273_trigger_calc_magn_threshold_value_from_regdata(
			dev, data[1] & reg_mask);
		val->val2 = 0;
		break;
	case SENSOR_CHAN_MAGN_Z:
		val->val1 = tmag5273_trigger_calc_magn_threshold_value_from_regdata(
			dev, data[2] & reg_mask);
		val->val2 = 0;
		break;
	default:
		LOG_ERR("unsupported channel %d", chan);
		return -ENOTSUP;
	}

	return 0;
}

static inline int tmag5273_trigger_attr_set_sleeptime(const struct device *dev,
						      enum sensor_channel chan,
						      const struct sensor_value *val)
{
	const struct tmag5273_config *drv_cfg = dev->config;

	int retval;
	uint8_t regdata;

	if (chan != SENSOR_CHAN_MAGN_XYZ) {
		LOG_ERR("unsupported channel %d", (int)chan);
		return -ENOTSUP;
	}

	if (val->val1 == 1) {
		regdata = TMAG5273_WS_SLEEPTIME_1MS;
	} else if (val->val1 <= 5) {
		regdata = TMAG5273_WS_SLEEPTIME_5MS;
	} else if (val->val1 <= 10) {
		regdata = TMAG5273_WS_SLEEPTIME_10MS;
	} else if (val->val1 <= 15) {
		regdata = TMAG5273_WS_SLEEPTIME_15MS;
	} else if (val->val1 <= 20) {
		regdata = TMAG5273_WS_SLEEPTIME_20MS;
	} else if (val->val1 <= 30) {
		regdata = TMAG5273_WS_SLEEPTIME_30MS;
	} else if (val->val1 <= 50) {
		regdata = TMAG5273_WS_SLEEPTIME_50MS;
	} else if (val->val1 <= 100) {
		regdata = TMAG5273_WS_SLEEPTIME_100MS;
	} else if (val->val1 <= 500) {
		regdata = TMAG5273_WS_SLEEPTIME_500MS;
	} else if (val->val1 <= 1000) {
		regdata = TMAG5273_WS_SLEEPTIME_1000MS;
	} else if (val->val1 <= 2000) {
		regdata = TMAG5273_WS_SLEEPTIME_2000MS;
	} else if (val->val1 <= 5000) {
		regdata = TMAG5273_WS_SLEEPTIME_5000MS;
	} else if (val->val1 <= 20000) {
		regdata = TMAG5273_WS_SLEEPTIME_20000MS;
	} else {
		LOG_ERR("invalid sleeptime %d", val->val1);
		return -ENOTSUP;
	}

	retval = i2c_reg_update_byte_dt(&drv_cfg->i2c, TMAG5273_REG_SENSOR_CONFIG_1,
					TMAG5273_SLEEPTIME_MSK, regdata);
	if (retval < 0) {
		LOG_ERR("error updating sleeptime %d", retval);
		return retval;
	}

	return 0;
}

static inline int tmag5273_trigger_attr_get_sleeptime(const struct device *dev,
						      enum sensor_channel chan,
						      struct sensor_value *val)
{
	const struct tmag5273_config *drv_cfg = dev->config;
	int retval;
	uint8_t regdata;

	if (chan != SENSOR_CHAN_MAGN_XYZ) {
		LOG_ERR("unsupported channel %d", (int)chan);
		return -ENOTSUP;
	}

	retval = i2c_reg_read_byte_dt(&drv_cfg->i2c, TMAG5273_REG_SENSOR_CONFIG_1, &regdata);
	if (retval < 0) {
		LOG_ERR("error reading SENSOR_CONFIG1 %d", retval);
		return retval;
	}

	switch (regdata & TMAG5273_SLEEPTIME_MSK) {
	case TMAG5273_WS_SLEEPTIME_1MS:
		val->val1 = 1;
		break;
	case TMAG5273_WS_SLEEPTIME_5MS:
		val->val1 = 5;
		break;
	case TMAG5273_WS_SLEEPTIME_10MS:
		val->val1 = 10;
		break;
	case TMAG5273_WS_SLEEPTIME_15MS:
		val->val1 = 15;
		break;
	case TMAG5273_WS_SLEEPTIME_20MS:
		val->val1 = 20;
		break;
	case TMAG5273_WS_SLEEPTIME_30MS:
		val->val1 = 30;
		break;
	case TMAG5273_WS_SLEEPTIME_50MS:
		val->val1 = 50;
		break;
	case TMAG5273_WS_SLEEPTIME_100MS:
		val->val1 = 100;
		break;
	case TMAG5273_WS_SLEEPTIME_500MS:
		val->val1 = 500;
		break;
	case TMAG5273_WS_SLEEPTIME_1000MS:
		val->val1 = 1000;
		break;
	case TMAG5273_WS_SLEEPTIME_2000MS:
		val->val1 = 2000;
		break;
	case TMAG5273_WS_SLEEPTIME_5000MS:
		val->val1 = 5000;
		break;
	case TMAG5273_WS_SLEEPTIME_20000MS:
		val->val1 = 20000;
		break;
	default:
		LOG_ERR("invalid value read from sensor 0x%X",
			(int)(regdata & TMAG5273_SLEEPTIME_MSK));
		return -EINVAL;
	}

	val->val2 = 0;

	return 0;
}

static inline int tmag5273_trigger_attr_set_threshold_limit(const struct device *dev,
							    enum sensor_channel chan,
							    const struct sensor_value *val)
{
	const struct tmag5273_config *drv_cfg = dev->config;

	int retval;

	uint8_t thr_hyst;
	uint8_t mag_thr_dir;

	if (chan != SENSOR_CHAN_MAGN_XYZ) {
		LOG_ERR("unsupported channel %d", (int)chan);
		return -ENOTSUP;
	}

	switch (val->val1) {
	case TMAG5273_THRESHOLD_LIMIT_ABOVE:
		thr_hyst = TMAG5273_THR_HYST_COMPLEMENT;
		mag_thr_dir = TMAG5273_MAG_THR_DIRECTION_ABOVE;
		break;
	case TMAG5273_THRESHOLD_LIMIT_BELOW:
		thr_hyst = TMAG5273_THR_HYST_COMPLEMENT;
		mag_thr_dir = TMAG5273_MAG_THR_DIRECTION_BELOW;
		break;
	case TMAG5273_THRESHOLD_LIMIT_OUT_OF_BAND:
		thr_hyst = TMAG5273_THR_HYST_LSB_ONLY;
		mag_thr_dir = TMAG5273_MAG_THR_DIRECTION_ABOVE;
		break;
	case TMAG5273_THRESHOLD_LIMIT_INSIDE_BAND:
		thr_hyst = TMAG5273_THR_HYST_LSB_ONLY;
		mag_thr_dir = TMAG5273_MAG_THR_DIRECTION_BELOW;
		break;
	default:
		LOG_ERR("unknown threshold limit value %d", val->val1);
		return -ENOTSUP;
	}

	retval = i2c_reg_update_byte_dt(&drv_cfg->i2c, TMAG5273_REG_DEVICE_CONFIG_2,
					TMAG5273_THR_HYST_MSK, thr_hyst);
	if (retval < 0) {
		LOG_ERR("error updating magnitude threshold direction %d", retval);
		return retval;
	}

	retval = i2c_reg_update_byte_dt(&drv_cfg->i2c, TMAG5273_REG_SENSOR_CONFIG_2,
					TMAG5273_MAG_THR_DIRECTION_MSK, mag_thr_dir);
	if (retval < 0) {
		LOG_ERR("error updating threshold hysteresis %d", retval);
		return retval;
	}

	/* update thresholds accordingly */
	retval = tmag527_trigger_attr_write_threshold(dev, SENSOR_CHAN_MAGN_XYZ);
	if (retval < 0) {
		LOG_ERR("error updating threshold values %d", retval);
		return retval;
	}

	return 0;
}

static inline int tmag5273_trigger_attr_get_threshold_limit(const struct device *dev,
							    enum sensor_channel chan,
							    struct sensor_value *val)
{
	const struct tmag5273_config *drv_cfg = dev->config;

	int retval;
	uint8_t thr_hyst;
	uint8_t mag_thr_dir;

	if (chan != SENSOR_CHAN_MAGN_XYZ) {
		LOG_ERR("unsupported channel %d", (int)chan);
		return -ENOTSUP;
	}

	retval = i2c_reg_read_byte_dt(&drv_cfg->i2c, TMAG5273_REG_DEVICE_CONFIG_2, &thr_hyst);
	if (retval < 0) {
		LOG_ERR("error reading magnet threshold %d", retval);
		return retval;
	}

	retval = i2c_reg_read_byte_dt(&drv_cfg->i2c, TMAG5273_REG_SENSOR_CONFIG_2, &mag_thr_dir);
	if (retval < 0) {
		LOG_ERR("error reading threshold hysteresis %d", retval);
		return retval;
	}

	switch (thr_hyst & TMAG5273_THR_HYST_MSK) {
	case TMAG5273_THR_HYST_COMPLEMENT:
		val->val1 = ((mag_thr_dir & TMAG5273_MAG_THR_DIRECTION_MSK) ==
			     TMAG5273_MAG_THR_DIRECTION_ABOVE)
				    ? TMAG5273_THRESHOLD_LIMIT_ABOVE
				    : TMAG5273_THRESHOLD_LIMIT_BELOW;
		break;
	case TMAG5273_THR_HYST_LSB_ONLY:
		val->val1 = ((mag_thr_dir & TMAG5273_MAG_THR_DIRECTION_MSK) ==
			     TMAG5273_MAG_THR_DIRECTION_ABOVE)
				    ? TMAG5273_THRESHOLD_LIMIT_OUT_OF_BAND
				    : TMAG5273_THRESHOLD_LIMIT_INSIDE_BAND;
		break;
	default:
		LOG_ERR("option not supported by driver %u", thr_hyst);
		return -EIO;
	}

	val->val2 = 0;

	return 0;
}

static inline int tmag5273_trigger_attr_set_threshold_count(const struct device *dev,
							    enum sensor_channel chan,
							    const struct sensor_value *val)
{
	const struct tmag5273_config *drv_cfg = dev->config;

	int retval;
	uint8_t regdata;

	if (chan != SENSOR_CHAN_MAGN_XYZ) {
		LOG_ERR("unsupported channel %d", (int)chan);
		return -ENOTSUP;
	}

	switch (val->val1) {
	case 1:
		regdata = TMAG5273_THRX_COUNT_1;
		break;
	case 4:
		regdata = TMAG5273_THRX_COUNT_4;
		break;
	default:
		LOG_ERR("invalid option %d", val->val1);
		return -EINVAL;
	}

	retval = i2c_reg_update_byte_dt(&drv_cfg->i2c, TMAG5273_REG_SENSOR_CONFIG_2,
					TMAG5273_THRX_COUNT_MSK, regdata);
	if (retval < 0) {
		LOG_ERR("error updating threshold count %d", retval);
		return retval;
	}

	return 0;
}

static inline int tmag5273_trigger_attr_get_threshold_count(const struct device *dev,
							    enum sensor_channel chan,
							    struct sensor_value *val)
{
	const struct tmag5273_config *drv_cfg = dev->config;

	int retval;
	uint8_t regdata;

	if (chan != SENSOR_CHAN_MAGN_XYZ) {
		LOG_ERR("unsupported channel %d", (int)chan);
		return -ENOTSUP;
	}

	retval = i2c_reg_read_byte_dt(&drv_cfg->i2c, TMAG5273_REG_SENSOR_CONFIG_2, &regdata);
	if (retval < 0) {
		LOG_ERR("error updating threshold count %d", retval);
		return retval;
	}

	val->val1 = ((regdata & TMAG5273_THRX_COUNT_MSK) == TMAG5273_THRX_COUNT_1) ? 1 : 4;
	val->val2 = 0;

	return 0;
}

int tmag5273_trigger_attr_set(const struct device *dev, enum sensor_channel chan,
			      enum sensor_attribute attr, const struct sensor_value *val)
{
	int retval;

	switch ((int)attr) {
	case SENSOR_ATTR_UPPER_THRESH:
	case SENSOR_ATTR_LOWER_THRESH:
		retval = tmag5273_trigger_attr_set_threshold(dev, chan, val);
		if (retval < 0) {
			return retval;
		}
		break;
	case TMAG5273_ATTR_SLEEPTIME:
		retval = tmag5273_trigger_attr_set_sleeptime(dev, chan, val);
		if (retval < 0) {
			return retval;
		}
		break;
	case TMAG5273_ATTR_THRESHOLD_COUNT:
		retval = tmag5273_trigger_attr_set_threshold_count(dev, chan, val);
		if (retval < 0) {
			return retval;
		}
		break;
	case TMAG5273_ATTR_THRESHOLD_LIMIT:
		retval = tmag5273_trigger_attr_set_threshold_limit(dev, chan, val);
		if (retval < 0) {
			return retval;
		}
		break;
	default:
		LOG_ERR("unsupported attribute: %d", (int)attr);
		return -ENOTSUP;
	}

	return 0;
}

int tmag5273_trigger_attr_get(const struct device *dev, enum sensor_channel chan,
			      enum sensor_attribute attr, struct sensor_value *val)
{
	int retval;

	switch ((int)attr) {
	case SENSOR_ATTR_UPPER_THRESH:
	case SENSOR_ATTR_LOWER_THRESH:
		retval = tmag5273_trigger_attr_get_threshold(dev, chan, val);
		if (retval < 0) {
			return retval;
		}
		break;
	case TMAG5273_ATTR_SLEEPTIME:
		retval = tmag5273_trigger_attr_get_sleeptime(dev, chan, val);
		if (retval < 0) {
			return retval;
		}
		break;
	case TMAG5273_ATTR_THRESHOLD_COUNT:
		retval = tmag5273_trigger_attr_get_threshold_count(dev, chan, val);
		if (retval < 0) {
			return retval;
		}
		break;
	case TMAG5273_ATTR_THRESHOLD_LIMIT:
		retval = tmag5273_trigger_attr_get_threshold_limit(dev, chan, val);
		if (retval < 0) {
			return retval;
		}
		break;
	default:
		LOG_ERR("unknown attribute %d", (int)attr);
		return -ENOTSUP;
	}

	return 0;
}

/**
 * @brief callback called if a GPIO-interrupt is registered
 *
 * @param port device struct for the GPIO device
 * @param cb @ref struct gpio_callback owning this handler
 * @param pins mask of pins that triggered the callback handler
 */
static void tmag5273_gpio_callback(const struct device *port, struct gpio_callback *cb,
				   uint32_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	if (!cb) {
		LOG_ERR("no callback for interrupt");
		return;
	}

	struct tmag5273_data *drv_data = CONTAINER_OF(cb, struct tmag5273_data, int_callback);
	const struct tmag5273_config *drv_cfg = drv_data->dev->config;
	int retval;

	retval = gpio_pin_interrupt_configure_dt(&drv_cfg->int_gpio, GPIO_INT_DISABLE);
	if (retval < 0) {
		LOG_ERR("error deactivating GPIO %d", retval);
	}

#ifdef CONFIG_TMAG5273_TRIGGER_OWN_THREAD

	k_sem_give(&drv_data->gpio_sem);

#elif CONFIG_TMAG5273_TRIGGER_GLOBAL_THREAD

	k_work_submit(&drv_data->work);

#endif
}

static void tmag5273_thread_cb(const struct device *dev)
{
	const struct tmag5273_config *drv_cfg = dev->config;
	struct tmag5273_data *drv_data = dev->data;
	int retval;

	atomic_set_bit(&drv_data->on_interrupt, TMAG5273_ON_INTERRUPT_BIT);

#if CONFIG_PM_DEVICE
	if (drv_cfg->pm_int_suspend_to_wakeup_sleep) {
		retval = pm_device_state_get(dev, &pm_state);
		if (retval < 0) {
			LOG_ERR("cannot read pm device state %d", retval);
		}

		if (pm_state == PM_DEVICE_STATE_SUSPENDED) {
			/* wakeup & sleep interrupt: if the user sets the PM state to "suspend" in
			 * the callback, another interrupt might be issued so the sensor is again
			 * not sleeping */
			pm_device_action_run(dev, PM_DEVICE_ACTION_RESUME);

			retval = gpio_pin_interrupt_configure_dt(&drv_cfg->int_gpio,
								 GPIO_INT_EDGE_FALLING);
			if (retval < 0) {
				LOG_ERR("error activating SoC interrupt %d", retval);
			}
		}
	}
#endif

	if (drv_data->int_handler != NULL) {
		drv_data->int_handler(dev, drv_data->int_trigger);
	}

#if CONFIG_PM_DEVICE
	/* set correct mode if wakeup-&-sleep is active */
	if (drv_cfg->pm_int_suspend_to_wakeup_sleep) {
		enum pm_device_state pm_state;

		retval = pm_device_state_get(dev, &pm_state);
		if (retval < 0) {
			LOG_ERR("cannot read pm device state %d", retval);
		}

		if (pm_state == PM_DEVICE_STATE_ACTIVE) {
			/* force correct mode (e.g. continuous) after user callback */
			tmag5273_pm_set_operation_mode(dev, PM_DEVICE_ACTION_RESUME);
		}
	}
#endif

	atomic_clear_bit(&drv_data->on_interrupt, TMAG5273_ON_INTERRUPT_BIT);

	retval = gpio_pin_interrupt_configure_dt(&drv_cfg->int_gpio, GPIO_INT_EDGE_FALLING);
	if (retval < 0) {
		LOG_ERR("error activating SoC interrupt %d", retval);
	}

	/* read device status register to clear a latching interrupt */
	if (drv_cfg->int_latched) {
		(void)tmag5273_clear_latching_interrupt(dev);
	}
}

#ifdef CONFIG_TMAG5273_TRIGGER_OWN_THREAD
static void tmag5273_thread(struct tmag5273_data *drv_data)
{
	while (1) {
		k_sem_take(&drv_data->gpio_sem, K_FOREVER);
		tmag5273_thread_cb(drv_data->dev);
	}
}
#endif

#ifdef CONFIG_TMAG5273_TRIGGER_GLOBAL_THREAD
static void tmag5273_work_cb(struct k_work *work)
{
	struct tmag5273_data *drv_data = CONTAINER_OF(work, struct tmag5273_data, work);

	tmag5273_thread_cb(drv_data->dev);
}
#endif

int tmag5273_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
			 sensor_trigger_handler_t handler)
{
	CHECKIF(dev == NULL) {
		LOG_ERR("dev: NULL");
		return -EINVAL;
	}

	CHECKIF(trig == NULL) {
		LOG_ERR("trig: NULL");
		return -EINVAL;
	}

	int retval;
	uint8_t regdata;

#ifdef CONFIG_PM_DEVICE
	retval = tmag5273_pm_is_modifiable(dev);
	if (retval != 0) {
		return retval;
	}
#endif

	const struct tmag5273_config *drv_cfg = dev->config;
	struct tmag5273_data *drv_data = dev->data;

	retval = gpio_pin_interrupt_configure_dt(&drv_cfg->int_gpio, GPIO_INT_DISABLE);
	if (retval < 0) {
		LOG_ERR("error changing interrupt config on SoC %d", retval);
		return -EIO;
	}

	if (trig->chan != SENSOR_CHAN_MAGN_XYZ) {
		LOG_ERR("trigger channel %d not supported", (int)trig->chan);
		return -ENOTSUP;
	}

	if (handler != NULL) {
		switch (trig->type) {
		case SENSOR_TRIG_DATA_READY:
			regdata = TMAG5273_RSLT_INT_ENABLED;
			break;
		case SENSOR_TRIG_THRESHOLD:
			regdata = TMAG5273_THRSLD_INT_ENABLED;
			break;
		default:
			LOG_ERR("unknown trigger type %d", trig->type);
			return -ENOTSUP;
		}

		switch (drv_cfg->int_mode) {
		case TMAG5273_DT_INT_THROUGH_INT:
			regdata |= TMAG5273_INT_MODE_INT;
			break;
		case TMAG5273_DT_INT_THROUGH_INT_EXC_I2C:
			regdata |= TMAG5273_INT_MODE_INT_EXC_I2C;
			break;
		default:
			LOG_ERR("unsupported interrupt mode %d", drv_cfg->int_mode);
			return -ENOTSUP;
		}

		retval = gpio_pin_interrupt_configure_dt(&drv_cfg->int_gpio, GPIO_INT_EDGE_FALLING);
		if (retval < 0) {
			LOG_ERR("error changing interrupt config on SoC %d", retval);
			return -EIO;
		}
	} else {
		regdata = 0;
		trig = NULL;
	}

	drv_data->int_trigger = trig;
	drv_data->int_handler = handler;

	/* deactivate old interrupt and install new */
	retval = i2c_reg_update_byte_dt(&drv_cfg->i2c, TMAG5273_REG_INT_CONFIG_1,
					TMAG5273_INT_MODE_MSK | TMAG5273_RSLT_THRSLD_INT_MSK,
					regdata);
	if (retval < 0) {
		LOG_ERR("error enabling conversion complete interrupt %d", retval);
		return retval;
	}

	retval = tmag5273_clear_latching_interrupt(dev);
	if (retval < 0) {
		return retval;
	}

	return 0;
}

int tmag5273_trigger_init(const struct device *dev)
{
	CHECKIF(dev == NULL) {
		LOG_ERR("dev: NULL");
		return -EINVAL;
	}

	const struct tmag5273_config *drv_cfg = dev->config;
	struct tmag5273_data *drv_data = dev->data;

	uint8_t regdata;
	int retval;

	drv_data->dev = dev;
	drv_data->on_interrupt = ATOMIC_INIT(TMAG5273_ON_INTERRUPT_BIT);

	/* REG_INT_CONFIG_1 */
	regdata = 0;

	regdata |= TMAG5273_RSLT_INT_DISABLED;
	regdata |= TMAG5273_THRSLD_INT_DISABLED;

	if (!drv_cfg->int_latched) {
		regdata |= TMAG5273_INT_STATE_PULSE;
	}

	retval = i2c_reg_write_byte_dt(&drv_cfg->i2c, TMAG5273_REG_INT_CONFIG_1, regdata);
	if (retval < 0) {
		LOG_ERR("error writing INT_CONFIG_1 register %d", retval);
		return -EIO;
	}

	/* set up working queues */
#ifdef CONFIG_TMAG5273_TRIGGER_OWN_THREAD
	k_sem_init(&drv_data->gpio_sem, 0, K_SEM_MAX_LIMIT);

	k_thread_create(&drv_data->thread, drv_data->thread_stack,
			CONFIG_TMAG5273_THREAD_STACK_SIZE, (k_thread_entry_t)tmag5273_thread,
			drv_data, NULL, NULL, K_PRIO_COOP(CONFIG_TMAG5273_THREAD_PRIORITY), 0,
			K_NO_WAIT);
#elif CONFIG_TMAG5273_TRIGGER_GLOBAL_THREAD
	drv_data->work.handler = tmag5273_work_cb;
#else
#error "invalid interrupt threading configuration"
#endif

	/* configure GPIO interrupt on SoC */
	gpio_init_callback(&drv_data->int_callback, tmag5273_gpio_callback,
			   BIT(drv_cfg->int_gpio.pin));

	retval = gpio_add_callback(drv_cfg->int_gpio.port, &drv_data->int_callback);
	if (retval < 0) {
		LOG_ERR("Could not set gpio callback %d", retval);
		return -EIO;
	}

	retval = gpio_pin_interrupt_configure_dt(&drv_cfg->int_gpio, GPIO_INT_DISABLE);
	if (retval < 0) {
		LOG_ERR("could not set interrupt pin");
		return -EIO;
	}

	return 0;
}

bool tmag5273_on_interrupt_handling(const struct device *dev)
{
	const struct tmag5273_data *drv_data = dev->data;
	return atomic_test_bit(&drv_data->on_interrupt, TMAG5273_ON_INTERRUPT_BIT);
}
