/*
 * Copyright (c) 2023 deveritec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_TMAG5273_H_
#define ZEPHYR_DRIVERS_SENSOR_TMAG5273_H_

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

/* --- Register definitions --- */
#define TMAG5273_REG_DEVICE_CONFIG_1		0x00
#define TMAG5273_REG_DEVICE_CONFIG_2		0x01
#define TMAG5273_REG_SENSOR_CONFIG_1		0x02
#define TMAG5273_REG_SENSOR_CONFIG_2		0x03
#define TMAG5273_REG_X_THR_CONFIG		0x04
#define TMAG5273_REG_Y_THR_CONFIG		0x05
#define TMAG5273_REG_Z_THR_CONFIG		0x06
#define TMAG5273_REG_T_CONFIG			0x07
#define TMAG5273_REG_INT_CONFIG_1		0x08
#define TMAG5273_REG_MAG_GAIN_CONFIG		0x09
#define TMAG5273_REG_MAG_OFFSET_CONFIG_1	0x0A
#define TMAG5273_REG_MAG_OFFSET_CONFIG_2	0x0B
#define TMAG5273_REG_I2C_ADDRESS		0x0C
#define TMAG5273_REG_DEVICE_ID			0x0D
#define TMAG5273_REG_MANUFACTURER_ID_LSB	0x0E
#define TMAG5273_REG_MANUFACTURER_ID_MSB	0x0F
#define TMAG5273_REG_T_MSB_RESULT		0x10
#define TMAG5273_REG_T_LSB_RESULT		0x11
#define TMAG5273_REG_X_MSB_RESULT		0x12
#define TMAG5273_REG_X_LSB_RESULT		0x13
#define TMAG5273_REG_Y_MSB_RESULT		0x14
#define TMAG5273_REG_Y_LSB_RESULT		0x15
#define TMAG5273_REG_Z_MSB_RESULT		0x16
#define TMAG5273_REG_Z_LSB_RESULT		0x17
#define TMAG5273_REG_CONV_STATUS		0x18
#define TMAG5273_REG_ANGLE_MSB_RESULT		0x19
#define TMAG5273_REG_ANGLE_LSB_RESULT		0x1A
#define TMAG5273_REG_MAGNITUDE_RESULT		0x1B
#define TMAG5273_REG_DEVICE_STATUS		0x1C

#define TMAG5273_REG_RESULT_BEGIN	(TMAG5273_REG_T_MSB_RESULT)
#define TMAG5273_REG_RESULT_END		(TMAG5273_REG_MAGNITUDE_RESULT)

/* Register DEVICE_CONFIG_1 */
#define TMAG5273_CRC_EN_POS	7
#define TMAG5273_MAG_TEMPCO_POS	5
#define TMAG5273_CONV_AVG_POS	2
#define TMAG5273_I2C_READ_POS	0

#define TMAG5273_CONV_AVB_MSK	GENMASK(4, 2)

#define TMAG5273_CRC_DISABLE	(0 << TMAG5273_CRC_EN_POS)
#define TMAG5273_CRC_ENABLE	(1 << TMAG5273_CRC_EN_POS)

#define TMAG5273_MAGNET_TEMP_COEFF_NONE		(0 << TMAG5273_MAG_TEMPCO_POS)
#define TMAG5273_MAGNET_TEMP_COEFF_NDBFE	(1 << TMAG5273_MAG_TEMPCO_POS)
#define TMAG5273_MAGNET_TEMP_COEFF_CERAMIC	(3 << TMAG5273_MAG_TEMPCO_POS)

#define TMAG5273_CONV_AVG_1	(0 << TMAG5273_CONV_AVG_POS)
#define TMAG5273_CONV_AVG_2	(1 << TMAG5273_CONV_AVG_POS)
#define TMAG5273_CONV_AVG_4	(2 << TMAG5273_CONV_AVG_POS)
#define TMAG5273_CONV_AVG_8	(3 << TMAG5273_CONV_AVG_POS)
#define TMAG5273_CONV_AVG_16	(4 << TMAG5273_CONV_AVG_POS)
#define TMAG5273_CONV_AVG_32	(5 << TMAG5273_CONV_AVG_POS)

#define TMAG5273_I2C_READ_MODE_STANDARD		(0 << TMAG5273_I2C_READ_POS)
#define TMAG5273_I2C_READ_MODE_16BIT_SENSOR	(1 << TMAG5273_I2C_READ_POS)
#define TMAG5273_I2C_READ_MODE_8BIT_MSB_DATA	(2 << TMAG5273_I2C_READ_POS)

/* Register DEVICE_CONFIG_2 */
#define TMAG5273_THR_HYST_POS		5
#define TMAG5273_LP_LN_POS		4
#define TMAG5273_I2C_GLITCH_FILTER_POS	3
#define TMAG5273_TRIGGER_MODE_POS	2
#define TMAG5273_OPERATING_MODE_POS	0

#define TMAG5273_THR_HYST_MSK		GENMASK(7, 5)
#define TMAG5273_OPERATING_MODE_MSK	GENMASK(1, 0)

#define TMAG5273_THR_HYST_COMPLEMENT	(0 << TMAG5273_THR_HYST_POS)
#define TMAG5273_THR_HYST_LSB_ONLY	(1 << TMAG5273_THR_HYST_POS)

#define TMAG5273_LP_LOWPOWER	(0 << TMAG5273_LP_LN_POS)
#define TMAG5273_LP_LOWNOISE	(1 << TMAG5273_LP_LN_POS)

#define TMAG5273_I2C_GLITCH_FILTER_ON	(0 << TMAG5273_I2C_GLITCH_FILTER_POS)
#define TMAG5273_I2C_GLITCH_FILTER_OFF	(1 << TMAG5273_I2C_GLITCH_FILTER_POS)

#define TMAG5273_TRIGGER_MODE_I2C	(0 << TMAG5273_TRIGGER_MODE_POS)
#define TMAG5273_TRIGGER_MODE_INT	(1 << TMAG5273_TRIGGER_MODE_POS)

#define TMAG5273_OPERATING_MODE_STANDBY		(0 << TMAG5273_OPERATING_MODE_POS)
#define TMAG5273_OPERATING_MODE_SLEEP		(1 << TMAG5273_OPERATING_MODE_POS)
#define TMAG5273_OPERATING_MODE_CONTINUOUS	(2 << TMAG5273_OPERATING_MODE_POS)
#define TMAG5273_OPERATING_MODE_WAKEUP_SLEEP	(3 << TMAG5273_OPERATING_MODE_POS)

/* Register SENSOR_CONFIG_1 */
#define TMAG5273_MAG_CH_EN_POS	4
#define TMAG5273_SLEEPTIME_POS	0

#define TMAG5273_SLEEPTIME_MSK	GENMASK(3, 0)
#define TMAG5273_MAG_CH_EN_NONE	(0x0 << TMAG5273_MAG_CH_EN_POS)
#define TMAG5273_MAG_CH_EN_X	(0x1 << TMAG5273_MAG_CH_EN_POS)
#define TMAG5273_MAG_CH_EN_Y	(0x2 << TMAG5273_MAG_CH_EN_POS)
#define TMAG5273_MAG_CH_EN_Z	(0x4 << TMAG5273_MAG_CH_EN_POS)

#define TMAG5273_WS_SLEEPTIME_1MS	(0x0 << TMAG5273_SLEEPTIME_POS)
#define TMAG5273_WS_SLEEPTIME_5MS	(0x1 << TMAG5273_SLEEPTIME_POS)
#define TMAG5273_WS_SLEEPTIME_10MS	(0x2 << TMAG5273_SLEEPTIME_POS)
#define TMAG5273_WS_SLEEPTIME_15MS	(0x3 << TMAG5273_SLEEPTIME_POS)
#define TMAG5273_WS_SLEEPTIME_20MS	(0x4 << TMAG5273_SLEEPTIME_POS)
#define TMAG5273_WS_SLEEPTIME_30MS	(0x5 << TMAG5273_SLEEPTIME_POS)
#define TMAG5273_WS_SLEEPTIME_50MS	(0x6 << TMAG5273_SLEEPTIME_POS)
#define TMAG5273_WS_SLEEPTIME_100MS	(0x7 << TMAG5273_SLEEPTIME_POS)
#define TMAG5273_WS_SLEEPTIME_500MS	(0x8 << TMAG5273_SLEEPTIME_POS)
#define TMAG5273_WS_SLEEPTIME_1000MS	(0x9 << TMAG5273_SLEEPTIME_POS)
#define TMAG5273_WS_SLEEPTIME_2000MS	(0xA << TMAG5273_SLEEPTIME_POS)
#define TMAG5273_WS_SLEEPTIME_5000MS	(0xB << TMAG5273_SLEEPTIME_POS)
#define TMAG5273_WS_SLEEPTIME_20000MS	(0xC << TMAG5273_SLEEPTIME_POS)

/* Register SENSOR_CONFIG_2 */
#define TMAG5273_THRX_COUNT_POS		6
#define TMAG5273_MAG_THR_DIR_POS	5
#define TMAG5273_GAIN_CORRECTION_CH_POS	4
#define TMAG5273_ANGLE_EN_POS		2
#define TMAG5273_X_Y_RANGE_POS		1
#define TMAG5273_Z_RANGE_POS		0

#define TMAG5273_THRX_COUNT_MSK		GENMASK(6, 6)
#define TMAG5273_MAG_THR_DIRECTION_MSK	GENMASK(5, 5)
#define TMAG5273_ANGLE_EN_MSK		GENMASK(3, 2)
#define TMAG5273_MEAS_RANGE_X_Y_MSK	GENMASK(1, 1)
#define TMAG5273_MEAS_RANGE_Z_MSK	GENMASK(0, 0)
#define TMAG5273_MEAS_RANGE_XYZ_MSK	(TMAG5273_MEAS_RANGE_X_Y_MSK | TMAG5273_MEAS_RANGE_Z_MSK)

#define TMAG5273_THRX_COUNT_1	(0 << TMAG5273_THRX_COUNT_POS)
#define TMAG5273_THRX_COUNT_4	(1 << TMAG5273_THRX_COUNT_POS)

#define TMAG5273_MAG_THR_DIRECTION_ABOVE	(0 << TMAG5273_MAG_THR_DIR_POS)
#define TMAG5273_MAG_THR_DIRECTION_BELOW	(1 << TMAG5273_MAG_THR_DIR_POS)

#define TMAG5273_MAG_GAIN_CORRECTION_CH_1	(0 << TMAG5273_GAIN_CORRECTION_CH_POS)
#define TMAG5273_MAG_GAIN_CORRECTION_CH_2	(1 << TMAG5273_GAIN_CORRECTION_CH_POS)

#define TMAG5273_ANGLE_EN_NONE	(0 << TMAG5273_ANGLE_EN_POS)
#define TMAG5273_ANGLE_EN_XY	(1 << TMAG5273_ANGLE_EN_POS)
#define TMAG5273_ANGLE_EN_YZ	(2 << TMAG5273_ANGLE_EN_POS)
#define TMAG5273_ANGLE_EN_XZ	(3 << TMAG5273_ANGLE_EN_POS)

#define TMAG5273_X_Y_MEAS_RANGE_LOW	(0 << TMAG5273_X_Y_RANGE_POS)
#define TMAG5273_X_Y_MEAS_RANGE_HIGH	(1 << TMAG5273_X_Y_RANGE_POS)

#define TMAG5273_Z_MEAS_RANGE_LOW	(0 << TMAG5273_Z_RANGE_POS)
#define TMAG5273_Z_MEAS_RANGE_HIGH	(1 << TMAG5273_Z_RANGE_POS)

#define TMAG5273_XYZ_MEAS_RANGE_LOW	(TMAG5273_X_Y_MEAS_RANGE_LOW | TMAG5273_Z_MEAS_RANGE_LOW)
#define TMAG5273_XYZ_MEAS_RANGE_HIGH	(TMAG5273_X_Y_MEAS_RANGE_HIGH | TMAG5273_Z_MEAS_RANGE_HIGH)

/* Register T_CONFIG */
#define TMAG5273_T_THR_CONFIG_POS	1
#define TMAG5273_T_CH_EN_POS		0

#define TMAG5273_T_CH_EN_DISABLED	(0 << TMAG5273_T_CH_EN_POS)
#define TMAG5273_T_CH_EN_ENABLED	(1 << TMAG5273_T_CH_EN_POS)

/* Register INT_CONFIG_1 */
#define TMAG5273_RSLT_INT_POS	7
#define TMAG5273_THRSLD_INT_POS	6
#define TMAG5273_INT_STATE_POS	5
#define TMAG5273_INT_MODE_POS	2
#define TMAG5273_MASK_INTB_POS	0

#define TMAG5273_RSLT_THRSLD_INT_MSK	GENMASK(7, 6)
#define TMAG5273_INT_MODE_MSK		GENMASK(4, 2)
#define TMAG5273_MASK_INTB_MSK		GENMASK(0, 0)

#define TMAG5273_RSLT_INT_DISABLED	(0 << TMAG5273_RSLT_INT_POS)
#define TMAG5273_RSLT_INT_ENABLED	(1 << TMAG5273_RSLT_INT_POS)

#define TMAG5273_THRSLD_INT_DISABLED	(0 << TMAG5273_THRSLD_INT_POS)
#define TMAG5273_THRSLD_INT_ENABLED	(1 << TMAG5273_THRSLD_INT_POS)

#define TMAG5273_INT_STATE_LATCHED	(0 << TMAG5273_INT_STATE_POS)
#define TMAG5273_INT_STATE_PULSE	(1 << TMAG5273_INT_STATE_POS)

#define TMAG5273_INT_MODE_NONE		(0 << TMAG5273_INT_MODE_POS)
#define TMAG5273_INT_MODE_INT		(1 << TMAG5273_INT_MODE_POS)
#define TMAG5273_INT_MODE_INT_EXC_I2C	(2 << TMAG5273_INT_MODE_POS)
#define TMAG5273_INT_MODE_SCL		(3 << TMAG5273_INT_MODE_POS)
#define TMAG5273_INT_MODE_SCL_EXC_I2C	(4 << TMAG5273_INT_MODE_POS)

#define TMAG5273_MASK_INTB_PIN_ENABLED	(0 << TMAG5273_MASK_INTB_POS)
#define TMAG5273_MASK_INTB_PIN_MASKED	(1 << TMAG5273_MASK_INTB_POS)

/* Register I2C_ADDRESS */
#define TMAG5273_I2C_ADDRESS_POS		1
#define TMAG5273_I2C_ADDRESS_UPDATE_EN_POS	0

#define TMAG5273_I2C_ADDRESS_MSK	GENMASK(7, 1)

#define TMAG5273_I2C_ADDRESS_UPDATE_DISABLE	(0 << TMAG5273_I2C_ADDRESS_UPDATE_EN_POS)
#define TMAG5273_I2C_ADDRESS_UPDATE_ENABLE	(1 << TMAG5273_I2C_ADDRESS_UPDATE_EN_POS)

/* Register DEVICE_ID */
#define TMAG5273_VER_POS	0
#define TMAG5273_VER_MSK	GENMASK(1, 0)

#define TMAG5273_VER_TMAG5273X1	(1 << TMAG5273_VER_POS)
#define TMAG5273_VER_TMAG5273X2	(2 << TMAG5273_VER_POS)

/* Register CONV_STATUS */
#define TMAG5273_SET_COUNT_POS		5
#define TMAG5273_POR_POS		4
#define TMAG5273_DIAG_STATUS_POS	1
#define TMAG5273_RESULT_STATUS_POS	0

#define TMAG5273_DIAG_STATUS_MSK	GENMASK(1, 1)
#define TMAG5273_RESULT_STATUS_MSK	GENMASK(0, 0)

#define TMAG5273_POR_OCCURRED		(1 << TMAG5273_POR_POS)
#define TMAG5273_DIAG_FAIL		(1 << TMAG5273_DIAG_STATUS_POS)
#define TMAG5273_CONVERSION_COMPLETE	(1 << TMAG5273_RESULT_STATUS_POS)

/* Register DEVICE_STATUS */
#define TMAG5273_INTB_RB_POS	4
#define TMAG5273_OSC_ER_POS	3
#define TMAG5273_INT_ER_POS	2
#define TMAG5273_OTP_CRC_ER_POS	1
#define TMAG5273_VCC_UV_ER_POS	0

#define TMAG5273_INTB_RB_MSK	GENMASK(4, 4)
#define TMAG5273_OSC_ER_MSK	GENMASK(3, 3)
#define TMAG5273_INT_ER_MSK	GENMASK(2, 2)
#define TMAG5273_OTP_CRC_ER_MSK	GENMASK(1, 1)
#define TMAG5273_VCC_UV_ER_MSK	GENMASK(0, 0)

#define TMAG5273_INTB_PIN_HIGH	(1 << TMAG5273_INTB_RB_POS)
#define TMAG5273_OSC_ERR	(1 << TMAG5273_OSC_ER_POS)
#define TMAG5273_INT_ERR	(1 << TMAG5273_INT_ER_POS)
#define TMAG5273_OTP_CRC_ERR	(1 << TMAG5273_OTP_CRC_ER_POS)
#define TMAG5273_VCC_UV_ERR	(1 << TMAG5273_VCC_UV_ER_POS)

#define TMAG5273_RESET_DEVICE_STATUS	0xF

/* additional values */
#define TMAG5273_MANUFACTURER_ID_MSB	0x54
#define TMAG5273_MANUFACTURER_ID_LSB	0x49

#define TMAG5273_MEAS_RANGE_LOW_MT_VER1		40
#define TMAG5273_MEAS_RANGE_HIGH_MT_VER1	80
#define TMAG5273_MEAS_RANGE_LOW_MT_VER2		133
#define TMAG5273_MEAS_RANGE_HIGH_MT_VER2	266

#define TMAG5273_TEMPERATURE_T_SENS_T0	25
#define TMAG5273_TEMPERATURE_T_ADC_T0	17508
#define TMAG5273_TEMPERATURE_T_ADC_RES	60.1

#define TMAG5273_T_TO_SLEEP_US	50
#define TMAG5273_T_WAKEUP_US	20

#define TMAG5273_CONV_FACTOR_MT_TO_GS	10
#define TMAG5273_CONV_FACTOR_GS_TO_MT	1 / TMAG5273_CONV_FACTOR_MT_TO_GS

/**
 * @brief calculate conversion time as defined in the datasheet
 * @param avg set averaging value
 * @param nb_channels number of captured channels
 */
#define TMAG5273_T_CONVERSION_US(avg, nb_channels)	(((1 << avg) * 25) * nb_channels + 25)

/** OR this bit to any register address to trigger a conversion in standby mode */
#define TMAG5273_CONVERSION_START_BIT	0x80

enum tmag5273_drv_data_threshold_axis {
	TMAG5273_DRV_DATA_THRESHOLD_AXIS_X = 0,
	TMAG5273_DRV_DATA_THRESHOLD_AXIS_Y = 1,
	TMAG5273_DRV_DATA_THRESHOLD_AXIS_Z = 2,

	TMAG5273_DRV_DATA_THRESHOLD_AXIS_SIZE,
};

struct tmag5273_config {
	struct i2c_dt_spec i2c;

	uint8_t mag_channel;
	uint8_t axis;
	bool temperature;

	uint8_t meas_range;
	uint8_t temperature_coefficient;
	uint8_t angle_magnitude_axis;
	uint8_t ch_mag_gain_correction;

	uint8_t operation_mode;
	uint8_t averaging;

	bool trigger_conv_via_int;
	bool low_noise_mode;
	bool ignore_diag_fail;

	struct gpio_dt_spec int_gpio;

	struct gpio_dt_spec supply_gpio; /** VCC for delayed startup */
	size_t startup_delay_us;         /** power up time after the VCC pin is set to high */
	bool i2c_update;             	 /** if true, i2c address needs to be updated after startup */
	uint8_t i2c_startup_address; 	 /** startup address of one device */

#ifdef CONFIG_CRC
	bool crc_enabled;
#endif

#ifdef CONFIG_PM_DEVICE
	bool pm_int_suspend_to_wakeup_sleep;
	bool pm_i2c_workaround;
#endif

#ifdef CONFIG_TMAG5273_TRIGGER
	uint8_t int_mode;
	bool int_latched;
#endif
};

struct tmag5273_data {
	uint8_t version;             /** version as given by the sensor */
	uint16_t conversion_time_us; /** time for one conversion */

	int16_t x_sample;           /** measured B-field @x-axis */
	int16_t y_sample;           /** measured B-field @y-axis */
	int16_t z_sample;           /** measured B-field @z-axis */
	int16_t temperature_sample; /** measured temperature data */

	uint16_t xyz_range; /** magnetic range for x/y/z-axis in mT */

	int16_t angle_sample;     /** measured angle in degree, if activated */
	uint8_t magnitude_sample; /** Positive vector magnitude (can be >7 bit). */

#ifdef CONFIG_TMAG5273_TRIGGER
	const struct device *dev;          /** points to own handle */
	struct gpio_callback int_callback; /** generic driver callback, forwards to user handle */

	int32_t int_threshold[TMAG5273_DRV_DATA_THRESHOLD_AXIS_SIZE];

	const struct sensor_trigger *int_trigger;
	sensor_trigger_handler_t int_handler;

	atomic_t on_interrupt;

#ifdef CONFIG_TMAG5273_TRIGGER_OWN_THREAD
	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_TMAG5273_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem gpio_sem;
#elif CONFIG_TMAG5273_TRIGGER_GLOBAL_THREAD
	struct k_work work;
#endif

#endif
};

int tmag5273_clear_latching_interrupt(const struct device *dev);

#ifdef CONFIG_PM_DEVICE
int tmag5273_pm_is_modifiable(const struct device *dev);
int tmag5273_pm_set_operation_mode(const struct device *dev, enum pm_device_action action);
#endif

#ifdef CONFIG_TMAG5273_TRIGGER
int tmag5273_trigger_init(const struct device *dev);

int tmag5273_trigger_attr_set(const struct device *dev, enum sensor_channel chan,
			      enum sensor_attribute attr, const struct sensor_value *val);
int tmag5273_trigger_attr_get(const struct device *dev, enum sensor_channel chan,
			      enum sensor_attribute attr, struct sensor_value *val);

int tmag5273_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
			 sensor_trigger_handler_t handler);

bool tmag5273_on_interrupt_handling(const struct device *dev);
#endif

#endif /* ZEPHYR_DRIVERS_SENSOR_TMAG5273_H_ */
