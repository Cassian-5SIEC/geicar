/*
 * iks01a2.c
 *
 *  Created on: Sep 23, 2021
 *      Author: dimercur
 */

#include "stm32l4xx_hal.h"

#include "lsm303agr_reg.h"
#include "lsm6dsl_reg.h"
#include "lps22hb_reg.h"
#include "hts221_reg.h"

#include "iks01a2.h"

#include "globalvar.h"

#include <string.h>

#define BOOT_TIME 15 //ms
#define SENSOR_BUS hi2c1

stmdev_ctx_t lsm6dl_handler;
stmdev_ctx_t lsm303agr_handler;
stmdev_ctx_t lps22hb_handler;
stmdev_ctx_t hts221_handler;

static int32_t platform_write_lsm6dsl(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read_lsm6dsl(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static int32_t platform_write_lsm303agr(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read_lsm303agr(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static int32_t platform_write_lps22hb(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read_lps22hb(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static int32_t platform_write_hts221(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read_hts221(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

typedef struct {
	void   *hbus;
	uint8_t i2c_address;
	GPIO_TypeDef *cs_port;
	uint16_t cs_pin;
} sensbus_t;

static sensbus_t mag_bus = {
		&SENSOR_BUS,
		LSM303AGR_I2C_ADD_MG,
		0,
		0
};

static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static int16_t data_raw_magnetic[3];
static int16_t data_raw_humidity;
static int16_t data_raw_temperature;
static uint32_t data_raw_pressure;

static uint8_t whoamI, rst;

/*
 *  Function used to apply coefficient
 */
typedef struct {
	float x0;
	float y0;
	float x1;
	float y1;
} lin_t;

static lin_t lin_hum;
static lin_t lin_temp;

/*
 * @brief  Compute interpolation between 2 data
 *
 * @param  lin pointer to a lin_t struct
 * @param  x raw value to interpolate
 * @return  interpolated value
 *
 */
float linear_interpolation(lin_t *lin, int16_t x)
{
	return ((lin->y1 - lin->y0) * x + ((lin->x1 * lin->y0) -
			(lin->x0 * lin->y1)))
			/ (lin->x1 - lin->x0);
}

/*
 * @brief  Initialize sensors
 *
 */
uint32_t IKS01A2_Init(void)
{
	/* Wait sensor boot time */
	HAL_Delay(BOOT_TIME);

	/*********************************************************
	 * lsm6dl Initialisation
	 *
	 * Output Data Rate (acc)  : 104 Hz
	 * Output Data Rate (gyro) : 104 Hz
	 * Full scale (acc)        : 2g
	 * Full scale (gyro)       : 500dps
	 *********************************************************/
	lsm6dl_handler.write_reg = platform_write_lsm6dsl;
	lsm6dl_handler.read_reg = platform_read_lsm6dsl;
	lsm6dl_handler.handle = &SENSOR_BUS;

	/* Check device ID */
	whoamI = 0;
	lsm6dsl_device_id_get(&lsm6dl_handler, &whoamI);

	if ( whoamI != LSM6DSL_ID )
		return 0; /*manage here device not found */

	/* Restore default configuration */
	lsm6dsl_reset_set(&lsm6dl_handler, PROPERTY_ENABLE);

	do {
		lsm6dsl_reset_get(&lsm6dl_handler, &rst);
	} while (rst);

	/* Enable Block Data Update */
	lsm6dsl_block_data_update_set(&lsm6dl_handler, PROPERTY_ENABLE);
	/* Set Output Data Rate */
	lsm6dsl_xl_data_rate_set(&lsm6dl_handler, LSM6DSL_XL_ODR_52Hz);
	lsm6dsl_gy_data_rate_set(&lsm6dl_handler, LSM6DSL_GY_ODR_52Hz);
	/* Set Power Mode */
	lsm6dsl_xl_power_mode_set(&lsm6dl_handler, LSM6DSL_XL_HIGH_PERFORMANCE);
	lsm6dsl_gy_power_mode_set(&lsm6dl_handler, LSM6DSL_GY_HIGH_PERFORMANCE);
	/* Set full scale */
	lsm6dsl_xl_full_scale_set(&lsm6dl_handler, LSM6DSL_2g);
	lsm6dsl_gy_full_scale_set(&lsm6dl_handler, LSM6DSL_500dps);
	/* Configure filtering chain(No aux interface) */
	/* Accelerometer - analog filter */
//	lsm6dsl_xl_filter_analog_set(&lsm6dl_handler, LSM6DSL_XL_ANA_BW_400Hz);
	/* Accelerometer - LPF1 path ( LPF2 not used )*/
	//lsm6dsl_xl_lp1_bandwidth_set(&lsm6dl_handler, LSM6DSL_XL_LP1_ODR_DIV_4);
	/* Accelerometer - LPF1 + LPF2 path */
//	lsm6dsl_xl_lp2_bandwidth_set(&lsm6dl_handler,LSM6DSL_XL_LOW_NOISE_LP_ODR_DIV_100);
	/* Accelerometer - High Pass / Slope path */
	//lsm6dsl_xl_reference_mode_set(&lsm6dl_handler, PROPERTY_DISABLE);
	//lsm6dsl_xl_hp_bandwidth_set(&lsm6dl_handler, LSM6DSL_XL_HP_ODR_DIV_100);
	/* Gyroscope - filtering chain */
	lsm6dsl_gy_band_pass_set(&lsm6dl_handler, LSM6DSL_HP_260mHz_LP1_STRONG);

	/*********************************************************
	 * lsm303agr initialisation
	 *
	 * Output Data Rate (mag)  : 10 Hz
	 *********************************************************/
	lsm303agr_handler.write_reg = platform_write_lsm303agr;
	lsm303agr_handler.read_reg = platform_read_lsm303agr;
	lsm303agr_handler.handle = (void *)&mag_bus;

	/* Check device ID */
	whoamI = 0;
	lsm303agr_mag_device_id_get(&lsm303agr_handler, &whoamI);

	if ( whoamI != LSM303AGR_ID_MG )
		while (1); /*manage here device not found */

	/* Restore default configuration for magnetometer */
	lsm303agr_mag_reset_set(&lsm303agr_handler, PROPERTY_ENABLE);

	do {
		lsm303agr_mag_reset_get(&lsm303agr_handler, &rst);
	} while (rst);

	/* Enable Block Data Update */
	lsm303agr_mag_block_data_update_set(&lsm303agr_handler, PROPERTY_ENABLE);
	/* Set Output Data Rate */
	lsm303agr_mag_data_rate_set(&lsm303agr_handler, LSM303AGR_MG_ODR_10Hz);
	/* Set / Reset magnetic sensor mode */
	lsm303agr_mag_set_rst_mode_set(&lsm303agr_handler,LSM303AGR_SENS_OFF_CANC_EVERY_ODR);
	/* Enable temperature compensation on mag sensor */
	lsm303agr_mag_offset_temp_comp_set(&lsm303agr_handler, PROPERTY_ENABLE);
	/* Enable temperature sensor */
	//lsm303agr_temperature_meas_set(&lsm303agr_handler, LSM303AGR_TEMP_ENABLE);
	/* Set magnetometer in continuous mode */
	lsm303agr_mag_operating_mode_set(&lsm303agr_handler,LSM303AGR_CONTINUOUS_MODE);

	/*********************************************************
	 * hts221 initialisation
	 *
	 * Output Data Rate (mag)  : 1 Hz
	 *********************************************************/
	hts221_handler.write_reg = platform_write_hts221;
	hts221_handler.read_reg = platform_read_hts221;
	hts221_handler.handle = &SENSOR_BUS;

	/* Check device ID */
	whoamI = 0;
	hts221_device_id_get(&hts221_handler, &whoamI);

	if ( whoamI != HTS221_ID )
		while (1); /*manage here device not found */

	/* Read humidity calibration coefficient */
	hts221_hum_adc_point_0_get(&hts221_handler, &lin_hum.x0);
	hts221_hum_rh_point_0_get(&hts221_handler, &lin_hum.y0);
	hts221_hum_adc_point_1_get(&hts221_handler, &lin_hum.x1);
	hts221_hum_rh_point_1_get(&hts221_handler, &lin_hum.y1);

	/* Read temperature calibration coefficient */
	hts221_temp_adc_point_0_get(&hts221_handler, &lin_temp.x0);
	hts221_temp_deg_point_0_get(&hts221_handler, &lin_temp.y0);
	hts221_temp_adc_point_1_get(&hts221_handler, &lin_temp.x1);
	hts221_temp_deg_point_1_get(&hts221_handler, &lin_temp.y1);
	/* Enable Block Data Update */
	hts221_block_data_update_set(&hts221_handler, PROPERTY_ENABLE);
	/* Set Output Data Rate */
	hts221_data_rate_set(&hts221_handler, HTS221_ODR_1Hz);
	/* Device power on */
	hts221_power_on_set(&hts221_handler, PROPERTY_ENABLE);

	/*********************************************************
	 * lps22hb initialisation
	 *
	 * Output Data Rate  : 1 Hz
	 *********************************************************/
	lps22hb_handler.write_reg = platform_write_lps22hb;
	lps22hb_handler.read_reg = platform_read_lps22hb;
	lps22hb_handler.handle = &SENSOR_BUS;

	/* Check device ID */
	lps22hb_device_id_get(&lps22hb_handler, &whoamI);

	if (whoamI != LPS22HB_ID) {
		while (1)/* manage here device not found */;
	}

	/* Restore default configuration */
	lps22hb_reset_set(&lps22hb_handler, PROPERTY_ENABLE);

	do {
		lps22hb_reset_get(&lps22hb_handler, &rst);
	} while (rst);

	/* Enable Block Data Update */
	//lps22hb_block_data_update_set(&lps22hb_handler, PROPERTY_ENABLE);
	/* Can be enabled low pass filter on output */
	lps22hb_low_pass_filter_mode_set(&lps22hb_handler, LPS22HB_LPF_ODR_DIV_2);
	/* Can be set Data-ready signal on INT_DRDY pin */
	//lps22hb_drdy_on_int_set(&lps22hb_handler, PROPERTY_ENABLE);
	/* Set Output Data Rate */
	lps22hb_data_rate_set(&lps22hb_handler, LPS22HB_ODR_1_Hz);

	return 1;
}

/*
 * @brief  Get acceleration data (in milli G (mg))
 *
 * @param  acceleration_mg    pointer to a values3d_t struct (x, y and z axis)
 * @return  1 if data are valid, 0 otherwise
 *
 */
uint32_t IKS01A2_GetAcceleration(values3d_t *acceleration_mg) {
	/* Read output only if new value is available */
	lsm6dsl_reg_t reg;
	lsm6dsl_status_reg_get(&lsm6dl_handler, &reg.status_reg);

	if (reg.status_reg.xlda) {
		/* Read acceleration data */
		memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
		lsm6dsl_acceleration_raw_get(&lsm6dl_handler, data_raw_acceleration);
		acceleration_mg->x = lsm6dsl_from_fs2g_to_mg(data_raw_acceleration[0]);
		acceleration_mg->y = lsm6dsl_from_fs2g_to_mg(data_raw_acceleration[1]);
		acceleration_mg->z = lsm6dsl_from_fs2g_to_mg(data_raw_acceleration[2]);

		return 1;
	}

	return 0;
}

/*
 * @brief  Get rotation data (in milli degree per second (mdps))
 *
 * @param  angular_rate_mdps    pointer to a values3d_t struct (x, y and z axis)
 * @return  1 if data are valid, 0 otherwise
 *
 */
uint32_t IKS01A2_GetRotation(values3d_t *angular_rate_mdps) {
	/* Read output only if new value is available */
	lsm6dsl_reg_t reg;
	lsm6dsl_status_reg_get(&lsm6dl_handler, &reg.status_reg);

	if (reg.status_reg.gda) {
		/* Read magnetic field data */
		memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
		lsm6dsl_angular_rate_raw_get(&lsm6dl_handler, data_raw_angular_rate);
		angular_rate_mdps->x = lsm6dsl_from_fs500dps_to_mdps(data_raw_angular_rate[0]);
		angular_rate_mdps->y = lsm6dsl_from_fs500dps_to_mdps(data_raw_angular_rate[1]);
		angular_rate_mdps->z = lsm6dsl_from_fs500dps_to_mdps(data_raw_angular_rate[2]);

		return 1;
	}

	return 0;
}

/*
 * @brief  Get temperature (from lsm6dl). Less accurate than HTS221
 *
 * @param  temperature_degC    value in celsius degree
 * @return  1 if data are valid, 0 otherwise
 *
 */
//uint32_t IKS01A2_GetTemperature(float *temperature_degC) {
//	/* Read output only if new value is available */
//	lsm6dsl_reg_t reg;
//	lsm6dsl_status_reg_get(&lsm6dl_handler, &reg.status_reg);
//
//	if (reg.status_reg.tda) {
//		/* Read temperature data */
//		memset(&data_raw_temperature, 0x00, sizeof(int16_t));
//		lsm6dsl_temperature_raw_get(&lsm6dl_handler, &data_raw_temperature);
//		*temperature_degC = lsm6dsl_from_lsb_to_celsius(
//				data_raw_temperature );
//
//      return 1;
//	}
//
//	return 0;
//}

/*
 * @brief  Get magnetic field data (in milli gauss)
 *
 * @param  magnetic_mG    pointer to a values3d_t struct (x, y and z axis)
 * @return  1 if data are valid, 0 otherwise
 *
 */
uint32_t IKS01A2_GetMagnetic(values3d_t *magnetic_mG) {
	/* Read output only if new value is available */
	lsm303agr_reg_t reg;
	lsm303agr_mag_status_get(&lsm303agr_handler, &reg.status_reg_m);

	if (reg.status_reg_m.zyxda) {
		/* Read magnetic field data */
		memset(data_raw_magnetic, 0x00, 3 * sizeof(int16_t));
		lsm303agr_magnetic_raw_get(&lsm303agr_handler, data_raw_magnetic);
		magnetic_mG->x = lsm303agr_from_lsb_to_mgauss(data_raw_magnetic[0]);
		magnetic_mG->y = lsm303agr_from_lsb_to_mgauss(data_raw_magnetic[1]);
		magnetic_mG->z = lsm303agr_from_lsb_to_mgauss(data_raw_magnetic[2]);

		return 1;
	}

	return 0;
}

/*
 * @brief  Get ambient humidity (in %)
 *
 * @param  humidity_perc    ambient humidity
 * @return  1 if data are valid, 0 otherwise
 *
 */
uint32_t IKS01A2_GetHumidity(float *humidity_perc) {
	/* Read output only if new value is available */
	hts221_reg_t reg;
	hts221_status_get(&hts221_handler, &reg.status_reg);

	if (reg.status_reg.h_da) {
		/* Read humidity data */
		memset(&data_raw_humidity, 0x00, sizeof(int16_t));
		hts221_humidity_raw_get(&hts221_handler, &data_raw_humidity);
		*humidity_perc = linear_interpolation(&lin_hum, data_raw_humidity);

		if (*humidity_perc < 0) {
			*humidity_perc = 0;
		}

		if (*humidity_perc > 100) {
			*humidity_perc = 100;
		}

		return 1;
	}

	return 0;
}

/*
 * @brief  Get temperature (in celsius degree)
 *
 * @param  temperature_degC    ambient temperature
 * @return  1 if data are valid, 0 otherwise
 *
 */
uint32_t IKS01A2_GetTemperature(float *temperature_degC) {
	/* Read output only if new value is available */
	hts221_reg_t reg;
	hts221_status_get(&hts221_handler, &reg.status_reg);

	if (reg.status_reg.t_da) {
		/* Read temperature data */
		memset(&hts221_handler, 0x00, sizeof(int16_t));
		hts221_temperature_raw_get(&hts221_handler, &data_raw_temperature);
		*temperature_degC = linear_interpolation(&lin_temp, data_raw_temperature);

		return 1;
	}

	return 0;
}

/*
 * @brief  Get ambient pressure (in hecto Pascal)
 *
 * @param  pressure_hPa    ambient pressure
 * @return  1 if data are valid, 0 otherwise
 *
 */
uint32_t IKS01A2_GetPressure(float *pressure_hPa) {
	/* Read output only if new value is available */
	uint8_t reg;
	lps22hb_press_data_ready_get(&lps22hb_handler, &reg);

	if (reg) {
		memset(&data_raw_pressure, 0x00, sizeof(int32_t));
		lps22hb_pressure_raw_get(&lps22hb_handler, &data_raw_pressure);
		*pressure_hPa = lps22hb_from_lsb_to_hpa(data_raw_pressure);

		return 1;
	}

	return 0;
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write_lsm6dsl(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
	HAL_I2C_Mem_Write(handle, LSM6DSL_I2C_ADD_H, reg,
			I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);

	return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read_lsm6dsl(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	HAL_I2C_Mem_Read(handle, LSM6DSL_I2C_ADD_H, reg,
			I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);

	return 0;
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write_lsm303agr(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
	sensbus_t *sensbus = (sensbus_t *)handle;

	if (sensbus->i2c_address == LSM303AGR_I2C_ADD_XL) {
		/* enable auto incremented in multiple read/write commands */
		reg |= 0x80;
	}

	HAL_I2C_Mem_Write(sensbus->hbus, sensbus->i2c_address, reg,
			I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);

	return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read_lsm303agr(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	sensbus_t *sensbus = (sensbus_t *)handle;

	if (sensbus->i2c_address == LSM303AGR_I2C_ADD_XL) {
		/* enable auto incremented in multiple read/write commands */
		reg |= 0x80;
	}

	HAL_I2C_Mem_Read(sensbus->hbus, sensbus->i2c_address, reg,
			I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);

	return 0;
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write_lps22hb(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
	HAL_I2C_Mem_Write(handle, LPS22HB_I2C_ADD_H, reg,
			I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);

	return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read_lps22hb(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	HAL_I2C_Mem_Read(handle, LPS22HB_I2C_ADD_H, reg,
			I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);

	return 0;
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write_hts221(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
	reg |= 0x80;
	HAL_I2C_Mem_Write(handle, HTS221_I2C_ADDRESS, reg,
			I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);

	return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read_hts221(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	reg |= 0x80;
	HAL_I2C_Mem_Read(handle, HTS221_I2C_ADDRESS, reg,
			I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);

	return 0;
}
