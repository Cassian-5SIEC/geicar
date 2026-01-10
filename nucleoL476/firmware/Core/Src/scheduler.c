/*
 * scheduler.c
 *
 *  Created on: Sep 24, 2021
 *      Author: dimercur
 */

#include <gnss_nmea_parser.h>
#include "stm32l4xx_hal.h"

#include "globalvar.h"
#include "iks01a2.h"
#include "can.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
uint8_t flag_1ms;

/***************************************
 * GPS Data send and variables
 */
extern char nmeaFrame[200];
extern struct UbxFrame ubxFrame;
extern int nmeaFrameValid;
extern int ubxFrameValid;
int gpsSend = 0;

uint8_t gps_data[8] = {0,0,0,0,0,0,0,0};

/***************************************
 * IMU Data send
 */
extern int imuInit;
uint8_t IMU1[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t IMU2[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t IMU3[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t IMU4[8] = {0, 0, 0, 0, 0, 0, 0, 0};
/*********************************
 * Task task_send_values_GPS
 * Periodic: 5hz (200 ms)
 */
void task_send_values_GPS (void);
uint32_t counter_task_send_values_GPS;
#define PERIOD_TASK_SEND_VALUES_GPS 200

/*********************************
 * Task task_send_values_IMU
 * Periodic: 50hz (20 ms)
 */
void task_send_values_IMU (void);
uint32_t counter_task_send_values_IMU;
#define PERIOD_TASK_SEND_VALUES_IMU 20

/*********************************
 * Task task_get_acceleration
 * Periodic: 50hz (20 ms)
 */
void task_get_acceleration (void);
uint32_t counter_task_get_acceleration;
#define PERIOD_TASK_GET_ACCELERATION 20

/*********************************
 * Task task_get_rotation
 * Periodic: 50hz (20 ms)
 */
void task_get_rotation (void);
uint32_t counter_task_get_rotation;
#define PERIOD_TASK_GET_ROTATION 20

/*********************************
 * Task task_get_magnetic
 * Periodic: 10hz (100 ms)
 */
void task_get_magnetic (void);
uint32_t counter_task_get_magnetic;
#define PERIOD_TASK_GET_MAGNETIC 100

/*********************************
 * Task task_get_temperature
 * Periodic: 1hz (1000 ms)
 */
void task_get_temperature (void);
uint32_t counter_task_get_temperature;
#define PERIOD_TASK_GET_TEMPERATURE 1000

/*********************************
 * Task task_get_pressure
 * Periodic: 1hz (1000 ms)
 */
void task_get_pressure (void);
uint32_t counter_task_get_pressure;
#define PERIOD_TASK_GET_PRESSURE 1000

/*********************************
 * Task task_get_humidity
 * Periodic: 1hz (1000 ms)
 */
void task_get_humidity (void);
uint32_t counter_task_get_humidity;
#define PERIOD_TASK_GET_HUMIDITY 1000

/*********************************
 * Task task_update_gps
 * Periodic: 50hz (20 ms)
 */
void task_update_gps (void);
uint32_t counter_task_update_gps;
#define PERIOD_TASK_UPDATE_GPS 20

/*********************************/

void task_background(void);

int sign(float n){
	if (n>=0){
		return 0;
	}else{
		return 1;
	}
}
/*
 * @brief  Initialize scheduler counters and flags
 *
 */
uint32_t SCHEDULER_Init(void) {
	flag_1ms =0;

	counter_task_send_values_IMU=0;
	counter_task_send_values_GPS=0;
	/* Counter for IMU tasks */
	counter_task_get_acceleration=0;
	counter_task_get_rotation=0;
	counter_task_get_magnetic=0;
	counter_task_get_pressure=0;
	counter_task_get_humidity=0;
	counter_task_get_temperature=0;

	/* Counter for GPS tasks */
	counter_task_update_gps=0;

	return 1;
}

/*
 * @brief  execute scheduler
 *
 */
void SCHEDULER_Run(void) {

	while (1)
	{
		if (flag_1ms!=0)
		{
			flag_1ms=0;

			/* increase task counters */
			if (imuInit){
				counter_task_send_values_IMU++;
				counter_task_get_acceleration++;
				counter_task_get_rotation++;
				// counter_task_get_magnetic++;
				// counter_task_get_temperature++;
				// counter_task_get_humidity++;
				// counter_task_get_pressure++;
			}
			/* !!!!!!!!!!!!!!! Deactivate GPS tasks  !!!!!!!!!!!!!!!!*/
			// counter_task_send_values_GPS++;
			// counter_task_update_gps++;
		}

		if (counter_task_get_acceleration>= PERIOD_TASK_GET_ACCELERATION) {
			counter_task_get_acceleration=0;
			task_get_acceleration();
		}

		if (counter_task_get_rotation>= PERIOD_TASK_GET_ROTATION) {
			counter_task_get_rotation=0;
			task_get_rotation();
		}

		if (counter_task_get_magnetic>= PERIOD_TASK_GET_MAGNETIC) {
			counter_task_get_magnetic=0;
			task_get_magnetic();
		}

		if (counter_task_get_temperature>= PERIOD_TASK_GET_TEMPERATURE) {
			counter_task_get_temperature=0;
			task_get_temperature();
		}

		if (counter_task_get_humidity>= PERIOD_TASK_GET_HUMIDITY) {
			counter_task_get_humidity=0;
			task_get_humidity();
		}

		if (counter_task_get_pressure>= PERIOD_TASK_GET_PRESSURE) {
			counter_task_get_pressure=0;
			task_get_pressure();
		}


		if (counter_task_update_gps>= PERIOD_TASK_UPDATE_GPS) {
			counter_task_update_gps=0;
			task_update_gps();
		}


		if (counter_task_send_values_IMU>= PERIOD_TASK_SEND_VALUES_IMU) {
			counter_task_send_values_IMU=0;
			task_send_values_IMU();
		}

		if (counter_task_send_values_GPS>= PERIOD_TASK_SEND_VALUES_GPS) {
			counter_task_send_values_GPS=0;
			task_send_values_GPS();
		}

		task_background();
	}
}

void task_background(void) {

}

void task_send_values_IMU (void) {

	//IMU1 : Magnetic field
	// IMU1[6] = 4*sign(current_magnetic_mG.x) + 2*sign(current_magnetic_mG.y) + 1*sign(current_magnetic_mG.z);

	// int mag_x = abs(current_magnetic_mG.x);
	// int mag_y = abs(current_magnetic_mG.y);
	// int mag_z = abs(current_magnetic_mG.z);

	// IMU1[0]= (mag_x & 0xff00)>>8;
	// IMU1[1]= (mag_x & 0xff);
	// IMU1[2]= (mag_y & 0xff00)>>8;
	// IMU1[3]= (mag_y & 0xff);
	// IMU1[4]= (mag_z & 0xff00)>>8;
	// IMU1[5]= (mag_z & 0xff);

	//IMU2 : Angular velocity
	IMU2[6] = 4*sign(current_angular_rate_mdps.x) + 2*sign(current_angular_rate_mdps.y) + 1*sign(current_angular_rate_mdps.z);

	int ang_x = abs(current_angular_rate_mdps.x);
	int ang_y = abs(current_angular_rate_mdps.y);
	int ang_z = abs(current_angular_rate_mdps.z);

	IMU2[0]= (ang_x & 0xff00)>>8;
	IMU2[1]= (ang_x & 0xff);
	IMU2[2]= (ang_y & 0xff00)>>8;
	IMU2[3]= (ang_y & 0xff);
	IMU2[4]= (ang_z & 0xff00)>>8;
	IMU2[5]= (ang_z & 0xff);


	//IMU3 : Linear acceleration
	IMU3[6] = 4*sign(current_acceleration_mg.x) + 2*sign(current_acceleration_mg.y) + 1*sign(current_acceleration_mg.z);

	int acc_x = abs(current_acceleration_mg.x);
	int acc_y = abs(current_acceleration_mg.y);
	int acc_z = abs(current_acceleration_mg.z);

	IMU3[0]= (acc_x & 0xff00)>>8;
	IMU3[1]= (acc_x & 0xff);
	IMU3[2]= (acc_y & 0xff00)>>8;
	IMU3[3]= (acc_y & 0xff);
	IMU3[4]= (acc_z & 0xff00)>>8;
	IMU3[5]= (acc_z & 0xff);

	//IMU4 : General (temperature, pressure, humidity)
	// int temp = (int)current_temperature_degC*10;
	// int pressure = current_pressure_hPa;
	// int humidity = current_humidity_perc;

	// IMU4[0] = (temp >> 8) & 0xff;
	// IMU4[1] = temp & 0xff;
	// IMU4[2] = (pressure >> 8) & 0xff;
	// IMU4[3] = pressure & 0xff;
	// IMU4[4] = humidity & 0xff;

	// CAN_Send(IMU1, CAN_ID_IMU1);
	CAN_Send(IMU2,CAN_ID_IMU2);
	CAN_Send(IMU3,CAN_ID_IMU3);
	// CAN_Send(IMU4,CAN_ID_IMU4);


}

void task_send_values_GPS (void) {

	if (gpsSend){
		//Latitude
		double computing = gpsCoords.lat;
		int Lat7 = floor(computing);
		computing = 100.0*(computing-(double)Lat7);
		int Lat6 = floor(computing);
		computing = 100.0*(computing-(double)Lat6);
		int Lat5 = floor(computing);
		computing = 100.0*(computing-(double)Lat5);
		int Lat4 = floor(computing);
		computing = 100.0*(computing-(double)Lat4);
		int Lat3 = floor(computing);
		computing = 100.0*(computing-(double)Lat3);
		int Lat2 = floor(computing);
		computing = 100.0*(computing-(double)Lat2);
		int Lat1 = floor(computing);
		computing = 100.0*(computing-(double)Lat1);
		int Lat0 = floor(computing);

		gps_data[0] = Lat7;
		gps_data[1] = Lat6;
		gps_data[2] = Lat5;
		gps_data[3] = Lat4;
		gps_data[4] = Lat3;
		gps_data[5] = Lat2;
		gps_data[6] = Lat1;
		gps_data[7] = Lat0;
		CAN_Send(gps_data, CAN_ID_GPS1);

		//Longitude
		computing = gpsCoords.lon;
		int Lon7 = floor(computing);
		computing = 100.0*(computing-(double)Lon7);
		int Lon6 = floor(computing);
		computing = 100.0*(computing-(double)Lon6);
		int Lon5 = floor(computing);
		computing = 100.0*(computing-(double)Lon5);
		int Lon4 = floor(computing);
		computing = 100.0*(computing-(double)Lon4);
		int Lon3 = floor(computing);
		computing = 100.0*(computing-(double)Lon3);
		int Lon2 = floor(computing);
		computing = 100.0*(computing-(double)Lon2);
		int Lon1 = floor(computing);
		computing = 100.0*(computing-(double)Lon1);
		int Lon0 = floor(computing);

		gps_data[0] = Lon7;
		gps_data[1] = Lon6;
		gps_data[2] = Lon5;
		gps_data[3] = Lon4;
		gps_data[4] = Lon3;
		gps_data[5] = Lon2;
		gps_data[6] = Lon1;
		gps_data[7] = Lon0;
		CAN_Send(gps_data, CAN_ID_GPS2);

		//Altitude
		computing = gpsCoords.alt;
		int Alt7 = floor(computing);
		computing = 100.0*(computing-(double)Alt7);
		int Alt6 = floor(computing);
		computing = 100.0*(computing-(double)Alt6);
		int Alt5 = floor(computing);
		computing = 100.0*(computing-(double)Alt5);
		int Alt4 = floor(computing);
		computing = 100.0*(computing-(double)Alt4);
		int Alt3 = floor(computing);
		computing = 100.0*(computing-(double)Alt3);
		int Alt2 = floor(computing);
		computing = 100.0*(computing-(double)Alt2);
		int Alt1 = floor(computing);
		computing = 100.0*(computing-(double)Alt1);
		int Alt0 = floor(computing);

		gps_data[0] = Alt7;
		gps_data[1] = Alt6;
		gps_data[2] = Alt5;
		gps_data[3] = Alt4;
		gps_data[4] = Alt3;
		gps_data[5] = Alt2;
		gps_data[6] = Alt1;
		gps_data[7] = Alt0;
		CAN_Send(gps_data, CAN_ID_GPS3);


		//Status
		gps_data[0] = gpsQuality;
		gps_data[1] = (ubx_nav_pvt_msg.hAcc & 0xff0000) >> 16;
		gps_data[2] = (ubx_nav_pvt_msg.hAcc & 0xff00) >> 8;
		gps_data[3] = ubx_nav_pvt_msg.hAcc & 0xff;
		gps_data[4] = (ubx_nav_pvt_msg.vAcc & 0xff0000) >> 16;
		gps_data[5] = (ubx_nav_pvt_msg.vAcc & 0xff00) >> 8;
		gps_data[6] = ubx_nav_pvt_msg.vAcc & 0xff;
		CAN_Send(gps_data, CAN_ID_GPS4);

		gpsSend = 0;
	}


}

void task_get_acceleration (void) {
	IKS01A2_GetAcceleration(&current_acceleration_mg);
}

void task_get_rotation (void) {
	IKS01A2_GetRotation(&current_angular_rate_mdps);
}

void task_get_magnetic (void) {
	IKS01A2_GetMagnetic(&current_magnetic_mG);
}

void task_get_temperature (void) {
	IKS01A2_GetTemperature(&current_temperature_degC);
}

void task_get_humidity (void) {
	IKS01A2_GetHumidity(&current_humidity_perc);
}

void task_get_pressure (void) {
	IKS01A2_GetPressure(&current_pressure_hPa);
}

void task_update_gps(void) {
	if (nmeaFrameValid){
		ProcessNMEALine(nmeaFrame,&gpsCoords, &gpsQuality);
		gpsSend = 1;
		nmeaFrameValid = 0;
	}

	if (ubxFrameValid){
		UBX_Parse_Raw_To_NAV_PVT(ubxFrame.payload,ubxFrame.length, &ubx_nav_pvt_msg);
		gpsSend = 1;
		ubxFrameValid = 0;
	}

}
