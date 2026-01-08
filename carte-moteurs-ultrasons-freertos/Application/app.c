/**
 * @file app.c
 * @author Sebastien DI MERCURIO
 * @version V1.0
 * @date 20 Aout 2023
 *
 * @brief Main application file.
 * This file contains the main application logic, including initialization and the main loop.
 * It handles the control of the car's motors, ultrasonic sensors, and communication via CAN.
 */

#include "app.h"

#include "configuration.h"

#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "power.h"
#include "FLASH_PAGE_F1.h"
#include "steering.h"
#include "wheels.h"
#include "ultrasound.h"
#include "control.h"
#include "calibrate.h"
#include "can_communication.h"
#include "measures.h"
#include "i2c_cassian.h"

#include "tasks.h"

#include <stdio.h>

#if defined (__TESTS__)
#include "tests.h"
#endif

///* Modes
// * 0- Calibration
// * 1- Control
// */
//int mode = 1;
//
//int UPDATE_CMD_FLAG = 1;
//int US_FLAG = 1;
//int CAN_SEND_MOTORS = 1;
//int CAN_SEND_US = 0;
//int CAN_SEND_BATT = 1;


/* Data buffer for CAN messages */
uint8_t data[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};

// Speed cmd
//int leftRearSpeed = -1;
//int rightRearSpeed = -1;
//int steeringAngle = -1;

// Periodic counters
//static int cmpt_can_motors = 0;
//static int cmpt_us = 0;
//static int cmpt_batt = 0;

//Communication checking request
//int commCheckingRequest = 0;

/**
 * @brief Initialize the application.
 * This function sets up the necessary software components
 */
void APP_Init(void) {
	// First of all, maintain power
	POWER_Boostrap();

	// Then, initialize all the peripherals subsystems
	// Moteurs
	WHEELS_Init();

	// Steering
	STEERING_Init();

	// Analog measures (ADC1)
	MEASURES_Init();

	// Ultrasonic sensors
	US_Init();

	// CAN communications
	CAN_COM_Init();

	// Control
	CAR_CONTROL_Init();

	// I2C
	I2C_init() ;

	// Tasks
	TASKS_Init();




	/* Steering Initialization*/
	// Write default calibration values in flash memory (first use only)
	if ((int)Flash_Read_NUM(STEERING_CALIBRATION_A_DEFAULT_ADDR)!=(int)STEERING_CALIBRATION_A_DEFAULT
			|| (int)Flash_Read_NUM(STEERING_CALIBRATION_B_DEFAULT_ADDR)!=(int)STEERING_CALIBRATION_B_DEFAULT)
	{
		Flash_Write_NUM(STEERING_CALIBRATION_A_DEFAULT_ADDR, STEERING_CALIBRATION_A_DEFAULT);
		Flash_Write_NUM(STEERING_CALIBRATION_B_DEFAULT_ADDR, STEERING_CALIBRATION_B_DEFAULT);

		Flash_Write_NUM(STEERING_CALIBRATION_A_ADDR, STEERING_CALIBRATION_A_DEFAULT);
		Flash_Write_NUM(STEERING_CALIBRATION_B_ADDR, STEERING_CALIBRATION_B_DEFAULT);
	}

//	printf("Motor - steering - ultrasonic sensors.\r\n");
	//printf("Application version: %s\r\n\n", APP_VERSION);
	//printf("Application started\r\n");
}

/**
 * @brief Run the application.
 *
 * This function is the main loop of the application. It handles the main logic,
 * processes inputs, and updates outputs.
 *
 * @remark: this function never returns, it runs indefinitely.
 */
void APP_Run(AppMessage_typeDef *msg) {
#if defined (__TESTS__)
	TESTS_Run(); // Run tests if defined
#else
	switch (msg->id) {
	case CAN_RECEIVED_FRAME_ID: // a CAN frame was received
		CANReceivedFrame_typeDef *canFrame = (CANReceivedFrame_typeDef*) msg;

		switch (canFrame->can_id) {
		case CAN_ID_MOTORS_CMD:
			if (canFrame->length >= 3) {
				CAR_CONTROL_Update((int) canFrame->data[0], // Get left rear motor speed
						(int) canFrame->data[1], // Get right rear motor speed
						(int8_t) canFrame->data[2]);// Get steering motor speed
			}
			break;
		case CAN_ID_CALIBRATION_MODE:
			if (canFrame->length >= 1 &&
					canFrame->data[0] == CALIBRATION_REQUEST) {
				//mode = 0;	// Enter in calibration mode
				CAL_SteeringCalibration(); // Start steering calibration
			}
			break;
		case CAN_ID_COMM_CHECKING:
			if (canFrame->length >= 1 &&
					canFrame->data[0] == COMM_CHECKING_REQUEST) {
				data[0] = COMM_CHECKING_REQUEST;
				data[1] = COMM_CHECKING_ACK;

				CAN_COM_Send(CAN_ID_COMM_CHECKING, data, 2); // Send ack
			}
			break;
		default:
			break;
		}
		break;
	case MOTORS_MEASURES_ID:
		WheelsState_typeDef *wheelsState = (WheelsState_typeDef*) msg;

		WHEELS_ResetOdometer(WHEELS_MOTOR_LEFT);
		WHEELS_ResetOdometer(WHEELS_MOTOR_RIGHT);

		//Number of sensor pulses since last message (left rear wheel and right rear wheel)
		data[0] = (uint8_t) wheelsState->nbImpulsionG;
		data[1] = (uint8_t) wheelsState->nbImpulsionD;

		data[2] = (uint8_t)((wheelsState->VMG_mes >> 8) & 0xFF); // Left Rear Speed MSB
		data[3] = (uint8_t)(wheelsState->VMG_mes & 0xFF); 	//LSB

		data[4] = (uint8_t)((wheelsState->VMD_mes >> 8) & 0xFF); // Right Rear Speed MSB
		data[5] = (uint8_t)(wheelsState->VMD_mes & 0xFF); // LSB

		//data[6] = (uint8_t)(STEERING_GetAngle());	//Steering Angle MSB

		CAN_COM_Send(CAN_ID_MOTORS_DATA, data, 6);
		break;
	case BATTERY_MEASURE_ID:
		BatteryMeasure_typeDef *battState = (BatteryMeasure_typeDef*) msg;
		//Battery Level

		//uint16_t vbat = MEASURES_GetBatteryLevel();

		data[0] = (battState->batteryLevel >> 8) & 0xFF; // Vbat MSB
		data[1] = battState->batteryLevel & 0xFF; 	//LSB

		CAN_COM_Send(CAN_ID_BATT_LEVEL, data, 2);
		break;
	case ULTRASOUND_MEASURES_ID:
		UltrasoundMesures_typeDef *usState = (UltrasoundMesures_typeDef*) msg;

		//Sending US1 data (front)
		data[0] = (usState->usDistance[0] >> 8) & 0xFF;	//US Front Left
		data[1] = usState->usDistance[0] & 0xFF;

		data[2] = (usState->usDistance[1] >> 8) & 0xFF;	//US Front Center
		data[3] = usState->usDistance[1] & 0xFF;

		data[4] = (usState->usDistance[2] >> 8) & 0xFF;	//US Front Right
		data[5] = usState->usDistance[2] & 0xFF;

		CAN_COM_Send(CAN_ID_US1, data, 6);

		//Sending US2 data (rear)
		data[0] = (usState->usDistance[3] >> 8) & 0xFF;	//US Rear Left
		data[1] = usState->usDistance[3] & 0xFF;

		data[2] = (usState->usDistance[4] >> 8) & 0xFF;	//US Rear Center
		data[3] = usState->usDistance[4] & 0xFF;

		data[4] = (usState->usDistance[5] >> 8) & 0xFF;	//US Rear Right
		data[5] = usState->usDistance[5] & 0xFF;

		CAN_COM_Send(CAN_ID_US2, data, 6);
		break;

	default:
		break;
	}

	vPortFree((void*)msg);
#endif /* __TESTS__ */

	// Perform ultrasonic sensors measurements
	//	if (msg->id == US_UPDATE_ID) {
	//		//US_FLAG=0;
	//
	//		// Manage ultrasonic sensors index and restart if needed
	//		if (currentUs >= 6)
	//			currentUs = 0;
	//
	//		usDistance[currentUs]=US_GetDistance(currentUs);
	//
	//		currentUs+=1;
	//
	//		// When all the us sensors have been performed,
	//		// send data to can
	//		if (currentUs == 6)
	//			CAN_SEND_US = 1;
	//	}
	//
	// Update motors commands
	//	if (msg->id == CAR_CONTROL_UPDATE_ID) {
	//		//UPDATE_CMD_FLAG = 0;
	//		CAR_CONTROL_Update(leftRearSpeed,rightRearSpeed, steeringAngle);
	//
	//		//		if (mode == 0) {	//Calibration Mode
	//		//			CAL_SteeringCalibration();
	//		//			mode = 1;
	//		//		} else {	//Control Mode
	//		//			CAR_CONTROL_Update(leftRearSpeed,rightRearSpeed, steeringAngle);
	//		//		}
	//	}
	//
	//	/* CAN : Sending motors data */
	//	if (msg->id == CAN_SEND_MOTORS_ID) {
	//
	//		//Number of sensor pulses since last message (left rear wheel and right rear wheel)
	//		data[0] = nbImpulsionG;
	//		data[1] = nbImpulsionD;
	//		nbImpulsionG = 0;
	//		nbImpulsionD = 0;
	//
	//		data[2] = (uint8_t)((VMG_mes >> 8) & 0xFF); // Left Rear Speed MSB
	//		data[3] = (uint8_t)(VMG_mes & 0xFF); 	//LSB
	//
	//		data[4] = (uint8_t)((VMD_mes >> 8) & 0xFF); // Right Rear Speed MSB
	//		data[5] = (uint8_t)(VMD_mes & 0xFF); // LSB
	//
	//		//data[6] = (uint8_t)(STEERING_GetAngle());	//Steering Angle MSB
	//
	//		CAN_COM_Send(CAN_ID_MOTORS_DATA, data, 6);
	//
	//		//CAN_SEND_MOTORS = 0;
	//	}
	//
	//	/* CAN : Sending battery data */
	//	if (CAN_SEND_BATT) {
	//		//Battery Level
	//
	//		uint16_t vbat = MEASURES_GetBatteryLevel();
	//
	//		data[0] = (vbat >> 8) & 0xFF; // Vbat MSB
	//		data[1] = vbat & 0xFF; 	//LSB
	//
	//		CAN_COM_Send(CAN_ID_BATT_LEVEL, data, 2);
	//
	//		CAN_SEND_BATT= 0;
	//	}
	//
	//	/* CAN : Sending ultrasonic sensors data */
	//	if (CAN_SEND_US) {
	//		//Sending US1 data (front)
	//		data[0] = (usDistance[0] >> 8) & 0xFF;	//US Front Left
	//		data[1] = usDistance[0] & 0xFF;
	//
	//		data[2] = (usDistance[1] >> 8) & 0xFF;	//US Front Center
	//		data[3] = usDistance[1] & 0xFF;
	//
	//		data[4] = (usDistance[2] >> 8) & 0xFF;	//US Front Right
	//		data[5] = usDistance[2] & 0xFF;
	//
	//		CAN_COM_Send(CAN_ID_US1, data, 6);
	//
	//		//Sending US2 data (rear)
	//		data[0] = (usDistance[3] >> 8) & 0xFF;	//US Rear Left
	//		data[1] = usDistance[3] & 0xFF;
	//
	//		data[2] = (usDistance[4] >> 8) & 0xFF;	//US Rear Center
	//		data[3] = usDistance[4] & 0xFF;
	//
	//		data[4] = (usDistance[5] >> 8) & 0xFF;	//US Rear Right
	//		data[5] = usDistance[5] & 0xFF;
	//
	//		CAN_COM_Send(CAN_ID_US2, data, 6);
	//
	//		CAN_SEND_US = 0;
	//	}
	//
	//	/* CAN : communication checking */
	//	if (commCheckingRequest) {
	//		data[1] = COMM_CHECKING_ACK;
	//
	//		CAN_COM_Send(CAN_ID_COMM_CHECKING, data, 1); //Send ack
	//
	//		commCheckingRequest = 0;
	//	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2)	{
		WHEELS_OverflowManager();
	} else if (htim->Instance == TIM3) {
		US_OverflowManager();
	}
}

//void APP_PeriodicCountersUpdate(void) {
//	cmpt_can_motors ++;
//	cmpt_us++;
//	cmpt_batt++;
//
//	if (cmpt_can_motors == PERIOD_SEND_MOTORS){
//		CAN_SEND_MOTORS = 1;
//		cmpt_can_motors = 0;
//	}
//	if (cmpt_us == US_MAX_WAIT_TIME_MS){
//		US_FLAG = 1;
//		cmpt_us = 0;
//	}
//	if (cmpt_batt == PERIOD_SEND_BATT){
//		CAN_SEND_BATT = 1;
//		cmpt_batt = 0;
//	}
//}

/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  ch: Character to be printed
 * @retval Character sent
 */
int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}
