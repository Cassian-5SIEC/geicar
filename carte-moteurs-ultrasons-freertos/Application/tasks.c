/**
 * @file    tasks.c
 * @author  Sebastien DI MERCURIO
 * @version V1.0
 * @date    27 Aout 2025
 * @brief   FreeRTOS tasks, queues, semaphores and timers management.
 * This file contains the implementation of FreeRTOS tasks, queues, semaphores, and timers used in the car application.
 * It defines the tasks for application logic, debugging, ultrasonic sensor management, control loop, and calibration events.
 */

#include "tasks.h"
#include "configuration.h"

#include "app.h"

#include "calibrate.h"
#include "control.h"
#include "ultrasound.h"
#include "wheels.h"
#include "measures.h"
#include "can_communication.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

#include "debug.h"

#if defined (__TESTS__)
#include "tests.h"
#endif /* __TESTS__ */

/* -------------------------------------------------------------------------
 * Déclaration de la tâche TASKS_AppLoop (statique)
 * ------------------------------------------------------------------------- */
void TASKS_AppLoop(void *argument);
/* Buffer pour la pile et le TCB */
static StackType_t xAppLoopTaskStack[ APPLOOP_TASK_STACK_SIZE ];
static StaticTask_t xAppLoopTaskTCB;

/* Handle vers la tâche */
static TaskHandle_t xAppLoopTaskHandle = NULL;

/* -------------------------------------------------------------------------
 * Déclaration de la tâche TASKS_DebugLoop (statique)
 * ------------------------------------------------------------------------- */
void TASKS_DebugLoop(void *argument);
/* Buffer pour la pile et le TCB */
static StackType_t xDebugLoopTaskStack[ DEBUGLOOP_TASK_STACK_SIZE ];
static StaticTask_t xDebugLoopTaskTCB;
/* Handle vers la tâche */
static TaskHandle_t xDebugLoopTaskHandle = NULL;

/* -------------------------------------------------------------------------
 * Déclaration de la tâche TASKS_UltrasoundLoop (statique)
 * ------------------------------------------------------------------------- */
void TASKS_UltrasoundLoop(void *argument);
/* Buffer pour la pile et le TCB */
static StackType_t xUltrasoundLoopTaskStack[ USLOOP_TASK_STACK_SIZE ];
static StaticTask_t xUltrasoundLoopTaskTCB;
/* Handle vers la tâche */
static TaskHandle_t xUltrasoundLoopTaskHandle = NULL;

/* -------------------------------------------------------------------------
 * Déclaration de la tâche TASKS_ControlLoop (statique)
 * ------------------------------------------------------------------------- */
void TASKS_ControlLoop(void *argument);
/* Buffer pour la pile et le TCB */
static StackType_t xControlLoopTaskStack[ CONTROLLOOP_TASK_STACK_SIZE ];
static StaticTask_t xControlLoopTaskTCB;
/* Handle vers la tâche */
static TaskHandle_t xControlLoopTaskHandle = NULL;

/* -------------------------------------------------------------------------
 * Déclaration de la tâche TASKS_CalibrationEvent (statique)
 * ------------------------------------------------------------------------- */
void TASKS_CalibrationEvent(void *argument);
/* Buffer pour la pile et le TCB */
static StackType_t xCalibrationTaskStack[ CALIBRATION_TASK_STACK_SIZE ];
static StaticTask_t xCalibrationTaskTCB;
/* Handle vers la tâche */
static TaskHandle_t xCalibrationTaskHandle = NULL;

/* -------------------------------------------------------------------------
 * Déclaration de la tâche TASKS_CANCommunicationEvent (statique)
 * ------------------------------------------------------------------------- */
void TASKS_CANCommunicationEvent(void *argument);
/* Buffer pour la pile et le TCB */
static StackType_t xCANCommunicationTaskStack[ CAN_COMMUNICATION_TASK_STACK_SIZE ];
static StaticTask_t xCANCommunicationTaskTCB;
/* Handle vers la tâche */
static TaskHandle_t xCANCommunicationTaskHandle = NULL;

/* -------------------------------------------------------------------------
 * Déclaration du buffer pour la queue xAppLoopQueue
 * ------------------------------------------------------------------------- */
static uint8_t ucAppLoopQueueStorageArea[ APPLOOP_QUEUE_LENGTH * APPLOOP_QUEUE_ITEM_SIZE ];
static StaticQueue_t xAppLoopStaticQueue;
QueueHandle_t xAppLoopQueue = NULL;

/* -------------------------------------------------------------------------
 * Déclaration du sémaphore de calibration (statique)
 * ------------------------------------------------------------------------- */
static StaticSemaphore_t xCalibrationSemaphoreBuffer;
SemaphoreHandle_t xCalibrationSemaphore = NULL;

/* -------------------------------------------------------------------------
 * Déclaration du timer pour l'envoi des données moteurs (statique)
 * ------------------------------------------------------------------------- */
static void MotorTimerCallback(TimerHandle_t xTimer);
static StaticTimer_t xMotorTimerBuffer;
static TimerHandle_t xMotorTimer   = NULL;

/* -------------------------------------------------------------------------
 * Déclaration du timer pour l'envoi de la mesure de la batterie (statique)
 * ------------------------------------------------------------------------- */
static void BatteryTimerCallback(TimerHandle_t xTimer);
static StaticTimer_t xBatteryTimerBuffer;
static TimerHandle_t xBatteryTimer   = NULL;

/*
 * @brief  Initialize tasks, queues, semaphores and timers.
 * This function creates the necessary FreeRTOS components for the application.
 * It sets up tasks, queues, semaphores, and timers used in the application.
 */
void TASKS_Init(void) {
	/* Création de la file pour l'application (statiquement) */
	xAppLoopQueue = xQueueCreateStatic(
			APPLOOP_QUEUE_LENGTH,          // nombre d’éléments
			APPLOOP_QUEUE_ITEM_SIZE,       // taille d’un élément
			ucAppLoopQueueStorageArea,    // buffer pour les données
			&xAppLoopStaticQueue          // buffer pour la structure de contrôle
	);

	if (xAppLoopQueue == NULL) {
		// Erreur : pas de mémoire statique ?
		Error_Handler();
	}

	/* Création de la tâche AppLoop (statiquement) */
	xAppLoopTaskHandle = xTaskCreateStatic(
			TASKS_AppLoop,          // fonction de la tâche
			"AppLoop",             // nom (debug)
			APPLOOP_TASK_STACK_SIZE,   // taille pile (en mots de 32 bits)
			NULL,                  // paramètre d’entrée
			APPLOOP_TASK_PRIORITY,     // priorité
			xAppLoopTaskStack,         // buffer pile
			&xAppLoopTaskTCB           // buffer TCB
	);

	if (xAppLoopTaskHandle == NULL) {
		// Erreur : pas de mémoire statique ?
		Error_Handler();
	}

	/* Création de la tâche DebugLoop (statiquement) */
	xDebugLoopTaskHandle = xTaskCreateStatic(
			TASKS_DebugLoop,          // fonction de la tâche
			"DebugLoop",             // nom (debug)
			DEBUGLOOP_TASK_STACK_SIZE,   // taille pile (en mots de 32 bits)
			NULL,                  // paramètre d’entrée
			DEBUGLOOP_TASK_PRIORITY,     // priorité
			xDebugLoopTaskStack,         // buffer pile
			&xDebugLoopTaskTCB           // buffer TCB
	);

	if (xDebugLoopTaskHandle == NULL) {
		// Erreur : pas de mémoire statique ?
		Error_Handler();
	}

	/* Création de la tâche UltrasoundLoop (statiquement) */
	xUltrasoundLoopTaskHandle = xTaskCreateStatic(TASKS_UltrasoundLoop, // fonction de la tâche
			"UltrasoundLoop",             // nom (debug)
			USLOOP_TASK_STACK_SIZE,   // taille pile (en mots de 32 bits)
			NULL,                  // paramètre d’entrée
			USLOOP_TASK_PRIORITY,     // priorité
			xUltrasoundLoopTaskStack,         // buffer pile
			&xUltrasoundLoopTaskTCB           // buffer TCB
	);

	if (xUltrasoundLoopTaskHandle == NULL) {
		// Erreur : pas de mémoire statique ?
		Error_Handler();

	}

	/* Création de la tâche ControlLoop (statiquement) */
	xControlLoopTaskHandle = xTaskCreateStatic(TASKS_ControlLoop, // fonction de la tâche
			"ControlLoop",             // nom (debug)
			CONTROLLOOP_TASK_STACK_SIZE,   // taille pile (en mots de 32 bits)
			NULL,                  // paramètre d’entrée
			CONTROLLOOP_TASK_PRIORITY,     // priorité
			xControlLoopTaskStack,         // buffer pile
			&xControlLoopTaskTCB           // buffer TCB
	);

	if (xControlLoopTaskHandle == NULL) {
		// Erreur : pas de mémoire statique ?
		Error_Handler();
	}

	/* Création de la tâche CalibrationEvent (statiquement) */
	xCalibrationTaskHandle = xTaskCreateStatic(TASKS_CalibrationEvent, // fonction de la tâche
			"CalibrationTask",             // nom (debug)
			CALIBRATION_TASK_STACK_SIZE,   // taille pile (en mots de 32 bits)
			NULL,                  // paramètre d’entrée
			CALIBRATION_TASK_PRIORITY,     // priorité
			xCalibrationTaskStack,         // buffer pile
			&xCalibrationTaskTCB           // buffer TCB
	);

	if (xCalibrationTaskHandle == NULL) {
		// Erreur : pas de mémoire statique ?
		Error_Handler();
	}

	/* Création de la tâche CANCommunicationEvent (statiquement) */
	xCANCommunicationTaskHandle = xTaskCreateStatic(TASKS_CANCommunicationEvent, // fonction de la tâche
			"CanComTask",             // nom (debug)
			CAN_COMMUNICATION_TASK_STACK_SIZE,   // taille pile (en mots de 32 bits)
			NULL,                  // paramètre d’entrée
			CAN_COMMUNICATION_TASK_PRIORITY,     // priorité
			xCANCommunicationTaskStack,         // buffer pile
			&xCANCommunicationTaskTCB           // buffer TCB
	);

	if (xCANCommunicationTaskHandle == NULL) {
		// Erreur : pas de mémoire statique ?
		Error_Handler();
	}

	/* Création du sémaphore de calibration (statiquement) */
	xCalibrationSemaphore = xSemaphoreCreateBinaryStatic(&xCalibrationSemaphoreBuffer);
	if (xCalibrationSemaphore == NULL) {
		// Erreur : pas de mémoire statique ?
		Error_Handler();
	}

	/* Au démarrage, le sémaphore est "pris" */
	xSemaphoreTake(xCalibrationSemaphore, 0);

	/* Timer moteur */
	xMotorTimer = xTimerCreateStatic(
			"MotorTimer",                                  // nom
			pdMS_TO_TICKS(MOTOR_TIMER_PERIOD_MS),          // période
			pdTRUE,                                        // auto-reload
			(void*)0,                                      // identifiant (optionnel)
			MotorTimerCallback,                            // callback
			&xMotorTimerBuffer                             // buffer statique
	);
	configASSERT(xMotorTimer != NULL);
	if (xMotorTimer == NULL) {
		// Erreur : pas de mémoire statique ?
		Error_Handler();
	}

	/* Timer batterie */
	xBatteryTimer = xTimerCreateStatic(
			"BatteryTimer",
			pdMS_TO_TICKS(BATTERY_TIMER_PERIOD_MS),
			pdTRUE,
			(void*)0,
			BatteryTimerCallback,
			&xBatteryTimerBuffer
	);
	configASSERT(xBatteryTimer != NULL);

	if (xTimerStart(xMotorTimer, 0) != pdPASS) {
		// Erreur : pas de mémoire statique ?
		Error_Handler();
	}

	if (xTimerStart(xBatteryTimer, 0) != pdPASS) {
		// Erreur : pas de mémoire statique ?
		Error_Handler();
	}
}

/**
 * @brief  Task function for the main application loop.
 * This function processes messages received in the application queue.
 * It runs indefinitely, handling messages as they arrive.
 * @param  argument: Not used
 */
void TASKS_AppLoop(void *argument ) {
	void *pReceived = NULL;

	for(;;)
	{
#if defined (__TESTS__)
		TESTS_Run(); // Run tests if defined
#else
		/* Attente infinie d’un élément dans la queue */
		if (xQueueReceive(xAppLoopQueue, &pReceived, portMAX_DELAY) == pdPASS)
		{
			if (pReceived != NULL) {

				/* Traitement de l’élément reçu */
				// Exemple : cast et utilisation
				// MyStruct_t *msg = (MyStruct_t*) pReceived;
				APP_Run((AppMessage_typeDef*) pReceived);
			}
		}
#endif /* __TESTS__ */
	}
}

/**
 * @brief  Task function for the debug loop.
 * This function runs periodically to perform debug tasks.
 * It runs indefinitely, executing its logic at defined intervals.
 * @param  argument: Not used
 */
void TASKS_DebugLoop(void *argument) {
	TickType_t xLastWakeTime;
	const TickType_t xPeriod = pdMS_TO_TICKS(DEBUG_LOOP_PERIOD_MS);

	/* Initialise la référence de temps */
	xLastWakeTime = xTaskGetTickCount();

	for (;;) {
		DEBUG_PrintPeriodicInfo();

		// Time is compensated from others events that can make processing longer
		vTaskDelayUntil(&xLastWakeTime, xPeriod);
	}
}

/**
 * @brief  Task function for the ultrasound measurement loop.
 * This function continuously triggers ultrasonic measurements.
 * It runs indefinitely, starting new measurements as soon as the previous ones are finished.
 * @param  argument: Not used
 */
void TASKS_UltrasoundLoop(void *argument) {
	// no waiting time here : ultrasonic sensors measurements are started as soon as previous one are finished
	for (;;) {
		US_StartMeasurements();
	}
}

/**
 * @brief  Task function for the car control loop.
 * This function runs periodically to manage the car's control system.
 * It executes the control logic at defined intervals.
 * @param  argument: Not used
 */
void TASKS_ControlLoop(void *argument) {
	TickType_t xLastWakeTime;
	const TickType_t xPeriod = pdMS_TO_TICKS(PERIOD_CAR_CONTROL_LOOP);

	/* Initialise la référence de temps */
	xLastWakeTime = xTaskGetTickCount();

	for (;;) {

		CAR_CONTROL_Manage();

		// Wait for next control loop period
		// Time is compensated from others events that can make processing longer
		vTaskDelayUntil(&xLastWakeTime, xPeriod);
	}
	//		if (mode == 0) {	//Calibration Mode
	//			CAL_SteeringCalibration();
	//			mode = 1;
	//		} else {	//Control Mode
	//			CAR_CONTROL_Manage(leftRearSpeed,rightRearSpeed, steeringAngle);
	//		}
}

/**
 * @brief  Task function for handling calibration events.
 * This function waits for a semaphore to be given, indicating a calibration request.
 * Upon receiving the semaphore, it performs the steering calibration.
 * @param  argument: Not used
 */
void TASKS_CalibrationEvent(void *argument) {
	// non periodic task, triggered by semaphore when calibration request is received

	for (;;) {
		// Exemple : tâche de gestion du CAN
		// Attente sur semaphore
		xSemaphoreTake(xCalibrationSemaphore, portMAX_DELAY);

		// Suspension de la tache de controle moteur (eviter les interferences)
		vTaskSuspend(xControlLoopTaskHandle);

		// Calibration de la direction
		CAL_SteeringCalibration();

		// Reprise de la tache de controle moteur
		vTaskResume(xControlLoopTaskHandle);
	}
}

/**
 * @brief  Task function for handling CAN communication events.
 * This function processes incoming CAN messages.
 * It runs indefinitely, handling CAN communication as messages are received.
 * @param  argument: Not used
 */
void TASKS_CANCommunicationEvent(void *argument) {
	CAN_COM_ReceiveTask();

	for (;;) {
		// This function never returns, it runs indefinitely
		vTaskDelay(pdMS_TO_TICKS(1000)); // Just to avoid compiler warning
	}
}

/**
 * @brief  Callback function for the motor timer.
 * This function is called when the motor timer expires.
 * It performs sending motors measurements.
 */
static void MotorTimerCallback(TimerHandle_t xTimer) {
	/* Send motors measurements to application main loop, for CAN formating */
	WHEELS_SendMesures();
}

/**
 * @brief  Callback function for the battery timer.
 * This function is called when the battery timer expires.
 * It performs sending battery measurements.
 */
static void BatteryTimerCallback(TimerHandle_t xTimer) {
	/* Send battery level to application main loop, for CAN formating */
	MEASURES_SendBatteryLevel();
}


