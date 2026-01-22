#ifndef __can_H
#define __can_H

#define ID_MOTORS_DATAS 0x200   //Odometry + Motors feedback
#define ID_US1 0x211            //Front US
#define ID_US2 0x221            //Rear US
#define ID_IMU1 0x232           //Magnetic
#define ID_IMU2 0x242           //Angular rate
#define ID_IMU3 0x252           //Linear acceleration
#define ID_IMU4 0x263           //Humidity, pressure, temperature
#define ID_BATT_LEVEL 0x273     //Battery level
#define ID_GPS1 0x272           //Latitude
#define ID_GPS2 0x282           //longitude
#define ID_GPS3 0x292           //Altitude
#define ID_GPS4 0x302           //Status : quality (fix, no fix ...), hAcc, vAcc
#define ID_INA_POWER 0x2A3      //Power

#define ID_CALIBRATION_MODE 0x400
#define ID_COMM_CHECKING 0x410

#define ID_MOTORS_CMD 0x100


#define CALIBRATION_REQUEST	0x1		//frame[0]
#define CALIBRATION_IN_PROGRESS 0x2	//frame[0]
#define CALIBRATION_SUCCESS 0x3		//frame[0]
#define CALIBRATION_FAIL 0x4		//frame[0]

#define CALIBRATION_USER_NEED 0x1	//frame[1]

#define COMM_CHECKING_REQUEST 0x1   //frame[0]
#define COMM_F103_ACK 0x1           //frame[1]
#define COMM_L476_ACK 0x2           //frame[1]


#endif /*__ can_H */