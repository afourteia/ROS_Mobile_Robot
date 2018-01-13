#ifndef __KART_SERIAL_H_INCLUDED
#define __KART_SERIAL_H_INCLUDED

#include <serial/serial.h>
#include <ros/ros.h>
#include <ros/console.h>


// Send protocol bit position
#define SOP		0	//SOP
#define CMD		1	//Command type bit
#define LEN		2	//
#define CTL_BYTE	3	//
#define ANGVELH		6	//Angular velocity MSB
#define ANGVELL		7	//Angular velocity LSB
#define LINVELH		8	//Linear velocity MSB
#define LINVELL		9	//Linear velocity LSB
#define XOR		12	//Parity bit


// Receieve protocl bit position
#define votlage_H 5
#define voltage_L 6
#define Err_4 3
#define Err_3 4
#define Err_2 5
#define Err_1 6
#define LeftCurrent_H   3
#define LeftCurrent_L   4
#define RightCurrent_H  5
#define RightCurrent_L  6
#define LeftTemp_H  3
#define LeftTemp_L  4
#define RightTemp_H 5
#define RightTemp_L 6


//Data recieved
#define	WCEMA				0xAD	// Enter wire ctl mode ack
#define WCExMA			0xAF	// Exit wire ctl mode ack
#define SpInfo			0xE6	// Speed info
#define CurInfo 		0xE4	// Current info
#define AltInfo			0xE3	// Altitude info
#define TempInfo		0xE5	// Temperature info
#define SSpInfo			0xE2	// Setting speed info
#define YawVoltInfo	0xE1	// Yaw and Voltage info
#define PwOFF				0xE7	// Power off detected
#define ODOInfo			0xE8	// Odometry info
#define VerInfo			0xE9	// Version info
#define ChipIDInfo	0xEA	// Chip ID info
#define ErrorInfo		0xEB	// Error info
#define GACA				0xA3	// Gyro & ACC calibration cmd ack

//Possible Command values
#define BEEP_CTL_BIT 		0x08	//Beep command
#define MOTOR_RELEASE_BIT	0X10	//Motor release command

//Erro Code bit position
// needs work

//constants
#define  CartRadius	271.462	//robot axle radius


//Functions

void checkReceivedData();
int parityBit(volatile unsigned char *data, int length);
int serialSetup(const unsigned char *data);


extern serial::Serial ser;
extern const unsigned char cmdSetup[13];	//Enter wire clt mode
extern unsigned char cmdSend[13];		//Send command
extern unsigned char cmdrecieve[8];		//recieve info

#endif
