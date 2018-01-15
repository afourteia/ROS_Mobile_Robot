#ifndef __KART_SERIAL_H_INCLUDED
#define __KART_SERIAL_H_INCLUDED

#include <serial/serial.h>
#include <ros/ros.h>
#include <ros/console.h>

class UKART{
public:
  UKART();
  int parityBit(volatile unsigned char *data, int length);
  void checkReceivedData();
  double voltPub = 0.0;
  void setVelocity(int linVelcmd, int angVelcmd);
  void UKART::beep(int& beepcmd);
  void UKART::send();

private:
  // Send protocol bit position
  const int SOP = 0;	//SOP
  const int CMD = 1;	//Command type bit
  const int LEN = 2;	//
  const int CTL_BYT = 3;	//
  const int ANGVELH = 6;	//Angular velocity MSB
  const int ANGVELL = 7;	//Angular velocity LSB
  const int LINVELH = 8;	//Linear velocity MSB
  const int LINVELL = 9;	//Linear velocity LSB
  const int XOR = 12;	//Parity bit

  // Receieve protocl bit position
  const int votlage_H = 5;
  const int voltage_L = 6;
  const int Err_4 = 3;
  const int Err_3 = 4;
  const int Err_2 = 5;
  const int Err_1 = 6;
  const int LeftCurrent_H   = 3;
  const int LeftCurrent_L   = 4;
  const int RightCurrent_H  = 5;
  const int RightCurrent_L  = 6;
  const int LeftTemp_H  = 3;
  const int LeftTemp_L  = 4;
  const int RightTemp_H = 5;
  const int RightTemp_L = 6;

  //Data recieved
  const unsigned char	WCEMA				= 0xAD;	// Enter wire ctl mode ack
  const unsigned char WCExMA			= 0xAF;	// Exit wire ctl mode ack
  const unsigned char SpInfo			= 0xE6;	// Speed info
  const unsigned char CurInfo 		= 0xE4;	// Current info
  const unsigned char AltInfo			= 0xE3;	// Altitude info
  const unsigned char TempInfo		= 0xE5;	// Temperature info
  const unsigned char SSpInfo			= 0xE2;	// Setting speed info
  const unsigned char YawVoltInfo	= 0xE1;	// Yaw and Voltage info
  const unsigned char PwOFF				= 0xE7;	// Power off detected
  const unsigned char ODOInfo			= 0xE8;	// Odometry info
  const unsigned char VerInfo			= 0xE9;	// Version info
  const unsigned char ChipIDInfo	= 0xEA;	// Chip ID info
  const unsigned char ErrorInfo		= 0xEB;	// Error info
  const unsigned char GACA				= 0xA3;	// Gyro & ACC calibration cmd ack

  //Possible Command values
  const unsigned char BEEP_CTL_BIT 		=0x08	;//Beep command
  const unsigned char MOTOR_RELEASE_BIT	=0X10;	//Motor release command

  //Erro Code bit position
  // needs work

  //constants
  const double  CartRadius=271.462;	//robot axle radius

  const unsigned char cmdSetup[13] = { 0x55, 0xAC, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0};	//Enter wire clt mode
  unsigned char cmdSend[13] = { 0x55, 0xAB, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF7};		//Send command
  unsigned char cmdrecieve [8];		//recieve info
};


#endif
