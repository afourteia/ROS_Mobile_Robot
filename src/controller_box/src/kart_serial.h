#ifndef __KART_SERIAL_H_INCLUDED
#define __KART_SERIAL_H_INCLUDED

#include <serial/serial.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <stdexcept>

class UKART{
  public:
    UKART();
    int parityBit(volatile unsigned char *data, int length);
    void checkReceivedData();
    void setVelocity(int linVelcmd, int angVelcmd);
    void beep(int& beepcmd);
    void calibrateIMU(int& imuCalibcmd);
    void clearError(int& clrErrorcmd);
    void releaseMotor(int& rlsmotorcmd);
    void isConnected(int& serCondition);
    void send();




    double CartRadius;



  private:
    unsigned char cmdSetup[13]  = { 0x55, 0xAC, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF6};		//Enter wire clt mode
    unsigned char cmdCalib[13]  = { 0x55, 0xA2, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};    // IMU calibration command
    unsigned char cmdErClr[13]  = { 0x55, 0xA6, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};     // Clear error command
    //unsigned char cmdExit[13]   =
    unsigned char cmdSend[13]   = { 0x55, 0xAB, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF7};		//Send command
    unsigned char cmdrecieve[8];		//recieve info

    serial::Serial ser;       // Serial object

    // variables to store information recieved from controller box
    uint16_t  voltageInfo;
    uint16_t  yawInfo;
    uint16_t  mtrRPML;
    uint16_t  mtrRPMR;
    uint16_t  speedlinGoal;
    uint16_t  speedAngGoal;
    uint16_t  currentLInfo;
    uint16_t  currentRInfo;
    uint16_t  pitchInfo;
    uint16_t  rollInfo;
    uint16_t  tempLInfo;
    uint16_t  tempRInfo;
    uint32_t  odomInfo;
    uint32_t  versionInfo;
    uint32_t  chipID;
    uint32_t  errorInfo;
    uint8_t   ctlExitAck;
    uint8_t   ctlEnterAck;
    uint8_t   imuCalibAck;
    uint8_t   powerOFF;
};
  // Send protocol bit position
  #define SOP  0	//SOP
  #define CMD  1	//Command type bit
  #define LEN  2	//
  #define CTL_BYTE  3	//
  #define ANGVELH  6	//Angular velocity MSB
  #define ANGVELL  7	//Angular velocity LSB
  #define LINVELH  8	//Linear velocity MSB
  #define LINVELL  9	//Linear velocity LSB
  #define XOR  12	//Parity bit

  // Receieve protocl bit position
  #define votlage_H  5
  #define voltage_L  6
  #define info_4  3
  #define info_3  4
  #define info_2  5
  #define info_1  6
  #define LeftCurrent_H    3
  #define LeftCurrent_L    4
  #define RightCurrent_H   5
  #define RightCurrent_L   6
  #define LeftTemp_H   3
  #define LeftTemp_L   4
  #define RightTemp_H  5
  #define RightTemp_L 6

  //Data recieved
  #define WCEMA         0xAD	// Enter wire ctl mode ack
  #define WCExMA        0xAF	// Exit wire ctl mode ack
  #define SpInfo		    0xE6  // Speed info
  #define CurInfo 	    0xE4	// Current info
  #define AltInfo		    0xE3	// Altitude info
  #define TempInfo	    0xE5	// Temperature info
  #define SSpInfo		    0xE2	// Setting speed info
  #define YawVoltInfo   0xE1	// Yaw and Voltage info
  #define PwOFF			    0xE7	// Power off detected
  #define ODOInfo		    0xE8	// Odometry info
  #define VerInfo	      0xE9	// Version info
  #define ChipIDInfo    0xEA	// Chip ID info
  #define ErrorInfo 	  0xEB	// Error info
  #define GACA          0xA3	// Gyro & ACC calibration cmd ack

  //Possible Command values
  #define BEEP_CTL_BIT 		    0x08	//Beep command
  #define MOTOR_RELEASE_BIT   0X10	//Motor release command


#endif
