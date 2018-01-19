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


    // variables to store information recieved from controller box\
    // should be private members
    uint16_t  voltage;
    uint16_t  yaw;
    uint16_t  mtrRPML;
    uint16_t  mtrRPMR;
    uint16_t  speedlinGoal;
    uint16_t  speedAngGoal;
    uint16_t  currentL;
    uint16_t  currentR;
    uint16_t  pitch;
    uint16_t  roll;
    uint16_t  tempL;
    uint16_t  tempR;
    uint32_t  odom;

    uint32_t  version;
    uint32_t  chipID;
    uint32_t  error;
    uint8_t   ctlExitAck;
    uint8_t   ctlEnterAck;
    uint8_t   imuCalibAck;
    uint8_t   powerOFF;

    const double CartRadius = 271.462;	//robot axle radius;



  private:
    unsigned char cmdSetup[13]  = { 0x55, 0xAC, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF6};		//Enter wire clt mode
    unsigned char cmdCalib[13]  = { 0x55, 0xA2, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};    // IMU calibration command
    unsigned char cmdErClr[13]  = { 0x55, 0xA6, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};     // Clear error command
    //unsigned char cmdExit[13]   =
    unsigned char cmdSend[13]   = { 0x55, 0xAB, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF7};		//Send command
    unsigned char cmdrecieve[8];		//recieve info

    serial::Serial ser;       // Serial object
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
#define info_4  3
#define info_3  4
#define info_2  5
#define info_1  6

//Data recieved
#define WCEMA         0xAD	// Enter wire ctl mode ack
#define WCExMA        0xAF	// Exit wire ctl mode ack
#define SpInfo		    0xE6  // Speed info
#define CurInfo 	    0xE4	// Current info
#define AttdInfo		  0xE3	// Altitude info
#define TempInfo	    0xE5	// Temperature info
#define SSpInfo		    0xE2	// Setting speed info
#define YawVoltInfo   0xE1	// Yaw and Voltage info
#define PwOFF			    0xE7	// Power off detected
#define ODOInfo		    0xE8	// Odometry info
#define VerInfo	      0xE9	// Version info
#define ChipIDInfo    0xEA	// Chip ID info
#define ErrorInfo 	  0xEB	// Error info
#define GACA          0xA3	// Gyro & ACC calibration cmd ack
#define RprtNothin    0xFF  // Nothing to report
//Possible Command values
#define BEEP_CTL_BIT 		    0x08	//Beep command
#define MOTOR_RELEASE_BIT   0X10	//Motor release command


#endif
