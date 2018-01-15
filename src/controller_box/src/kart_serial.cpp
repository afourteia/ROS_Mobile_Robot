#include "kart_serial.h"

//Setup serial communication
UKART::UKART(){
	cmdSetup = { 0x55, 0xAC, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0};	//Enter wire clt mode
  cmdSend = { 0x55, 0xAB, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF7};		//Send command
	// Send protocol bit position
	 int SOP = 0;	//SOP
	 int CMD = 1;	//Command type bit
	 int LEN = 2;	//
	 int CTL_BYT = 3;	//
	int ANGVELH = 6;	//Angular velocity MSB
	 int ANGVELL = 7;	//Angular velocity LSB
	 int LINVELH = 8;	//Linear velocity MSB
	 int LINVELL = 9;	//Linear velocity LSB
	 int XOR = 12;	//Parity bit

	// Receieve protocl bit position
	 int votlage_H = 5;
	 int voltage_L = 6;
	 int Err_4 = 3;
	 int Err_3 = 4;
	 int Err_2 = 5;
	 int Err_1 = 6;
	 int LeftCurrent_H   = 3;
	 int LeftCurrent_L   = 4;
	 int RightCurrent_H  = 5;
	 int RightCurrent_L  = 6;
	 int LeftTemp_H  = 3;
	 int LeftTemp_L  = 4;
	 int RightTemp_H = 5;
	 int RightTemp_L = 6;

	//Data recieved
	 unsigned char	WCEMA				= 0xAD;	// Enter wire ctl mode ack
	 unsigned char WCExMA			= 0xAF;	// Exit wire ctl mode ack
	 unsigned char SpInfo			= 0xE6;	// Speed info
	 unsigned char CurInfo 		= 0xE4;	// Current info
	 unsigned char AltInfo			= 0xE3;	// Altitude info
	 unsigned char TempInfo		= 0xE5;	// Temperature info
	 unsigned char SSpInfo			= 0xE2;	// Setting speed info
	 unsigned char YawVoltInfo	= 0xE1;	// Yaw and Voltage info
	 unsigned char PwOFF				= 0xE7;	// Power off detected
	 unsigned char ODOInfo			= 0xE8;	// Odometry info
	 unsigned char VerInfo			= 0xE9;	// Version info
	 unsigned char ChipIDInfo	= 0xEA;	// Chip ID info
	 unsigned char ErrorInfo		= 0xEB;	// Error info
	 unsigned char GACA				= 0xA3;	// Gyro & ACC calibration cmd ack

	//Possible Command values
	 unsigned char BEEP_CTL_BIT 		=0x08	;//Beep command
	 unsigned char MOTOR_RELEASE_BIT	=0X10;	//Motor release command

	//Erro Code bit position
	// needs work

	//constants
	 CartRadius=271.462;	//robot axle radius
	try{
		serial::Serial ser;
		ser.setPort("/dev/ttyUSB0");
		ser.setBaudrate(115200);
		serial::Timeout to =serial::Timeout::simpleTimeout(1000);
		ser.setTimeout(to);
		ser.open();
	}catch(serial::IOException& e){
		ROS_INFO("unable to open port");
		throw serial::IOException("Unable to connect");
	}
	if(ser.isOpen()){
		ROS_INFO("Serial Port Initialized");
		ser.write(cmdSetup,13);
	}
	else{
		throw serial::IOException("Serial Port Not Open");
	}
}

//Set the parity bit
int UKART::parityBit(volatile uint8_t *data, int length){
	char XorVal = 0;
	for (int i = 0; i < length; i++){
		XorVal ^= *data++;
	}
	return XorVal;
}

// Read the serial comm and fetch the data
void UKART::checkReceivedData(){

	ser.read(&cmdrecieve,8);

	switch (cmdrecieve[CMD]) {
		case WCEMA:
			ROS_INFO("Enter wire ctl mode");
			break;
		case WCExMA:
			ROS_INFO("Exit wire ctl mode");
			break;
		case SpInfo:
			ROS_INFO("reporting speed");
			break;
		case CurInfo:
			ROS_INFO("reporting current");
			break;
		case AltInfo:
			ROS_INFO("reporting altitude");
			break;
		case TempInfo:
			ROS_INFO("reporting temperature");
			break;
		case SSpInfo:
			ROS_INFO("reporting speed setting");
			break;
		case YawVoltInfo:
			ROS_INFO("reporting yaw and voltage");
      voltPub = ((cmdrecieve[votlage_H]<<8) + cmdrecieve[voltage_L])/1000.0;
			break;
		case PwOFF:
			ROS_INFO("reporting power off");
			break;
		case ODOInfo:
			ROS_INFO("reporting odometry");
			break;
		case VerInfo:
			ROS_INFO("reporting version");
			break;
		case ChipIDInfo:
			ROS_INFO("reporting chip ID");
			break;
		case ErrorInfo:
			ROS_INFO("reporting error");
			break;
		case GACA:
			ROS_INFO("reporting gyro calibration ack");
			break;
		default:
			ROS_INFO("Kart Controller has nothing to report");
			break;
	};
}

void UKART::setVelocity(int linVelcmd, int angVelcmd){
	cmdSend[LINVELL] = 0xff & linVelcmd;
	cmdSend[LINVELH] = 0xff & (linVelcmd >> 8);
	cmdSend[ANGVELL] = 0xff & angVelcmd;
	cmdSend[ANGVELH] = 0xff & (angVelcmd >> 8);
}

void UKART::beep(int& beepcmd){
	if(beepcmd > 0){
				cmdSend[CTL_BYTE]	= BEEP_CTL_BIT;
				beepcmd--;
	}else{
		cmdSend[CTL_BYTE] = 0;
	}
}

void UKART::send(){
	cmdSend[XOR] = parityBit(&cmdSend, 12);
	ser.write(&cmdSend,13);
}
