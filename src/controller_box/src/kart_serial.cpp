#include "kart_serial.h"
#include <serial/serial.h>

serial::Serial ser;
const unsigned char cmdSetup[13] = { 0x55, 0xAC, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0};	//Enter wire clt mode
unsigned char cmdSend[13] = { 0x55, 0xAB, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF7};		//Send command
unsigned char cmdrecieve [8];		//recieve info

//Setup serial communication
int serialSetup(const unsigned char *data){
	try{
		ser.setPort("/dev/ttyUSB0");
		ser.setBaudrate(115200);
		serial::Timeout to =serial::Timeout::simpleTimeout(1000);
		ser.setTimeout(to);
		ser.open();
	}catch(serial::IOException& e){
		ROS_INFO("unable to open port");
		return 0;
	}if(ser.isOpen()){
		ROS_INFO("Serial Port Initialized");
		ser.write(data,13);
		return 1;
	}else	return 0;
}

//Set the parity bit
int parityBit(volatile uint8_t *data, int length){

	char XorVal = 0;
	for (int i = 0; i < length; i++){
		XorVal ^= *data++;
	}
	return XorVal;
}

// Read the serial comm and fetch the data
void checkReceivedData(){

	ser.read(&cmdrecieve[0],8);

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
      //voltPub.data = ((cmdrecieve[votlage_H]<<8) + cmdrecieve[voltage_L])/1000.0;
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
