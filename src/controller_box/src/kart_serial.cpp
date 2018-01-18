#include "kart_serial.h"
#include <serial/serial.h>

//Setup serial communication
UKART::UKART(){
	voltPub=0.0;
	CartRadius=271.462;	//robot axle radius
	//serial::Serial ser;
	try{
		ser.setPort("/dev/ttyUSB0");
		ser.setBaudrate(115200);
		serial::Timeout to =serial::Timeout::simpleTimeout(1000);
		ser.setTimeout(to);
		ser.open();
	}catch(serial::IOException& e){
		ROS_INFO("unable to open port");
		throw std::invalid_argument( "Unable to open the usb port (likely its the wrong usb name or its busy)" );
	}
	if(ser.isOpen()){
		ROS_INFO("Serial Port Initialized");
		ser.write(&cmdSetup[0],13);
		ROS_INFO("PORT SETUP COMPLETED");
	}else{
		throw std::invalid_argument( "Unable to send serial setup cmd to UKART" );
	}
}

//Set the parity bit
int UKART::parityBit(volatile uint8_t *data, int length){
	//ROS_INFO("Setting Parity Bit");
	char XorVal = 0;
	for (int i = 0; i < length; i++){
		XorVal ^= *data++;
	}
	return XorVal;
}

// Read the serial comm and fetch the data
void UKART::checkReceivedData(){
	//ROS_INFO("Checking recieved data");
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
			ROS_INFO("Left %i", (cmdrecieve[info_4]<<8) + cmdrecieve[info_3]);
			ROS_INFO("Right %i", (cmdrecieve[info_2]<<8) + cmdrecieve[info_1]);
			break;
		case CurInfo:
			//ROS_INFO("reporting current");
			break;
		case AltInfo:
			//ROS_INFO("reporting altitude");
			break;
		case TempInfo:
			//ROS_INFO("reporting temperature");
			break;
		case SSpInfo:
			ROS_INFO("reporting speed setting");
			ROS_INFO("Linear %i",  ((cmdrecieve[info_4]<<8) + cmdrecieve[info_3]));
			ROS_INFO("Angular %i", (cmdrecieve[info_2]<<8) + cmdrecieve[info_1]);
			break;
		case YawVoltInfo:
			//ROS_INFO("reporting yaw and voltage");
      voltPub = ((cmdrecieve[votlage_H]<<8) + cmdrecieve[voltage_L])/1000.0;
			break;
		case PwOFF:
			ROS_INFO("reporting power off");
			break;
		case ODOInfo:
			//ROS_INFO("reporting odometry");
			break;
		case VerInfo:
			//ROS_INFO("reporting version");
			break;
		case ChipIDInfo:
			//ROS_INFO("reporting chip ID");
			break;
		case ErrorInfo:
			ROS_INFO("reporting error");
			ROS_INFO("%i", (cmdrecieve[info_4]<<24) + (cmdrecieve[info_3]<<16) + (cmdrecieve[info_2]<<8) + (cmdrecieve[info_1]));
			break;
		case GACA:
			//ROS_INFO("reporting gyro calibration ack");
			break;
		default:
			//ROS_INFO("Kart Controller has nothing to report");
			break;
	};
}

void UKART::setVelocity(int linVelcmd, int angVelcmd){
	//ROS_INFO("Setting the speed values");
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
	cmdSend[XOR] = parityBit(&cmdSend[0], 12);
	//ROS_INFO("Sending data");
	if(ser.isOpen()){
		ser.write(&cmdSend[0],13);
	}else{
		throw std::invalid_argument( "Unable to send serial cmd to UKART" );
	}
}
