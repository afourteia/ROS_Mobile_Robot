#include "kart_serial.h"

//Setup serial communication
UKART::UKART(){
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
