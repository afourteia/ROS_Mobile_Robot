#include "kart_serial.h"
#include <serial/serial.h>

//Setup serial communication
UKART::UKART(){
	voltPub=0;
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
	ROS_DEBUG("Setting Parity Bit");
	char XorVal = 0;
	for (int i = 0; i < length; i++){
		XorVal ^= *data++;
	}
	return XorVal;
}

// Read the serial comm and assign the data to the public variables
// It sets a publish flag as well
uint8_t UKART::checkReceivedData(){
	ROS_DEBUG("Checking recieved data");
	ser.read(&cmdrecieve[0],8);
	switch (cmdrecieve[CMD]) {
		case WCEMA:
			ROS_DEBUG("Enter wire ctl mode");
			return WCEMA;
			break;
		case WCExMA:
			ROS_DEBUG("Exit wire ctl mode");
			return WCExMA;
			break;
		case SpInfo:
			ROS_DEBUG("reporting motor RPM");
			mtrRPML = (cmdrecieve[info_4]<<8) + cmdrecieve[info_3];
			mtrRPMR = (cmdrecieve[info_2]<<8) + cmdrecieve[info_1];
			return SpInfo;
			break;
		case CurInfo:
			ROS_DEBUG("reporting current");
			currentL = (cmdrecieve[info_4]<<8) + cmdrecieve[info_3];
			currentR = (cmdrecieve[info_2]<<8) + cmdrecieve[info_1];
			return CurInfo;
			break;
		case AttdInfo:
			ROS_DEBUG("reporting altitude");
			pitch = (cmdrecieve[info_4]<<8) + cmdrecieve[info_3];
			roll = (cmdrecieve[info_2]<<8) + cmdrecieve[info_1];
			return AttdInfo;
			break;
		case TempInfo:
			ROS_DEBUG("reporting temperature");
			tempL = (cmdrecieve[info_4]<<8) + cmdrecieve[info_3];
			tempR = (cmdrecieve[info_2]<<8) + cmdrecieve[info_1];
			return TempInfo;
			break;
		case SSpInfo:
			ROS_DEBUG("reporting speed setting");
			speedlinGoal = (cmdrecieve[info_4]<<8) + cmdrecieve[info_3];
			speedAngGoal = (cmdrecieve[info_2]<<8) + cmdrecieve[info_1];
			return SSpInfo;
			break;
		case YawVoltInfo:
			ROS_DEBUG("reporting yaw and voltage");
			yaw = (cmdrecieve[info_4]<<8) + cmdrecieve[info_3];
			voltage = (cmdrecieve[info_2]<<8) + cmdrecieve[info_1];
			return YawVoltInfo;
			break;
		case PwOFF:
			ROS_DEBUG("reporting power off");
			return PwOFF;
			break;
		case ODOInfo:
			ROS_DEBUG("reporting odometry");
			odom = (cmdrecieve[info_4]<<24) + (cmdrecieve[info_3]<<16)
			 	+ (cmdrecieve[info_2]<<8) + (cmdrecieve[info_1]);
			return ODOInfo;
			break;
		case VerInfo:
			ROS_DEBUG("reporting version");
			version = (cmdrecieve[info_4]<<24) + (cmdrecieve[info_3]<<16)
			 	+ (cmdrecieve[info_2]<<8) + (cmdrecieve[info_1]);
			return VerInfo;
			break;
		case ChipIDInfo:
			ROS_DEBUG("reporting chip ID");
			chipID = (cmdrecieve[info_4]<<24) + (cmdrecieve[info_3]<<16)
			 	+ (cmdrecieve[info_2]<<8) + (cmdrecieve[info_1]);
			return ChipIDInfo;
			break;
		case ErrorInfo:
			ROS_DEBUG("reporting error");
			error = (cmdrecieve[info_4]<<24) + (cmdrecieve[info_3]<<16)
			 	+ (cmdrecieve[info_2]<<8) + (cmdrecieve[info_1]);
			return ErrorInfo;
			break;
		case GACA:
			ROS_DEBUG("reporting gyro calibration ack");
			return GACA;
			break;
		default:
			ROS_DEBUG("Kart Controller has nothing to report");
			return 0xFF; // nothing to report
			break;
	};
}

void UKART::setVelocity(int linVelcmd, int angVelcmd){
	ROS_DEBUG("Setting the speed values");
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

void UKART::calibrateIMU(int& imuCalibcmd){
	if(imuCalibcmd > 0){
		ser.write(&cmdCalib[0],13);
		imuCalibcmd--;
	}
}

void UKART::clearError(int& clrErrorcmd){
	if(clrErrorcmd > 0){
		ser.write(&cmdErClr[0],13);
		clrErrorcmd--;
	}
}

void UKART::releaseMotor(int& rlsmotorcmd){
	if(rlsmotorcmd > 0){
		cmdSend[CTL_BYTE]	= MOTOR_RELEASE_BIT;
		rlsmotorcmd--;
	}
}

void UKART::isConnected(int& serCondition){
	if(ser.isOpen()){
		serCondition = 1;
	}else{
		serCondition = 0;
	}
}

void UKART::send(){
	cmdSend[XOR] = parityBit(&cmdSend[0], 12);
	ROS_DEBUG("Sending data");
	if(ser.isOpen()){
		ser.write(&cmdSend[0],13);
	}else{
		throw std::invalid_argument( "Unable to send serial cmd to UKART" );
	}
}
