#include kart_serial.h
#include <serial.h>
#include <ros/ros.h>
#include <ros/console.h>

//Setup serial communication
int serialSetup(unsigned char *data){
	try{
		ser.setPort("/dev/ttyUSB0");
		ser.setBaudrate(115200);
		serial::Timeout to =serial::Timeout::simpleTimeout(1000);
		ser.setTimeout(to);
		ser.open();
	}catch(serial::IOExecption& e){
		ROS_INFO("unable to open port");
		return 0;
	}if(ser.isOpen()){
		ROS_INFO("Serial Port Initialized");
		ser.write(data,13);
		return 1;
	}else	return 0;
}

//Set the parity bit
int parityBit(volatile unsigned char *data, int length){

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

		case WCEMA: ROS_INFO("Enter wire ctl mode");
		case WCExMA: ROS_INFO("Exit wire ctl mode");
		case SpInfo:{
			ROS_INFO("reporting speed");
		}
		case CurInfo:{
			ROS_INFO("reporting current");
		}
		case AltInfo:{
			ROS_INFO("reporting altitude");
		}
		case TempInfo:{
			ROS_INFO("reporting temperature");
		}
		case SSpInfo:{
			ROS_INFO("reporting speed setting");
		}
		case YawVoltInfo:{
			ROS_INFO("reporting yaw and voltage");
      voltPub.data = cmdrecieve[]
		}
		case PwOFF:{
			ROS_INFO("reporting power off");
		}
		case ODOInfo:{
			ROS_INFO("reporting odometry");
		}
		case VerInfo:{
			ROS_INFO("reporting version");
		}
		case ChipIDInfo:{
			ROS_INFO("reporting chip ID");
		}
		case ErrorInfo:{
			ROS_INFO("reporting error");
		}
		case GACA:{
			ROS_INFO("reporting gyro calibration ack");
		}
		default: ROS_INFO("Kart Controller has nothing to report");
	}
}
