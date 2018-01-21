# Paramters provided by the Ukart controller box
# uses the leftnRight.msg message data type

int8   ctlMode

LeftnRight mtrRPM
LeftnRight current
LeftnRight temp
LeftnRight mtrRPMgoal

uint16 voltage
int16 yaw

int16 pitch
int16 roll
int32 odom

uint32  version
uint32  chipID
uint32  error


uint8   imuCalibAck
uint8   powerOFF

uint8   serCondition
