# Paramters provided by the Ukart controller box
# uses the leftnRight.msg message data type

uint8   ctlEnterAck
uint8   ctlExitAck

LeftnRight mtrRPM
LeftnRight current
LeftnRight temp
uint16 voltage
uint16 yaw
uint16 speedlinGoal
uint16 speedAngGoal
uint16 pitch
uint16 roll
uint32 odom

uint32  version
uint32  chipID
uint32  error


uint8   imuCalibAck
uint8   powerOFF
