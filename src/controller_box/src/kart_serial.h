#ifndef __KART_SERIAL_H_INCLUDED
#define __KART_SERIAL_H_INCLUDED

#include <serial/serial.h>
#include <ros/ros.h>
#include <ros/console.h>

class UKART{
public:
  UKART();
  int parityBit(volatile unsigned char *data, int length);
  void checkReceivedData();
  double voltPub = 0.0;
  void setVelocity(int linVelcmd, int angVelcmd);
  void beep(int& beepcmd);
  void send();
  double voltPub;

  double  CartRadius;

private:


  unsigned char cmdSetup[13];
  unsigned char cmdSend[13];
  unsigned char cmdrecieve [8];		//recieve info
  serial::Serial ser;
};


#endif
