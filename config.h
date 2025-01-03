#ifndef _CONFIG_H
#define _CONFIG_H



#include "BluetoothSerial.h"
extern BluetoothSerial SerialBT;
#define SerialCom SerialBT

//uncomment if you are using a BMP085 àr BMP180
//#define BMP085_180

//Uncomment if you are using a BMP280
#define BMP280_sensor

#endif
