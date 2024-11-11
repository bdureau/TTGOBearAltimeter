# TTGOBearAltimeter (firmware)
This is a simple altimeter that can be attached to the nose cone of a rocket, it detect lift off and record altitude, pressure, temperature and acceleration on the 3 axes

<img src="/photos/TTGOAltimeterAccel.jpg" width="35%"><img src="/photos/TTGOAltimeterAltitude.jpg" width="35%">
<img src="/photos/TTGOAltimeterSplashScreen.jpg" width="20%">
<img src="/photos/TTGOAltimeterTemperature.jpg" width="35%">
<img src="/photos/TTGOAltimeterMainScreen.jpg" width="35%">

https://www.youtube.com/watch?v=wF-OxLI5ShA


# Building the code
You will need to download the Arduino ide from the [Arduino web site](https://www.arduino.cc/). 
Make sure that you install ESP32 support
The project depend on the following libraries
  - TFT
  - Adafruit ADXL345
  - Adafruit unified
  - Button2
  - uiwidgets
  
Compile with the following options:

Prior to compiling go to the TFT_eSPI and open up the file User_Setup_Select.h

Comment out the following line

//#include <User_Setup.h> 

and uncomment the following line

#include <User_Setups/Setup25_TTGO_T_Display.h> 
