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
  - TFT_eSPI
  - Adafruit ADXL345
  - Adafruit unified
  - Button2
  - uiwidgets
  
Compile with the following options:

<img src="/photos/TTGO-T-Display.png" width="35%">

Prior to compiling go to the TFT_eSPI and open up the file User_Setup_Select.h

Comment out the following line
```
//#include <User_Setup.h> 
```
and uncomment the following line
```
#include <User_Setups/Setup25_TTGO_T_Display.h> 
```
You will need to have the ESP32 board support version 2.0.14, anything higher than that may not work !!!

<img src="/photos/Esp32 board.png" width="55%">

# Contributing

If you want to contribute to the project just fork my project or send me some code. 

Report any issue or bug that you have

Suggestions and enhancement are welcome

The code is free for you to download and you do not need to buy anything from me. However it cost money to try out new boards, you need to buy them and fly them so if you want to financially help me you can donate via paypal

| Paypal | 
| ------ |
| [![](https://www.paypalobjects.com/en_US/i/btn/btn_donateCC_LG.gif)](https://www.paypal.com/paypalme/bearaltimeter) | 

# Disclaimer

I am not responsible for any damage that could happen. The code is provided as it is
