/*
 *  TTGO Bear Altimeter ver0.1
 *  Copyright Boris du Reau 2012-2024
 *  This is using a BMP085 or BMP180 presure sensor
 *  An ADXL345 for the accelerometer
 *  to compile it use ESP32 Dev module
 *  This uses an 24LC512 eeprom to record the flight
 *  
 *  Major changes on version 0.1
 *  Initial version of the code, this is re-using code for the Altimulti firmware
 *  can record altitude, acceleration, temperature and pressure
 * 
 */

#include <TFT_eSPI.h>
#include <Button2.h>
#include <TFT_eWidget.h>               // Widget library
#include <driver/rtc_io.h>

#include "Bear_BMP085.h"
#include "logger_i2c_eeprom.h"
#include "kalman.h"
#include <Adafruit_ADXL345_U.h>
#include <Pangodream_18650_CL.h>

#include "images/bear_altimeters128x128.h"
#include "images/battery_01.h"
#include "images/battery_02.h"
#include "images/battery_03.h"
#include "images/battery_04.h"
#include "images/battery_05.h"



#define BTN_UP 35 // Pinnumber for button for up/previous and select / enter actions (don't change this if you want to use the onboard buttons)
#define BTN_DWN 0 // Pinnumber for button for down/next and back / exit actions (don't change this if you want to use the onboard buttons)

Button2 btnUp(BTN_UP); // Initialize the up button
Button2 btnDwn(BTN_DWN); // Initialize the down button

TFT_eSPI tft = TFT_eSPI();
GraphWidget gr = GraphWidget(&tft);    // Graph widget
// Flight curves are drawn on tft using graph instance
TraceWidget trAltitude = TraceWidget(&gr);    // Altitude
TraceWidget trTemperature = TraceWidget(&gr);
TraceWidget trPressure = TraceWidget(&gr);
TraceWidget trAccelX = TraceWidget(&gr);    // Accel X
TraceWidget trAccelY = TraceWidget(&gr);    // Accel Y
TraceWidget trAccelZ = TraceWidget(&gr);    // Accel Z

//////////////////////////////////////////////////////////////////////
// Global variables
//////////////////////////////////////////////////////////////////////
#define ICON_WIDTH 70
#define ICON_HEIGHT 36
#define STATUS_HEIGHT_BAR ICON_HEIGHT
#define ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))
#define ICON_POS_X (tft.width() - ICON_WIDTH)

#define MIN_USB_VOL 4.7
#define ADC_PIN 34
#define CONV_FACTOR 1.8
#define READS 20

Pangodream_18650_CL BL(ADC_PIN, CONV_FACTOR, READS);

Adafruit_ADXL345_Unified accel345 = Adafruit_ADXL345_Unified();

// Built in button GPIO - adjust for your board
#define BUTTON_GPIO GPIO_NUM_35

bool inGraph = false;

BMP085 bmp;

//EEProm address
logger_I2C_eeprom logger(0x50) ;
// End address of the 512 eeprom
long endAddress = 65536;
// current file number that you are recording
int currentFileNbr = 0;
// EEPROM start address for the flights. Anything before that is the flight index
long currentMemaddress = 200;

//ground level altitude
long initialAltitude;

//stop recording a maximum of 20 seconds after main has fired
long recordingTimeOut = 120 * 1000;
boolean canRecord = true;
boolean exitRecording = false;

long liftoffAltitude = 20;
long lastAltitude;
//current altitude
long currAltitude;
long diplayedFlightNbr =0;
long currentCurveType = 0;


/*
 * drawingText(String text)
 * 
 */
void drawingText(String text) {
  tft.fillRect(0, 0, ICON_POS_X, ICON_HEIGHT, TFT_BLACK);
  tft.setTextDatum(5);
  tft.drawString(text, ICON_POS_X - 2, STATUS_HEIGHT_BAR / 2, 4);
}

/*
 * tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap)
 * 
 * 
 */
bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap)
{
  if ( y >= tft.height() ) return 0;
  tft.pushImage(x, y, w, h, bitmap);
  return 1;
}

/*
 * button_init()
 * 
 */
void button_init()
{
  btnUp.setLongClickHandler([](Button2 & b) {
    // Select
    unsigned int time = b.wasPressedFor();
    if (time >= 3000) {
      Serial.println("Turning off");
      inGraph = false;
      enter_sleep();
    }
  });

  btnUp.setClickHandler([](Button2 & b) {
    // Up
    Serial.println("Changing curve type");// It's called downCmd because it decreases the index of an array. Visually that would mean the selector goes upwards.
    if(inGraph){
      long lastFlightNbr = logger.getLastFlightNbr();
      //Make sure we have no reach the last flight
      if(lastFlightNbr >= diplayedFlightNbr) {
       if(currentCurveType < 3) {
        currentCurveType++;
        drawFlightNbr(diplayedFlightNbr, currentCurveType);
       } else {
          currentCurveType =0;
          drawFlightNbr(diplayedFlightNbr, currentCurveType);
       }  
      } 
    }
  });

  btnDwn.setLongClickHandler([](Button2 & b) {
    // Exit
    Serial.println("Button Down slow");
    unsigned int time = b.wasPressedFor();
    if (time >= 1000 & time < 10000) {
      if (!inGraph) {
        long lastFlightNbr = logger.getLastFlightNbr();
        Serial.print("lastFlightNbr:");
        Serial.println(lastFlightNbr);
        if (!(lastFlightNbr < 0)) {
          inGraph = true;
          diplayedFlightNbr =0;
          drawFlightNbr(diplayedFlightNbr, currentCurveType);  
        } 
      } else {
        inGraph = false;
        tft.init();

        tft.fillScreen(TFT_BLACK);

        // Graph area is 200 pixels wide, 150 high, dark grey background
        gr.createGraph(200, 100, tft.color565(5, 5, 5));
        // x scale units is from 0 to 100, y scale units is 0 to 50
        gr.setGraphScale(0.0, 100.0, 0, 50.0);
        //Serial.println(gr.getTextPadding());
      }
    }
    if (time >= 10000) {
      inGraph = false;
      tft.init();

      tft.fillScreen(TFT_BLACK);

      // Graph area is 200 pixels wide, 150 high, dark grey background
      gr.createGraph(200, 100, tft.color565(5, 5, 5));
      // x scale units is from 0 to 100, y scale units is 0 to 50
      gr.setGraphScale(0.0, 100.0, 0, 50.0);
      Serial.println("Erasing flights!!!");

      // erasing flights
      logger.clearFlightList();
      logger.writeFlightList();
      currentFileNbr = 0;
      currentMemaddress = 201;
    }

  });

  btnDwn.setClickHandler([](Button2 & b) {
    // Down
    Serial.println("Button Down fast"); // It's called upCmd because it increases the index of an array. Visually that would mean the selector goes downwards.
    if(inGraph){
      long lastFlightNbr = logger.getLastFlightNbr();
      //Make sure we have no reach the last flight
      if(lastFlightNbr > diplayedFlightNbr) {
        
        diplayedFlightNbr ++;
        Serial.print("Flight:");
        Serial.println(diplayedFlightNbr);
        drawFlightNbr(diplayedFlightNbr, currentCurveType);
      } else {
        // if not lets go back to the first one if it exists
        if(!(lastFlightNbr < 0)){
          diplayedFlightNbr =0;
          drawFlightNbr(diplayedFlightNbr, currentCurveType);
        }
      }
    }
  });
}

/*
 * button_loop()
 * 
 */
void button_loop()
{
  // Check for button presses
  btnUp.loop();
  btnDwn.loop();
}

/*
 * setup()
 * 
 */
void setup() {
  // initialise the connection
  Wire.begin();
  Serial.begin(115200);
  bmp.begin();

  pinMode(14, OUTPUT);
  digitalWrite(14, HIGH);

  tft.init();
  tft.fillScreen(TFT_BLACK);
  tft.setSwapBytes(true);

  tft.pushImage(6, 0, 128, 128, bear_altimeters128x128);
  tft.drawString("Bear Altimeter", 6, 135);
  tft.drawString("ver 1.0", 6, 145);
  tft.drawString("Copyright", 6, 155);
  tft.drawString("Boris du Reau", 6, 165);
  tft.drawString("2012-2024", 6, 175);
  tft.drawString("Initializing....", 6, 185);
  //beepAltiVersion(MAJOR_VERSION, MINOR_VERSION);
  // init Kalman filter
  KalmanInit();
  // let's do some dummy altitude reading
  // to initialise the Kalman filter
  for (int i = 0; i < 50; i++) {
    ReadAltitude();
  }
  //let's read the launch site altitude
  float sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += ReadAltitude();
    delay(50);
  }
  initialAltitude = (long)(sum / 10.0);


  // init accelerometer
  if (!accel345.begin())
  {
    //There was a problem detecting the ADXL375 ... check your connections
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
  } else {
    accel345.setRange(ADXL345_RANGE_16_G);
  }

  int v_ret;
  v_ret = logger.readFlightList();
  long lastFlightNbr = logger.getLastFlightNbr();
  if (lastFlightNbr < 0)
  {
    currentFileNbr = 0;
    currentMemaddress = 201;
    Serial.println("First flight");
  }
  else
  {
    currentMemaddress = logger.getFlightStop(lastFlightNbr) + 1;
    currentFileNbr = lastFlightNbr + 1;
    Serial.print("flight");
    Serial.println(currentFileNbr);
  }
  canRecord = logger.CanRecord();
  button_init();
  

  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.drawString("altitude", 6, 0);
  // Graph area is 200 pixels wide, 150 high, dark grey background
  gr.createGraph(200, 100, tft.color565(5, 5, 5));
  // x scale units is from 0 to 100, y scale units is 0 to 50
  gr.setGraphScale(0.0, 100.0, 0, 50.0);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.fillScreen(TFT_BLACK);
  tft.setSwapBytes(true);
  tft.setTextFont(2);
}

/*
 * loop()
 * 
 * 
 */
void loop() {

  button_loop();

  if (!inGraph) {
    tft.setCursor (0, STATUS_HEIGHT_BAR);

    if (BL.getBatteryVolts() >= MIN_USB_VOL) {
      drawingText("Chrg");
      tft_output(ICON_POS_X, 0, ICON_WIDTH, 36, (uint16_t*) battery_01);
      delay(500);
      tft_output(ICON_POS_X, 0, ICON_WIDTH, 36, (uint16_t*) battery_02);
      delay(500);
      tft_output(ICON_POS_X, 0, ICON_WIDTH, 36, (uint16_t*) battery_03);
      delay(500);
      tft_output(ICON_POS_X, 0, ICON_WIDTH, 36, (uint16_t*) battery_04);
      delay(500);
      tft_output(ICON_POS_X, 0, ICON_WIDTH, 36, (uint16_t*) battery_05);
      delay(500);
    } else {
      int batteryLevel = BL.getBatteryChargeLevel();
      if (batteryLevel >= 80) {
        tft_output(ICON_POS_X, 0, 70, 36, (uint16_t*) battery_04);
      } else if (batteryLevel < 80 && batteryLevel >= 50 ) {
        tft_output(ICON_POS_X, 0, 70, 36, (uint16_t*) battery_03);
      } else if (batteryLevel < 50 && batteryLevel >= 20 ) {
        tft_output(ICON_POS_X, 0, 70, 36, (uint16_t*) battery_02);
      } else if (batteryLevel < 20 ) {
        tft_output(ICON_POS_X, 0, 70, 36, (uint16_t*) battery_01);
      }
      drawingText(String(batteryLevel) + "%");
    }
    char Altitude [15];
    currAltitude = (long)ReadAltitude() - initialAltitude;
    sprintf(Altitude, "Altitude = %i meters    ", currAltitude );
    tft.setCursor (0, STATUS_HEIGHT_BAR);
    tft.println("                                     ");
    tft.println(Altitude);


    char temp [15];
    sensors_event_t event345;
    accel345.getEvent(&event345);
    sprintf(temp, "x=%3.2f m/s", (float)event345.acceleration.x );
    tft.println("");
    tft.println(temp);
    sprintf(temp, "y=%3.2f m/s", (float)event345.acceleration.y );
    tft.println(temp);
    sprintf(temp, "z=%3.2f m/s", (float)event345.acceleration.z );
    tft.println(temp);

   
    if ( currAltitude > liftoffAltitude ) {
      Serial.println("Recording!!!!");
      recordAltitude();
    }
  }
}


/*
 * ReadAltitude()
 * 
 */

float ReadAltitude()
{
  return KalmanCalc(bmp.readAltitude());
}

/*
 * 
 * enter_sleep()
 * 
 */
void enter_sleep()
{
  tft.fillScreen(TFT_BLACK);
  tft.setRotation(0);
  tft.drawString("turning off...", 6, 185);
  delay(2000);
  pinMode(BUTTON_GPIO, INPUT_PULLUP);
  rtc_gpio_hold_en(BUTTON_GPIO);
  esp_sleep_enable_ext0_wakeup(BUTTON_GPIO, LOW);
  esp_deep_sleep_start();
}

/*
 * drawAxesXY(float minX, float maxX, float minY, float maxY )
 * 
 */

void drawAxesXY(float minX, float maxX, float minY, float maxY, int flightNbr, char *curveName ) {
  tft.fillScreen(TFT_BLACK);
  
  // x scale units is from 0 to 100, y scale units is 0 to 50
  gr.setGraphScale(minX, maxX, minY, maxY);
  // X grid starts at 0 with lines every 10 x-scale units
  // Y grid starts at 0 with lines every 10 y-scale units
  // blue grid
  gr.setGraphGrid(0.0, maxX / 5, 0.0, maxY / 5, TFT_BLUE);

  // Draw empty graph, top left corner at 20,10 on TFT
  gr.drawGraph(30, 10);

  // Draw the x axis scale
  tft.setTextDatum(TC_DATUM); // Top centre text datum
  tft.drawNumber(minX, gr.getPointX(minX), gr.getPointY(0) + 3);
  if(maxX < 1000) {
    tft.drawNumber(maxX / 2, gr.getPointX(maxX / 2), gr.getPointY(0) + 3);
    tft.drawNumber(maxX, gr.getPointX(maxX), gr.getPointY(0) + 3);
    tft.drawString("time(ms)", gr.getPointX(maxX/4),  gr.getPointY(0)+3);   
  } else {
    char temp[10];
    sprintf(temp, "%3.1f",  maxX/1000/2);
    tft.drawString(temp, gr.getPointX(maxX/2),  gr.getPointY(0)+3);
    sprintf(temp, "%3.1f",  maxX/1000);
    tft.drawString(temp, gr.getPointX(maxX)-10,  gr.getPointY(0)+3);
    tft.drawString("time (s)", gr.getPointX(maxX/4),  gr.getPointY(0)+3);
  }
  char flight[15];
  sprintf(flight, "Flight %i  ",  flightNbr);
  tft.drawString(flight, gr.getPointX(maxX)-10,  gr.getPointY(maxY));
  tft.drawString(curveName, gr.getPointX(maxX/3),gr.getPointY(maxY));

  // Draw the y axis scale
  tft.setTextDatum(MR_DATUM); // Middle right text datum
  tft.drawNumber(maxY, gr.getPointX(0.0), gr.getPointY(maxY));
}

/*
 * drawFlightNbr(int flightNbr)
 * 
 */
void drawFlightNbr(int flightNbr, int curveType){
  
  
  logger.getFlightMinAndMax(flightNbr);


  //altitude
  if( curveType == 0) {
    // Start altitude trace
    trAltitude.startTrace(TFT_GREEN);
    drawAxesXY(0.0, logger.getFlightDuration(), 0, (float) logger.getMaxAltitude(),flightNbr, "Altitude (meters)" );
  }

  //pressure
  if(curveType == 1) {
    trPressure.startTrace(TFT_GREY); 
    drawAxesXY(0.0, logger.getFlightDuration(), 0, (float) logger.getMaxPressure(),flightNbr, "Pressure (mBar)" );
  }

  if(curveType == 2) {
    trAccelX.startTrace(TFT_RED); 
    trAccelY.startTrace(TFT_PURPLE);
    trAccelY.startTrace(TFT_YELLOW);
    float maxAccel = 0.0f;
    if(logger.getMaxAccelX() > maxAccel)
      maxAccel = (float)logger.getMaxAccelX();
    if(logger.getMaxAccelY() > maxAccel)
      maxAccel = (float)logger.getMaxAccelY();
    if(logger.getMaxAccelZ() > maxAccel)
      maxAccel = (float)logger.getMaxAccelZ();
        
    drawAxesXY(0.0, logger.getFlightDuration(), 0, (float) maxAccel/1000.0,flightNbr, "Accel X,Y,Z (m/s)" );
  }
  //temperature
  if(curveType == 3) {
    trTemperature.startTrace(TFT_BROWN);
    drawAxesXY(0.0, logger.getFlightDuration(), 0, (float) logger.getMaxTemperature(),flightNbr, "Temp (°C)" );  
  } 
  unsigned long startaddress;
  unsigned long endaddress;

  startaddress = logger.getFlightStart(flightNbr);
  endaddress = logger.getFlightStop(flightNbr);

  if (startaddress > 200)
  {
    unsigned long i = startaddress;
    unsigned long currentTime = 0;
    
    while (i < (endaddress + 1))
    {
      i = logger.readFlight(i) + 1;
      
      currentTime = currentTime + logger.getFlightTimeData();

      //altitude
      if( curveType == 0) {
        long altitude = logger.getFlightAltitudeData();
        trAltitude.addPoint(currentTime, altitude);
      }
      if( curveType == 1) {
        long pressure = logger.getFlightPressureData();
        trPressure.addPoint(currentTime, pressure);
      }
      if( curveType == 2) {
        trAccelX.addPoint( currentTime, (float)logger.getADXL345accelX()/1000.0);
        trAccelY.addPoint( currentTime, (float)logger.getADXL345accelY()/1000.0);
        trAccelZ.addPoint( currentTime, (float)logger.getADXL345accelZ()/1000.0);
      }
      if( curveType == 3) {
        long temperature = logger.getFlightTemperatureData();
        trTemperature.addPoint(currentTime, temperature);
      }    
    }
  }
}


/*
 * recordAltitude()
 * 
 * 
 */
void recordAltitude()
{
  long recordingTimeOut = 120 * 1000;

  exitRecording = false;
  boolean liftOff = false;
  long initialTime = 0;
  lastAltitude = 0;

  while (!exitRecording)
  {
    //read current altitude
    currAltitude = (ReadAltitude() - initialAltitude);

    if ((currAltitude > liftoffAltitude) && !liftOff)
    {
      liftOff = true;
      // save the time
      initialTime = millis();

      if (canRecord)
      {
        //Save start address
        logger.setFlightStartAddress (currentFileNbr, currentMemaddress);
        Serial.println("setFlightStartAddress");
      }
    }
    unsigned long prevTime = 0;
    long prevAltitude = 0;
    // loop until we have reach an altitude of 3 meter
    while (liftOff)
    {
      unsigned long currentTime;
      unsigned long diffTime;

      currAltitude = (ReadAltitude() - initialAltitude);

      currentTime = millis() - initialTime;
      
      prevAltitude = currAltitude;
      diffTime = currentTime - prevTime;
      prevTime = currentTime;
      //display 
      char Altitude [15];
      currAltitude = (long)ReadAltitude() - initialAltitude;
      sprintf(Altitude, "Altitude = %i meters    ", currAltitude );
      tft.setCursor (0, STATUS_HEIGHT_BAR);
      tft.println("Recording in progress .....");
      tft.println(Altitude);


      char temp [15];
      sensors_event_t event345;
      accel345.getEvent(&event345);
      sprintf(temp, "x=%3.2f m/s", (float)event345.acceleration.x );
      tft.println("");
      tft.println(temp);
      sprintf(temp, "y=%3.2f m/s", (float)event345.acceleration.y );
      tft.println(temp);
      sprintf(temp, "z=%3.2f m/s", (float)event345.acceleration.z );
      tft.println(temp);
      
      //record
      if (canRecord)
      {
        logger.setFlightTimeData( diffTime);
        logger.setFlightAltitudeData(currAltitude);
        logger.setFlightTemperatureData((long) bmp.readTemperature());
        logger.setFlightPressureData((long) bmp.readPressure());

        sensors_event_t event345;
        accel345.getEvent(&event345);
        logger.setADXL345accelX((long) 1000 * event345.acceleration.x);
        logger.setADXL345accelY((long) 1000 * event345.acceleration.y);
        logger.setADXL345accelZ((long) 1000 * event345.acceleration.z);


        if ( (currentMemaddress + logger.getSizeOfFlightData())  > endAddress) {
          //flight is full let's save it
          //save end address
          logger.setFlightEndAddress (currentFileNbr, currentMemaddress - 1);
          canRecord = false;
        } else {
          currentMemaddress = logger.writeFastFlight(currentMemaddress);
          currentMemaddress++;
        }
      }

      if ((canRecord  && (currAltitude < 10) ) || (canRecord  && (millis() - initialTime) > recordingTimeOut))
      {
        //end loging
        //store start and end address
        Serial.println("setFlightEndAddress");
        //save end address
        logger.setFlightEndAddress (currentFileNbr, currentMemaddress - 1);
        liftOff = false;

        logger.writeFlightList();
        exitRecording = true;
        if(currentFileNbr < 25)
          currentFileNbr ++;
      }
    } // end while (liftoff)
  } //end while(recording)
}
