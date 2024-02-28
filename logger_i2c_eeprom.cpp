#include "logger_i2c_eeprom.h"
#include "IC2extEEPROM.h"
extEEPROM eep(kbits_512, 1, 64);
logger_I2C_eeprom::logger_I2C_eeprom(uint8_t deviceAddress)
{
  //_deviceAddress = deviceAddress;
  //logger_I2C_eeprom(deviceAddress, LOGGER_I2C_EEPROM_PAGESIZE);
}

void logger_I2C_eeprom::begin()
{
  Wire.begin();
  //initialize Flight structure

}
/*
   clearFlightList()
   Clear the flight list. Rather than clearing the entire eeprom
   let's just reset addresses 0 to 200 which contains the flights addresses

*/
void logger_I2C_eeprom::clearFlightList()
{
  int i;
  for (i = 0; i < 25; i++)
  {
    _FlightConfig[i].flight_start = 0;
    _FlightConfig[i].flight_stop = 0;
  }
}


/*
   readFlightList()

*/
int logger_I2C_eeprom::readFlightList()
{
  eep.read(0, ((byte*)&_FlightConfig), sizeof(_FlightConfig));
  return FLIGHT_LIST_START + sizeof(_FlightConfig) ;
}
/*
   readFlight(int eeaddress)

*/
unsigned long logger_I2C_eeprom::readFlight(unsigned long eeaddress)
{
  eep.read(eeaddress, ((byte*)&_FlightData), sizeof(_FlightData));
  return eeaddress + sizeof(_FlightData);
}

/*
   writeFlightList()

*/
int logger_I2C_eeprom::writeFlightList()
{
  eep.write(FLIGHT_LIST_START, ((byte*)&_FlightConfig), sizeof(_FlightConfig));
  return FLIGHT_LIST_START + sizeof(_FlightConfig);
}

/*
   writeFastFlight(int eeaddress)

*/
unsigned long logger_I2C_eeprom::writeFastFlight(unsigned long eeaddress)
{
  eep.write(eeaddress, ((byte*)&_FlightData), sizeof(_FlightData));
  return eeaddress + sizeof(_FlightData);
}

/*

   getLastFlightNbr()
   Parse the flight index end check if the flight_start address is > 0
   return -1 if no flight have been recorded else return the flight number

*/
int logger_I2C_eeprom::getLastFlightNbr()
{
  int i;
  for (i = 0; i < 25; i++)
  {
    if (_FlightConfig[i].flight_start == 0)
    {
      break;
    }
  }
  i--;
  return i;
}
/*

  eraseLastFlight()

*/
bool logger_I2C_eeprom::eraseLastFlight() {
  int i;
  for (i = 0; i < 25; i++)
  {
    if (_FlightConfig[i].flight_start == 0)
    {
      if (i > 0) {
        _FlightConfig[i - 1].flight_start = 0;
        _FlightConfig[i - 1].flight_stop = 0;
        writeFlightList();
        return true;
      }
    }
  }
  return false;
}
/*

   getLastFlightEndAddress()
   Parse the flight index end check if the flight_start address is > 0
   return -1 if no flight have been recorded else return the flight number

*/
long logger_I2C_eeprom::getLastFlightEndAddress()
{
  int i;
  for (i = 0; i < 25; i++)
  {
    if (_FlightConfig[i].flight_start == 0)
    {
      break;
    }
  }
  i--;
  return _FlightConfig[i].flight_stop;
}

/*

   printFlightList()


*/
int logger_I2C_eeprom::printFlightList()
{
  //retrieve from the eeprom
  int v_ret =  readFlightList();

  //Read the stucture
  int i;
  for (i = 0; i < 25; i++)
  {
    if (_FlightConfig[i].flight_start == 0)
      break;
  }
  return i;
}

void logger_I2C_eeprom::setFlightStartAddress(int flightNbr, long startAddress)
{
  _FlightConfig[flightNbr].flight_start = startAddress;
}

void logger_I2C_eeprom::setFlightEndAddress(int flightNbr, long endAddress)
{
  _FlightConfig[flightNbr].flight_stop = endAddress;
}

void logger_I2C_eeprom::setFlightTimeData( long difftime)
{
  _FlightData.diffTime = difftime;
}
void logger_I2C_eeprom::setFlightAltitudeData( long altitude)
{
  _FlightData.altitude = altitude;
}
void logger_I2C_eeprom::setFlightTemperatureData( long temperature)
{
  _FlightData.temperature = temperature;
}
void logger_I2C_eeprom::setFlightPressureData( long pressure) {
  _FlightData.pressure = pressure;
}

void logger_I2C_eeprom::setADXL345accelX(long accelX)
{
  _FlightData.ADXL345accelX = accelX; 
}
void logger_I2C_eeprom::setADXL345accelY(long accelY)
{
  _FlightData.ADXL345accelY = accelY;
}
void logger_I2C_eeprom::setADXL345accelZ(long accelZ)
{
  _FlightData.ADXL345accelZ = accelZ;
}
    
long logger_I2C_eeprom::getFlightStart(int flightNbr)
{
  return _FlightConfig[flightNbr].flight_start;
}
long logger_I2C_eeprom::getFlightStop(int flightNbr)
{
  return _FlightConfig[flightNbr].flight_stop;
}
long logger_I2C_eeprom::getFlightTimeData()
{
  return _FlightData.diffTime;
}
long logger_I2C_eeprom::getFlightAltitudeData()
{
  return _FlightData.altitude;
}

long logger_I2C_eeprom::getFlightTemperatureData()
{
  return _FlightData.temperature;
}

long logger_I2C_eeprom::getFlightPressureData()
{
  return _FlightData.pressure;
}

long logger_I2C_eeprom::getADXL345accelX()
{
  return _FlightData.ADXL345accelX;
}

long logger_I2C_eeprom::getADXL345accelY()
{
  return _FlightData.ADXL345accelY;
}

long logger_I2C_eeprom::getADXL345accelZ()
{
  return _FlightData.ADXL345accelZ;
}

long logger_I2C_eeprom::getSizeOfFlightData()
{
  return sizeof(_FlightData);
}

long logger_I2C_eeprom::getMinAltitude()
{
  return _FlightMinAndMax.minAltitude;
}

long logger_I2C_eeprom::getMaxAltitude()
{
  return _FlightMinAndMax.maxAltitude;
}

long logger_I2C_eeprom::getMaxTemperature()
{
  return _FlightMinAndMax.maxTemperature;
}

long logger_I2C_eeprom::getMaxPressure()
{
  return _FlightMinAndMax.maxPressure;
}

long logger_I2C_eeprom::getMaxAccelX()
{
  return _FlightMinAndMax.maxAccelX;
}

long logger_I2C_eeprom::getMinAccelX()
{
  return _FlightMinAndMax.minAccelX;
}

long logger_I2C_eeprom::getMinAccelY()
{
  return _FlightMinAndMax.minAccelY;
}
long logger_I2C_eeprom::getMaxAccelY()
{
  return _FlightMinAndMax.maxAccelY;
}

long logger_I2C_eeprom::getMinAccelZ()
{
  return _FlightMinAndMax.minAccelZ;
}

long logger_I2C_eeprom::getMaxAccelZ()
{
  return _FlightMinAndMax.maxAccelZ;
}
long logger_I2C_eeprom::getFlightDuration()
{
  return _FlightMinAndMax.flightDuration;
}
void logger_I2C_eeprom::getFlightMinAndMax(int flightNbr)
{
  unsigned long startaddress;
  unsigned long endaddress;

  startaddress = getFlightStart(flightNbr);
  endaddress = getFlightStop(flightNbr);

  if (startaddress > 200)
  {
    unsigned long i = startaddress;
    unsigned long currentTime = 0;
    _FlightMinAndMax.minAltitude=0;
    _FlightMinAndMax.maxAltitude=0;
    _FlightMinAndMax.minTemperature=0;
    _FlightMinAndMax.maxTemperature=0;
    _FlightMinAndMax.minPressure=0;
    _FlightMinAndMax.maxPressure=0;
    _FlightMinAndMax.minAccelX=0;
    _FlightMinAndMax.maxAccelX=0;
    _FlightMinAndMax.minAccelY=0;
    _FlightMinAndMax.maxAccelY=0;
    _FlightMinAndMax.minAccelZ=0;
    _FlightMinAndMax.maxAccelZ=0;
    _FlightMinAndMax.flightDuration=0;

    while (i < (endaddress + 1))
    {
      i = readFlight(i) + 1;

      _FlightMinAndMax.flightDuration = _FlightMinAndMax.flightDuration + getFlightTimeData();
      long altitude = getFlightAltitudeData();
      if(altitude < _FlightMinAndMax.minAltitude)
        _FlightMinAndMax.minAltitude = altitude;
      if(altitude > _FlightMinAndMax.maxAltitude)
        _FlightMinAndMax.maxAltitude = altitude;
        
      long temperature = getFlightTemperatureData();
      if(temperature < _FlightMinAndMax.minTemperature)
        _FlightMinAndMax.minTemperature=temperature;
      if(temperature > _FlightMinAndMax.maxTemperature)
        _FlightMinAndMax.maxTemperature=temperature;

      long pressure = getFlightPressureData();
      if(pressure < _FlightMinAndMax.minPressure)
        _FlightMinAndMax.minPressure=pressure;
      if(pressure > _FlightMinAndMax.maxTemperature)
        _FlightMinAndMax.maxPressure=pressure;
        
      long accelX = getADXL345accelX();
      if(accelX < _FlightMinAndMax.minAccelX)
        _FlightMinAndMax.minAccelX=accelX;
      if(accelX > _FlightMinAndMax.maxAccelX)
        _FlightMinAndMax.maxAccelX=accelX;

      long accelY = getADXL345accelY();
      if(accelY < _FlightMinAndMax.minAccelY)
        _FlightMinAndMax.minAccelX=accelY;
      if(accelY > _FlightMinAndMax.maxAccelY)
        _FlightMinAndMax.maxAccelX=accelY;
        
      long accelZ = getADXL345accelZ();
      if(accelZ < _FlightMinAndMax.minAccelZ)
        _FlightMinAndMax.minAccelZ=accelZ;
      if(accelZ > _FlightMinAndMax.maxAccelZ)
        _FlightMinAndMax.maxAccelZ=accelZ;
    }
  }
}

void logger_I2C_eeprom::printFlightData(int flightNbr)
{
  unsigned long startaddress;
  unsigned long endaddress;

  startaddress = getFlightStart(flightNbr);
  endaddress = getFlightStop(flightNbr);

  if (startaddress > 200)
  {
    unsigned long i = startaddress;
    unsigned long currentTime = 0;

    while (i < (endaddress + 1))
    {
      i = readFlight(i) + 1;
      char flightData[120] = "";
      char temp[20] = "";
      currentTime = currentTime + getFlightTimeData();
      strcat(flightData, "data,");
      sprintf(temp, "%i,", flightNbr );
      strcat(flightData, temp);
      sprintf(temp, "%i,", currentTime );
      strcat(flightData, temp);
      sprintf(temp, "%i,", getFlightAltitudeData() );
      strcat(flightData, temp);
      sprintf(temp, "%i,", _FlightData.temperature );
      strcat(flightData, temp);
      sprintf(temp, "%i,", _FlightData.pressure );
      strcat(flightData, temp);
      sprintf(temp, "%i,", _FlightData.ADXL345accelX );
      strcat(flightData, temp);
      sprintf(temp, "%i,", _FlightData.ADXL345accelY );
      strcat(flightData, temp);
      sprintf(temp, "%i,", _FlightData.ADXL345accelZ );
      strcat(flightData, temp);

      unsigned int chk = msgChk(flightData, sizeof(flightData));
      sprintf(temp, "%i", chk);
      strcat(flightData, temp);
      strcat(flightData, ";\n");
      
      Serial.print("$");
      Serial.print(flightData);
    }
  }
}
/*
   CanRecord()
   First count the number of flights. It cannot be greter than 25
   if last flight end address is greater than the max possible
   address then the EEprom is full
*/
boolean logger_I2C_eeprom::CanRecord()
{
  long lastFlight;
  lastFlight = getLastFlightNbr();
  if (lastFlight == -1)
    return true;

  if (lastFlight == 24)
  {
    //#ifdef SERIAL_DEBUG
    //Serial.println("25 flights");
    //#endif
    return false;
  }
  // Check if eeprom is full
  if (getFlightStop(lastFlight) > 65500 )
  {
    //#ifdef SERIAL_DEBUG
    //Serial.println("memory is full");
    //#endif
    return false;
  }
  return true;
}

unsigned int logger_I2C_eeprom::msgChk( char * buffer, long length ) {

  long index;
  unsigned int checksum;

  for ( index = 0L, checksum = 0; index < length; checksum += (unsigned int) buffer[index++] );
  return (unsigned int) ( checksum % 256 );

}
