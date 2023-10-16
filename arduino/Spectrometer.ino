/* This example will read all channels from the AS7341 and print out reported values */

#define AS7341
//#define LCD
//#define TSL2591
//#define BH1750
//#define ANALOG

#ifdef AS7341
#include <Adafruit_AS7341.h>
auto asgain=AS7341_GAIN_0_5X;
Adafruit_AS7341 as7341;

bool init_as7341() {
  if (!as7341.begin()){
    Serial.println("Could not find AS7341 spectrometer, stopping");
    return false;
  } else {
    Serial.print("Found AS7341 spectrometer, configuring...");
  }

  as7341.setATIME(100);
  as7341.setASTEP(999);
  as7341.setGain(asgain);
  as7341.setLEDCurrent(4);
  as7341.enableLED(false);
  Serial.println("done.");
  return true;
}
#endif

#ifdef LCD
#include <SerLCD.h> //Click here to get the library: http://librarymanager/All#SparkFun_SerLCD
SerLCD lcd; // Initialize the library with default I2C address 0x72

bool init_lcd() {
  lcd.begin(Wire); //Set up the LCD for I2C communication

  lcd.setBacklight(0, 0, 0); //Set backlight to bright white
  lcd.setContrast(0); //Set contrast. Lower to 0 for higher contrast.

  lcd.clear(); //Clear the display - this moves the cursor to home position as well

  return true;
}
#endif

#ifdef BH1750
#include <hp_BH1750.h>  //  include the library
hp_BH1750 BH1750;       //  create the sensor

bool init_bh1750() {
  // init the sensor with address pin connetcted to ground
  if (BH1750.begin(BH1750_TO_GROUND)) {
    Serial.println("BH1750 sensor found and initialized.");
  } else {
    Serial.println("No BH1750 sensor found!");
    return false;
  }
  return true;
}
#endif

#ifdef TSL2591
#include "Adafruit_TSL2591.h"
auto tslgain=TSL2591_GAIN_MED;
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); // pass in a number for the sensor identifier (for your use later)

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displayTSLSensorDetails(void)
{
  sensor_t sensor;
  tsl.getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" lux"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" lux"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution, 4); Serial.println(F(" lux"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
  delay(500);
}

/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2591
*/
/**************************************************************************/
void configureTSLSensor(void)
{
  // You can change the gain on the fly, to adapt to brighter/dimmer light situations
  //tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
  tsl.setGain(tslgain);      // 25x gain
  //tsl.setGain(TSL2591_GAIN_HIGH);   // 428x gain
  
  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_200MS);
  tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_400MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_500MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS);  // longest integration time (dim light)

  /* Display the gain and integration time for reference sake */  
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Gain:         "));
  tsl2591Gain_t gain = tsl.getGain();
  switch(gain)
  {
    case TSL2591_GAIN_LOW:
      Serial.println(F("1x (Low)"));
      break;
    case TSL2591_GAIN_MED:
      Serial.println(F("25x (Medium)"));
      break;
    case TSL2591_GAIN_HIGH:
      Serial.println(F("428x (High)"));
      break;
    case TSL2591_GAIN_MAX:
      Serial.println(F("9876x (Max)"));
      break;
  }
  Serial.print  (F("Timing:       "));
  Serial.print((tsl.getTiming() + 1) * 100, DEC); 
  Serial.println(F(" ms"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
}

bool init_tsl2591() {
  if (tsl.begin()) {
    Serial.println(F("Found a TSL2591 sensor"));
  } else {
    Serial.println(F("No sensor found ... check your wiring?"));
    return false;
  }
    
  /* Display some basic information on this sensor */
  displayTSLSensorDetails();
  
  /* Configure the sensor */
  configureTSLSensor();
  return true;
}
#endif

void setup() {
  Serial.begin(115200);

  // Wait for communication with the host computer serial monitor
  while (!Serial) {
    delay(1);
  }
  Serial.println("Startup");
  
  Wire.begin();

  Serial.println("Startup2");

#ifdef AS7341
//Initialize spectrometer
  if(!init_as7341()) for(;;) {
    Serial.println("Error in as7431");
    delay(10);
  }
#endif

#ifdef LCD
  //set up lcd screen
  if(!init_lcd()) for(;;) {
    Serial.println("Error in LCD");
    delay(10);
  }
#endif

#ifdef BH1750
  // Activate BH1750
  if(!init_bh1750()) for(;;) {
    Serial.println("Error in BH1750");
    delay(10);
  }
#endif

#ifdef TSL2591
  // Activate TSL2591
  if(!init_tsl2591()){
    Serial.println("Error in BH1750");
    delay(10);
  }
#endif


  Serial.print("t");
#ifdef ANALOG
  Serial.print(",A0,A1");
#endif
#ifdef TSL2591
  Serial.print(",IR,Full,Visible,TSL_Lux");
#endif
#ifdef BH1750
  Serial.print(",BH_lux");
#endif
#ifdef AS7341
  Serial.print(",Gain,415nm,,445nm,,515nm,,555nm,,590nm,,630nm,,680nm,,Clear,,NearIR,");
#endif
  Serial.println();
}

#ifdef AS7341
uint32_t maxval;

void as_agc() {
    // Cycle the gain of the AS7341
  if(false) {
    // level-based AGC
    if(maxval==65535) {
      if(asgain>AS7341_GAIN_0_5X) {
        asgain=as7341_gain_t(int(asgain)-1);
        //Serial.println("Dropping gain");
        as7341.setGain(asgain);
      } else {
        //Serial.println("Gain minimized");
      }
    }else if(maxval<32767) {
      if(asgain<AS7341_GAIN_256X) {
        asgain=as7341_gain_t(int(asgain)+1);
        //Serial.println("Raising gain");
        as7341.setGain(asgain);
      } else {
        //Serial.println("Gain maxed out");
      }
    }
  } else {
    // Cycle through gains
    if(asgain==AS7341_GAIN_256X) {
      asgain=AS7341_GAIN_0_5X;
    } else {
      asgain=as7341_gain_t(int(asgain)+1);
    }
    as7341.setGain(asgain);
  }
}

void as_PrintChannel(int channel) {
  uint32_t readout=as7341.getChannel(channel);
  uint32_t cal=readout<<(9-asgain);
  maxval=(readout>maxval?readout:maxval);
  Serial.print(" ,");Serial.print(readout);Serial.print(',');Serial.print(cal);
}
#endif

void loop() {
#ifdef BH1750
  BH1750.start();   //starts a measurement
#endif

#ifdef AS7341
  // Read all channels at the same time and store in as7341 object
  if (!as7341.readAllChannels()){
    Serial.println("Error reading all channels!");
    return;
  }
#endif

#ifdef ANALOG
  // read the analog in value:
  auto s0 = analogRead(A0);
  auto s1 = analogRead(A1);
#endif

#ifdef TSL2591
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir, full;
  ir = lum >> 16;
  full = lum & 0xFFFF;
#endif

                    Serial.print(millis());

#ifdef ANALOG
  Serial.print(',');Serial.print(s0);
  Serial.print(',');Serial.println(s1);
#endif

#ifdef TSL2591
  Serial.print(',');Serial.print(ir);
  Serial.print(',');Serial.print(full);
  Serial.print(',');Serial.print(full - ir);
  Serial.print(',');Serial.print(tsl.calculateLux(full, ir), 6);
#endif

#ifdef BH1750
  float lux=BH1750.getLux();
  Serial.print(',');Serial.print(lux);
#endif

#ifdef AS7341
  Serial.print(',');Serial.print(asgain);maxval=0;
  as_PrintChannel(AS7341_CHANNEL_415nm_F1);
  as_PrintChannel(AS7341_CHANNEL_445nm_F2);
  as_PrintChannel(AS7341_CHANNEL_480nm_F3);
  as_PrintChannel(AS7341_CHANNEL_515nm_F4);
  as_PrintChannel(AS7341_CHANNEL_555nm_F5);
  as_PrintChannel(AS7341_CHANNEL_590nm_F6);
  as_PrintChannel(AS7341_CHANNEL_630nm_F7);
  as_PrintChannel(AS7341_CHANNEL_680nm_F8);
  as_PrintChannel(AS7341_CHANNEL_CLEAR)   ;
  as_PrintChannel(AS7341_CHANNEL_NIR)     ;
#endif
  Serial.println();

#ifdef LCD
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Gain");
  lcd.setCursor(8,0);
  lcd.print("Val");
  lcd.setCursor(0, 1);

  lcd.print(int(asgain));
  lcd.setCursor(8, 1);
  lcd.print(maxval);
#endif

#ifdef AS7341
  as_agc();
#endif
}
