#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include "SoftwareSerial.h"
#include <Adafruit_Sensor.h>

//SCP30_eCO2
#include "Adafruit_SGP30.h"

//K30_pCO2
int analogPin = 0;
int val = 0;
int volt;
int pCO2;

//MS5803_pressure
#include <SparkFun_MS5803_I2C.h>
MS5803 sensor(ADDRESS_HIGH);

//Create variables to store results
float temperature_c;
float temp_box;
double pressure_abs, pressure_relative, altitude_delta, pressure_baseline;

// Create Variable to store altitude in (m) for calculations;
double base_altitude = 1655.0; //may change to the correct base altitude

Adafruit_SGP30 sgp;
uint32_t getAbsoluteHumidity(float temperature, float humidity) {
    // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
    const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
    const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
    return absoluteHumidityScaled;
}

#define aref_voltage 3.3         // we tie 3.3V to ARef and measure it with a multimeter!

//SoftwareSerial K_30_Serial(8,9); // Virtual serial port for k30 on pinS 8,9 (Rx,Tx). Logger Shield uses 10,12,13
File logfile;  // the logging file

//SD_Card
const int chipSelect = 10; // for the data logging shield, we use digital pin 10 for the SD cs line

//RTC_time
RTC_DS1307 RTC; // define the Real Time Clock object

//millis_time
unsigned long time;

//temperature
#define ONE_WIRE_BUS 2
  OneWire oneWire(ONE_WIRE_BUS);
  DallasTemperature sensors(&oneWire);

void setup(void) {

  Serial.begin(9600);

  Serial.print("Initializing SD card...");   // initialize the SD card
  pinMode(10, OUTPUT); // default chip select pin set to output, even if you don't use it:
  if (!SD.begin(chipSelect)) {  // see if the card is present and can be initialized:
    error("Card failed, or not present");
  }
#if ECHO_TO_SERIAL
  Serial.println("card initialized.");
#endif

  char filename[] = "M_WGAS00.TXT";  // create a new file
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (!SD.exists(filename)) {  // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE);
      break;  // leave the loop
    }
  }
  if (!logfile)
    error("couldnt create file");

  Serial.print("Logging to file: ");
  Serial.println(filename);
  Wire.begin();  // connect to RTC
//  analogReference(EXTERNAL); // If you want to set the aref to something other than 5v

//SCP30_eCO2

  Serial.println("SGP30 test");
  logfile.println("SGP30 test");
  if (! sgp.begin()){
    Serial.println("Sensor not found :(");
    logfile.println("Sensor not found :(");
//    while (1);
  }
 
  Serial.println("Found SGP30 serial #");
  logfile.println("Found SGP30 serial #");

//MS5803_pressure
  sensor.reset();
  sensor.begin();
  pressure_baseline = sensor.getPressure(ADC_4096);
  
//to signal the unit is in good condition
  digitalWrite(4, HIGH); 
  delay(60000);
  digitalWrite(4, LOW);
}

int counter = 0;

void loop()  {
  Serial.begin(9600);
//  Serial.println("starting loop");
  //mosfet
  //testing
/*
  digitalWrite(4, LOW); //turn off the power to air pump
  delay(1500);
  digitalWrite(4, HIGH); //turn on the power to air pump
  delay(1000);
*/
  //mosfet
  //collecting data in the field
  //every 15 minutes, record date, time and all sensors reading (digital pin )
  //1 minute before reading CO2 and CH4, start the air pump (digital pin )
  //30 seconds before reading sensors, turn on power to sensors (warm up)

  digitalWrite(4, LOW);
  delay(840000);
  digitalWrite(4, HIGH);
  delay(60000);
 
//  Serial.println("before send request");

//RCT time
  DateTime now;
  now = RTC.now();

//millis_time
time = millis();

//temperature
  sensors.requestTemperatures();

//SCP30_eCO2
   if (! sgp.IAQmeasure()) {
    Serial.println("eCO2 Measurement failed");
    logfile.println("eCO2 Measurement failed");
//    return;
  }

//MS5803_pressure
  temperature_c = sensor.getTemperature(CELSIUS, ADC_512);
  pressure_abs = sensor.getPressure(ADC_4096);
  pressure_relative = sealevel(pressure_abs, base_altitude);
  altitude_delta = altitude(pressure_abs , pressure_baseline);

//K30_pCO2
  volt = analogRead(analogPin);
  pCO2 = map(volt, 0, 1023, 0, 2450);

  logfile.print(now.year(), DEC);
  logfile.print("/");
  logfile.print(now.month(), DEC);
  logfile.print("/");
  logfile.print(now.day(), DEC);
  logfile.print(" ");
  logfile.print(now.hour(), DEC);
  logfile.print(":");
  logfile.print(now.minute(), DEC);
  logfile.print(":");
  logfile.print(now.second(), DEC);
  logfile.print(" ");

  logfile.print("Time(milli_seconds)");
  logfile.print(" ");
  logfile.print(time);
  logfile.print(" ");
  logfile.print("Temp_Air(^C):");
  logfile.print(sensors.getTempCByIndex(0)+3.0);
  logfile.print(" ");
  logfile.print("eCO2(ppm):");
  logfile.print(" ");
  logfile.print(sgp.eCO2);
  logfile.print(" ");

//K30_pCO2
  logfile.print("pCO2(ppm):");
  logfile.print(" ");
  logfile.print(pCO2);
  logfile.print(" ");
  logfile.print("pCO2(mV):");
  logfile.print(" ");
  logfile.print(volt);
  logfile.print(" ");
  Serial.println(pCO2);
  
  logfile.print("Temp_Box(^C):");
  logfile.print(" ");
  logfile.print(temperature_c);
  logfile.print(" ");
  logfile.print("Pressure_abs(mbar):");
  logfile.print(" ");
  logfile.print(pressure_abs);
  logfile.print(" ");
  logfile.print("Altitude_Change(m):");
  logfile.print(" ");
  logfile.print(altitude_delta);
  logfile.println ();

//  Serial.print("Time; ");
//  Serial.print(time);

/*
  Serial.print(now.year(), DEC);
  Serial.print("/");
  Serial.print(now.month(), DEC);
  Serial.print("/");
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(":");
  Serial.print(now.minute(), DEC);
  Serial.print(":");
  Serial.print(now.second(), DEC);
  Serial.print(" ");
  Serial.print("Temp_Air(^C):");
  Serial.print(sensors.getTempCByIndex(0)+3.0);
  Serial.print(" ");
  Serial.print("eCO2 ");
  Serial.print(sgp.eCO2);
  Serial.print(" ppm");
  Serial.print(" ");
  Serial.print("Temp_Box(^C):");
  Serial.print(" ");
  Serial.print(temp_box);
  Serial.print(" ");
  Serial.print("Pressure_abs(mbar):");
  Serial.print(" ");
  Serial.print(pressure_abs);
  Serial.print(" Altitude_Change(m):");
  Serial.print(" ");
  Serial.print(altitude_delta);  
  Serial.println ();
*/
  logfile.flush();

//SCP30_eCO2
counter++;
  if (counter == 30) {
    counter = 0;
    uint16_t TVOC_base, eCO2_base;
    if (! sgp.getIAQBaseline(&eCO2_base, &TVOC_base)) {
    Serial.println("Failed to get baseline readings");
      return;
    }
  }
}

void error(const char * str) {
  Serial.print("error: ");
  Serial.println(str);
  while(1);
}

//MS5803_pressure
double sealevel(double P, double A)
// Given a pressure P (mbar) taken at a specific altitude (meters),
// return the equivalent pressure (mbar) at sea level.
// This produces pressure readings that can be used for weather measurements.
{
  return (P / pow(1 - (A / 44330.0), 5.255));
}
double altitude(double P, double P0)
// Given a pressure measurement P (mbar) and the pressure at a baseline P0 (mbar),
// return altitude (meters) above baseline.
{
  return (44330.0 * (1 - pow(P / P0, 1 / 5.255)));
}
