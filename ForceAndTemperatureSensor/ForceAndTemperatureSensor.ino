#include "setup.h"
#include <HX711_ADC.h>
#include <OneWireNg_CurrentPlatform.h>
#include <DallasTemperature.h>

// Initialize some global variables
float fMean, allTimeSum, allTimeSamples, force, last_force = 0;
float t, lastT, t_offset = 0; // Keep track of time
float temperature;

// Declaration of externals used in this file
//extern int getCalibration();

HX711_ADC hx711(HX711_DOUT, HX711_SCK);                           // HX711 constructor
OneWire oneWireObject(ONE_WIRE_BUS);   // OneWire constructor
DallasTemperature ds18b20(&oneWireObject);  // DallasTemperature constructor
int getCalibration();

void setup() {
  
  uint8_t hx711Cal;
  // put your setup code here, to run once:
  while(!Serial);

  // Initialize force statistics
  fMean = allTimeSum = allTimeSamples = 0;

  // Initialize the force sensor
  hx711.begin();
  hx711.setSamplesInUse(SMOOTHING);
  if( DEBUG == 2){
    Serial.print("Initializing Load Cell and Taring....");
  }
  hx711.start(2000, true); // Settle for 2s, then tare
   if( DEBUG == 2){
    Serial.println("Done.");
  }

#ifdef OVERRIDE_CALIBRATION
  hx711Cal = OVERRIDE_CALIBRATION;
  if ( DEBUG == 2)
  {
    Serial.println("Calibration overridden with value " + String(hx711Cal));
  }
#else
  hx711Cal = getCalibration();
#endif

  hx711.setCalFactor(hx711Cal);

  if ( DEBUG == 2)
  {
    Serial.println("Finished initializing load cell.");
  }

 /* if( DEBUG==2){
    Serial.print("Intializing DS18B20 temperature sensor....");
  }
  ds18b20.begin();
  if(int tmp = ds18b20.getDS18Count()){
    Serial.print("found ");
    Serial.print(tmp);
    Serial.println(" ds18 temperature device(s).");
  }else{
    Serial.println("NO DS18 TEMPERATURE DEVICES FOUND");
  }
  if( DEBUG == 2){
    Serial.println("Done.\n---------------------------");
  }*/
  t_offset = millis()/1000;  // Start the clock from here
}

void loop() {
  // put your main code here, to run repeatedly:
  float degC, degF;

  if( Serial.available())
  {
    char serdata = Serial.read();  
    if( serdata == '1')
    {
      serdata = (char) 0;
      hx711.tareNoDelay();
    }
  }

  // Get smoothed value from the dataset:
  if ( hx711.update())
  {
    // All of the time-based logic will blow up when millis() overflows (~49 days).
    if (millis() > ((lastT * 1000) + DATA_INTERVAL))
    {
      force = hx711.getData();                   // Read the load cell value as a float (4 bytes on 8-bit AVRs)
      if( INVERT_FORCES)
      {
        force = -force;
      }
      
      t = (float(millis()) / 1000 - t_offset); // Elapsed time in seconds.  (Why must I cast millis() here?)

      // Check for an outlier (presumed glitch/noise)
      /*if (fabs(force - last_force) > FORCE_SENTINEL)
      {
        if ( DEBUG == 2)
        {
          Serial.print("FORCE OUTLIER " + String(force) + " - IGNORING THIS VALUE.");
        }
        last_force = force;  // Remember the current value so we can compare to the next one
        force = fMean;
      }
      
      last_force = force; // Remember the current value so we can compare to the next one
      */
      allTimeSamples += 1;
      allTimeSum += force;
      fMean = allTimeSum / allTimeSamples;

      /* if( !(int(allTimeSamples) % 10) ){
        if( DEBUG == 2){
          Serial.println("Updating temperatures");
        }
      ds18b20.requestTemperatures();
      }
      degC = ds18b20.getTempCByIndex(0);
      degF = ds18b20.getTempFByIndex(0);
      //Check if reading was successful
      if( degC != DEVICE_DISCONNECTED_C)
      {
      // Report the current [time, force, temperature] value
      if( DEBUG == 2){
        Serial.print("Current time, force, temperature: ");
      }
      Serial.print(t);
      Serial.print(", ");
      Serial.print(force);
      Serial.print(", ");
      #ifdef T_IN_C
        Serial.println(degC);
      #elif defined T_IN_F
        Serial.println(degF);
      #else
        Serial.println("NaN");
      #endif
      }else{
        // Report the current [time, force] value
      if( DEBUG == 2){
        Serial.print("Current time, force: ");
      }
      Serial.print(t);
      Serial.print(", ");
      Serial.println(force);
      }
    }
    */

  //Temporary code until I get a version with working temp sensor:
  if( DEBUG == 2){
        Serial.print("Current time, force: ");
      }
      Serial.print(t);
      Serial.print(", ");
      Serial.println(force);
      }
      // Go to sleep for DATA_INTERVAL
    sleep_ms( DATA_INTERVAL);
  }
}