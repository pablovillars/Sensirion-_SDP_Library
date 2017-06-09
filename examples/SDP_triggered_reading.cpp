#include <Arduino.h>
#include "sdplib.h"
#include "Wire.h"

SDPclass sdp;

float diff_pressure, temperature;
uint32_t prevMillis = 0;

void setup(){

  while(!Serial){
  Serial.begin(115200);
}
  
  Wire.begin();
  sdp.begin();
  sdp.startContinuousMeasurement(SDP_TEMPCOMP_DIFFERENTIAL_PRESSURE, SDP_AVERAGING_TILL_READ);
}

void loop(){
  // reading at 100Hz
  if((millis() - prevMillis) > 10){
    // Trigger measurement with differential pressure temperature compensation
    // and without clock stretching
    diff_pressure = sdp.getDiffPressureTrigger(SDP_TEMPCOMP_DIFFERENTIAL_PRESSURE, SDP_CLKST_NONE);
    // Once we have triggered one differential pressure measurement, we can get with temperature.
    // We can also trigger the temperature measurement directly with sdp.getTemperatureTrigger(tempcomp, clkst);
    temperature = sdp.getTemperature();
  }

}
