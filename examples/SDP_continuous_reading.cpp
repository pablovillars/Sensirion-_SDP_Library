#include <Arduino.h>
#include "sdplib.h"
#include "Wire.h"

SDPclass sdp;

double diff_pressure, temperature;
uint32_t prevMillis = 0;

void setup(){

  while(!Serial){
  Serial.begin(115200);
}
  SDP_timer.begin(SDP_isr,250000); // 1000Hz
  Wire.begin();
  sdp.begin();
  sdp.getAddress();
  sdp.startContinuousMeasurement(SDP_TEMPCOMP_DIFFERENTIAL_PRESSURE, SDP_AVERAGING_TILL_READ);
}

void loop(){
  // reading at 100Hz
  if((millis() - prevMillis) > 10){
    diff_pressure = sdp.getDiffPressure();
    temperature = sdp.getTemperature();
  }
}
