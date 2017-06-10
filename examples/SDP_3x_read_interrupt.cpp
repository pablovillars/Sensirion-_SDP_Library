#include <Arduino.h>
#include "sdplib.h"
#include "Wire.h"

#define IRQ_PIN 4

SDPclass sdp;

double diff_pressure, temperature;
bool sdpFlag = false;

// Interrupt routine
void SDP_isr(void){
  sdpFlag = true;
}

void setup(){

  while(!Serial){
  Serial.begin(115200);
}
  pinMode(IRQ_PIN, INPUT_PULLUP);
  attachInterrupt(IRQ_PIN, SDP_isr, FALLING); // active when low, so we seek for change to low

  Wire.begin();
  sdp.begin();
  sdp.getAddress();
  sdp.startContinuousMeasurement(SDP_TEMPCOMP_DIFFERENTIAL_PRESSURE, SDP_AVERAGING_TILL_READ);
}

void loop(){

  if(sdpFlag == true){
    diff_pressure = sdp.getDiffPressure();
    temperature = sdp.getTemperature();
    sdpFlag = false;
  }

}
