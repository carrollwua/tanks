/* Source file for Signet2536 class
 */

#include "Signet2536.h"
#include <arduino.h>
#include <Adafruit_ASFcore.h>

Signet2536::Signet2536(float kFactor)
{
  //Set local values
  this->kFactor = kFactor;

  //Set up the pins
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(INPUT_PIN, INPUT);
}

//Set up the system to start sampling
void Signet2536::startSampling()
{
  if (this->state != idle)
  {
    this->pulses = 0;
    this->ticks = 0;
    //Turn on the voltage booster
  }
}

//Getter
bool Signet2536::isSampleReady()
{
 return (this->state == dataReady);
}

//If a sample is ready, calculate the flow rate based on the number of ticks
//and return it. If data is not ready, return -1
float Signet2536::getSample()
{
  if (this->state != dataReady)
  {
    return -1;
  }
  this->state = idle;
  return this->pulses/this->kFactor;
}

//Need a 1 second timer
static void Signet2536::tickTimerCallback()
{
  ticks++;
  if (ticks == SECONDS_TO_SAMPLE)
  {
    state = dataReady;
    tc_disable(&TCC0);
  }
}
//Need a pin change interrupt
