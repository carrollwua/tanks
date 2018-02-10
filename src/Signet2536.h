/* Header file for Signet2536 class
 * Monitors open-collector output from Signet 2536 flow meter and calculates
 * liquid flow speed.
 * Operation of the Signet 2536 is controlled by enable pin on the power booster
 */

#ifndef SIGNET_2536_H
#define SIGNET_2536_H

class Signet2536
{
private:
  enum SampleState {idle, sampling, calculating, data ready}
public:
  start(int enablePin, int signalPin);

};

#endif
