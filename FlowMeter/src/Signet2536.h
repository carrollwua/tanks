/* Header file for Signet2536 class
 * Monitors open-collector output from Signet 2536 flow meter and calculates
 * liquid flow speed.
 * Operation of the Signet 2536 is controlled by enable pin on the power booster
 */

#ifndef SIGNET_2536_H
#define SIGNET_2536_H

#define SECONDS_TO_SAMPLE 15
#define K_FACTOR_1_25_INCH 47.2
#define ENABLE_PIN 15
#define INPUT_PIN 16

class Signet2536
{
public:
  Signet2536(float kFactor);
  void startSampling();
  bool isSampleReady();
  float getSample();
  static void tickTimerCallback();
private:
  float kFactor;
  int pulses;
  static int ticks;
  enum SampleState {idle, sampling, calculating, dataReady};
  static SampleState state;
};

#endif
