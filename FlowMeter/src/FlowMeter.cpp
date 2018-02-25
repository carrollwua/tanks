/* Flow meter main source file
 * University of Arizona ENG 498 A&B Team 17023
 * Team TANKS
 *
 * Main file for firmware for the Flow Monitor subsystem
 * Designed for the Adafruit Feather M0 LoRa
 * Monitors and transmits data from a Signet 2536 padlewheel flow meter
 * Flow meter is powered from an Adafruit Verter Buck/Boost controller
 * Requests and receives data from the Camera Subsystem via NRF24L01+ ISM radio
 */

#include <arduino.h>
#include "RF24.h"

/* Pins are as follows:
 * Board Pin#     Description
 * A1/D15         Voltage booster enable
 * A2/D16         Flow meter signal
 * A4/D18         ISM radio enable
 * A5/D19         ISM radio chip select
 * SCK/MOSI/MISO  SPI pins for ISM radio
 * More as components are included
 */

#define METER_ENABLE_PIN 15
#define METER_INPUT_PIN 16
#define ISM_ENABLE 18
#define ISM_CS 19

//Constant definitions
#define SAMPLE_FREQUENCY 1UL           //in minutes
#define SAMPLE_DURATION 15UL           //in seconds
#define ULONG_MAX 4294967295           //2^32 - 1
#define K_FACTOR_1_25_INCH 47.2        //Pulses per liter
#define ISM_BUFFER_MAX 32              //Max radio size
#define ISM_TIMEOUT_MS 5000           //ms to wait for cow presence data

//Function prototype(s)
void meterPinISR();                    //Interrupt for meter signal

//global objects
RF24 ism(ISM_ENABLE, ISM_CS);

//global variables
volatile unsigned long pulses;         //Count of pulses during each sample
unsigned long currentTick;
unsigned long msDifference;
unsigned long sampleStart;
unsigned long lastSample;
unsigned long lastIRreq;
float flowRate;
byte ismAddr[][6] = {{'F','L','O','W','M','T'},{'C','A','M','E','R','A'}};
char ismBuf[ISM_BUFFER_MAX];
char presenceReq = 'C';

/* States for FSM type operations, allowing for multiple simultaneous
 * functions (mostly) without blocking flow.
 * State            Function
 * -----            --------
 * idle             Wait for the next sample period and listen for radio packets
 * beginSampling    Enable the voltage booster and start sampling pulses
 * sampling         Continue sampling pulses until time has elapsed
 * stopSampling     Disable the voltage booster
 * calculateResults Use the k-value from the datasheet to calculate flow volume
 * requestIR        Send a request packet to the Camera
 * waitForIR        Wait for response from the Camera
 * makePacket       Upon response received from the Camera encode a mesh packet
 * transmitPacket   Push the packet to the mesh network and return to idle
 */
enum FlowMeterState { meterIdle, beginSampling, sampling, stopSampling,
                      requestIR, waitForIR//, makePacket,
                      //transmitPacket};
                    };
volatile FlowMeterState meterState = meterIdle;
/* States for LoRa radio FSM
 * State            Function
 * -----            --------
 * idle             Do nothing until a packet is received from either the
 *                    network or the main FSM
 * packetReceived   Receive a packet from the network, evaluate whether it needs
 *                    needs to be retransmitted
 * appendPathData   If necessary append this unit's ID and stage for retransmit
 * transmit         Transmit a packet ready to transmit
 */
//enum MeshRadioState { mrIdle, packetReceived, appendPathData, transmit};


void setup()
{
  //Take serial out later
  Serial.begin(9600);

  //Setup functions
  //Start hardware
  //start the ISM radio
  ism.begin();
  ism.setPALevel(RF24_PA_LOW);
  ism.setRetries(15,5);
  ism.setAutoAck(true);
  ism.setPayloadSize(1);
  ism.openWritingPipe(ismAddr[1]);
  ism.openReadingPipe(0,ismAddr[0]);
  for (int i = 0; i < ISM_BUFFER_MAX; i++)
  {//Initialize the buffer
    ismBuf[i]=0;
  }

  //Initialize variables
  currentTick = millis();
  msDifference = 0;
  sampleStart = 0;
  lastSample = 0;
  flowRate = 0;
  pulses = 0;
  lastIRreq=0;

  //Set up pins
  pinMode(METER_ENABLE_PIN, OUTPUT);
  pinMode(METER_INPUT_PIN, INPUT_PULLUP);
  //Attach ISR for meter pin
  attachInterrupt(digitalPinToInterrupt(METER_INPUT_PIN), meterPinISR, FALLING);
}

void loop()
{
  //Store current timestamp
  currentTick = millis();
  if (currentTick < lastSample)
  {//account for overflow
    msDifference = currentTick + (ULONG_MAX - lastSample);
  }
  else
  {
    msDifference = currentTick - lastSample;
  }
  //Check for millis() overflow, should happen once every 50 days or so
  switch (meterState)
  {
    case meterIdle:
      if (msDifference >= SAMPLE_FREQUENCY * 60 * 1000)
      {
        meterState = beginSampling;
      }
    break;
    case beginSampling:
      Serial.println("Beginning a sample...");
      lastSample = currentTick;
      sampleStart = currentTick;            //store start time
      pulses = 0;                           //reset the pulse count
      flowRate = 0;
      digitalWrite(METER_ENABLE_PIN, HIGH); //Enable the voltage boost
      meterState = sampling;
    break;
    case sampling:
      if (currentTick - sampleStart > SAMPLE_DURATION * 1000)
      {
        meterState = stopSampling;
      }
    break;
    case stopSampling:
      digitalWrite(METER_ENABLE_PIN, LOW);  //Disable voltage boost
      flowRate = ((pulses / K_FACTOR_1_25_INCH) / SAMPLE_DURATION);
      Serial.print("Sample rate: ");
      Serial.print(flowRate);
      Serial.println(" l/s.");
      meterState = requestIR;
    break;
    case requestIR:
      ism.stopListening();  //probably unnecessary
      delay(10);
      Serial.println("Requesting cow presence");
      if (ism.write(&presenceReq,ism.getPayloadSize()))
      {
        Serial.println("Request sent.");
        ism.startListening();
        delay(10);
        meterState = waitForIR;
        lastIRreq=currentTick;
      }
      else
      {
        Serial.print("Receiver did not acknowledge");
        meterState = meterIdle;
      }
    break;
    case waitForIR:
      if (ism.available())
        {
          Serial.println("Reading response...");
          ism.read(ismBuf, ISM_BUFFER_MAX);
          Serial.print("Received: ");
          Serial.println(ismBuf);
          ism.stopListening();
          meterState = meterIdle;
        }
      else if (currentTick - lastIRreq > ISM_TIMEOUT_MS)
      {
        Serial.print("IR request timed out.");
        ism.stopListening();
        meterState = meterIdle;
      }
    break;
  }

  delay(10);
}

//ISR for falling signal on meter input pin. Increments pulse count.
void meterPinISR()
{
  if (meterState == sampling)
  {
    pulses++;
  }
}
