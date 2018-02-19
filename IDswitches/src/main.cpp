/* Stand-alone project to test CTS 220-series 10 position DIP switches for
 * setting radio node ID. This should be incorporated after PCBs are designed
 * so that other functions can be tested without having to wire switches in.
 *
 * The switches output the set value in binary on three data lines. To read from
 * two switches with only six pins the two switches will share data lines.
 * Because of the shared data lines and the fact that logic LOW pulls a pin to
 * ground, we'll have to read the switches as pull-up resistors and the lines
 * pulled to ground indicate a 1 in that binary place.
 *
 * Board pin                Function
 * ---------                --------
 * D12                      Common/enable pin for tens place switch
 * D11                      Common/enable pin for ones place switch
 * D10                      Ones place pin for both switches
 * D9                       Twos place pin for both switches
 * D6                       Fours place pin for both switches
 * D5                       Eights place pin for both switches
 */
#include <Arduino.h>

#define ONES_PLACE_COMMON_PIN 11
#define TENS_PLACE_COMMON_PIN 12
#define ID_SWITCH_ONES 10
#define ID_SWITCH_TWOS 9
#define ID_SWITCH_FOURS 6
#define ID_SWITCH_EIGHTS 5

unsigned int id;

void setup()
{
  unsigned int tens = 0;
  unsigned int ones = 0;
  id = 0;

  //Start serial
  Serial.begin(9600);
  //Set up the common pins
  pinMode(ONES_PLACE_COMMON_PIN, OUTPUT);
  pinMode(TENS_PLACE_COMMON_PIN, OUTPUT);
  pinMode(ID_SWITCH_ONES, INPUT_PULLUP);
  pinMode(ID_SWITCH_TWOS, INPUT_PULLUP);
  pinMode(ID_SWITCH_FOURS, INPUT_PULLUP);
  pinMode(ID_SWITCH_EIGHTS, INPUT_PULLUP);

  //Get the tens place
  digitalWrite(ONES_PLACE_COMMON_PIN, HIGH);
  digitalWrite(TENS_PLACE_COMMON_PIN, LOW);
  if (digitalRead(ID_SWITCH_ONES) == LOW)
  {
    tens |= 1;
  }
  if (digitalRead(ID_SWITCH_TWOS) == LOW)
  {
    tens |= (1 << 1);
  }
  if (digitalRead(ID_SWITCH_FOURS) == LOW)
  {
    tens |= (1 << 2);
  }
  if (digitalRead(ID_SWITCH_EIGHTS) == LOW)
  {
    tens |= (1 << 3);
  }

  //Get the ones
  digitalWrite(TENS_PLACE_COMMON_PIN, HIGH);
  digitalWrite(ONES_PLACE_COMMON_PIN, LOW);
  if (digitalRead(ID_SWITCH_ONES) == LOW)
  {
    ones |= 1;
  }
  if (digitalRead(ID_SWITCH_TWOS) == LOW)
  {
    ones |= (1 << 1);
  }
  if (digitalRead(ID_SWITCH_FOURS) == LOW)
  {
    ones |= (1 << 2);
  }
  if (digitalRead(ID_SWITCH_EIGHTS) == LOW)
  {
    ones |= (1 << 3);
  }

  digitalWrite(TENS_PLACE_COMMON_PIN, LOW);

  id = ones + (10 * tens);

}

void loop()
{
  Serial.print("ID: ");
  Serial.println(id);
  delay(1000);
}
