#include <Arduino.h>

const int PERIOD_TEN_SECONDS = 10000;
const int PERIOD_ONE_SECOND = 1000;
const int PERIOD_TWO_MS = 2;
const int BUFFER_SIZE = 2000;
const int BAUD_RATE = 115200;

unsigned long current_time = 0.0;
unsigned long elapsed_time_ten;
unsigned long elapsed_time_one;
unsigned long elapsed_time_twoms;

float buf[2000];

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(BAUD_RATE);
  while (!Serial)
  {
  }
  current_time = millis();
}

void executeAfter(int threshold_time, unsigned long *previous_time)
{
  if (current_time - *previous_time >= threshold_time)
  {
    Serial.print("printing after ");
    Serial.println(threshold_time);
    *previous_time = current_time;
  }
}

void loop()
{
  current_time = millis();

  executeAfter(PERIOD_ONE_SECOND, &elapsed_time_one);
  executeAfter(PERIOD_TEN_SECONDS, &elapsed_time_ten);

  // put your main code here, to run repeatedly:
  delay(100);
}