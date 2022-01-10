
#include <Arduino.h>

void setup()
{
    Serial.begin(115200);
    for (int i = 0; i <= 24; i++)
        Serial.println(i);
}
void loop()
{
}