
#include <Arduino.h>
#include <EEPROM.h> // библиотека EEPROM

void Write()
{
    for (int it = 0; it <= 128; it++)
    {
        EEPROM.write(it, 0); // прочитали температуру
        EEPROM.commit();
    }
}
void Read()
{
    for (int i = 0; i <= 128; i++)
    {
        int io = EEPROM.read(i); // прочитали температуру

        Serial.println(String("meaning:") + i + ("  address: ") + io);
    }
}
void setup()
{
    Serial.begin(115200);
    EEPROM.begin(128); // активация функции EEPROM
    Write();
    Read();
}
void loop()
{
}