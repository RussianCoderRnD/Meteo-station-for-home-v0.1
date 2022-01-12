
#include <Arduino.h>
#include <EEPROM.h> // библиотека EEPROM

float t0, t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12; // переменные для значения температуры на каждый час
float t13, t14, t15, t16, t17, t18, t19, t20, t21, t22, t23; // переменные для значения температуры на каждый час
float ii[] = {t0, t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t13, t14, t15, t16, t17, t18, t19, t20, t21, t22, t23};
int aa[] = {0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46};

void Write()
{
    for (int it = 0; it <= 23; it++)
    {
        EEPROM.write(aa[it], 0); // прочитали температуру
        EEPROM.commit();
    }
}
void Read()
{
    for (int i = 0; i <= 23; i++)
    {
        ii[i] = EEPROM.read(aa[i]); // прочитали температуру

        Serial.println(String("t") + i + (" ") + ii[i] + ("  address: ") + aa[i]);
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