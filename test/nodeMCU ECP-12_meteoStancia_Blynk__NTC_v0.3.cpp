
//! ============= liberse block ================================
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <BlynkSimpleEsp8266.h> // библиотека Blynk
#include <LiquidCrystal_I2C.h>  // библиотека LCD 1602 дисплея
#include <DS1302.h>             // библиотека модуля РЕАЛЬНОГО ВРМЕНИ
#include <GyverBME280.h>        // библиотека термодатчика BME280
#include <DHT.h>                // библиотека DHT датчика
#include <GyverNTC.h>           // библиотека термодатчика NTC
#include <microDS18B20.h>       // библиотека термодатчика DS18B20
#include <EEPROM.h>             // библиотека EEPROM

#define INIT_ADDR 1023 // номер резервной ячейки. EEPROM
#define INIT_KEY 50    // ключ первого запуска. 0-254, на выбор. EEPROM

float t0, t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12; // переменные для значения температуры на каждый час
float t13, t14, t15, t16, t17, t18, t19, t20, t21, t22, t23; // переменные для значения температуры на каждый час
int hour;

uint32_t eepromTimer = 0;   // EEPROM таймер
boolean eepromFlag = false; // EEPROM флаг = 0

//!=======================setBright===================================
void setBright()
{
    eepromFlag = true;      // поднять флаг
    eepromTimer = millis(); // сбросить таймер
}

//!=======================checkEEPROM===================================
void checkEEPROM()
{
    if (eepromFlag && (millis() - eepromTimer >= 1000)) // если флаг поднят и с последнего нажатия прошло 10 секунд (10 000 мс)
    {
        eepromFlag = false;  // опустили флаг
        EEPROM.write(0, t0); // записали в EEPROM
        EEPROM.commit();

        Serial.println(String("записали в EEPROM t0 ") + t0);
    }
}
//!=======================EEPROMRead===================================
void EEPROMRead()
{
    if (EEPROM.read(INIT_ADDR) != INIT_KEY)
    {                                      // первый запуск (ЕСЛИ INIT_ADDR (1023)не равен INIT_KEY (50) то записать EEPROM.write(INIT_ADDR, INIT_KEY);EEPROM.put(0, izmenenieTemp);
        EEPROM.write(INIT_ADDR, INIT_KEY); // записали ключ
        EEPROM.write(0, t0);               // записали стандартное значение температуры. в данном случае это значение переменной, объявленное выше
        EEPROM.commit();
    }
    t0 = EEPROM.read(0); // прочитали температуру
    Serial.println(String("прочитали EEPROM t0 ") + t0);
}
//!=========================== void setup() ==============================================
void setup()
{
    Serial.begin(115200);
    EEPROMRead();
    EEPROM.begin(10);    // активация функции EEPROM
    t0 = EEPROM.read(0); //считывание из EEPROM значений
    Serial.println(String("setup t0 ") + t0);
}

//!=======================================================================================
void loop()
{
    checkEEPROM(); // проверка EEPROM

    int radomais;
    radomais = random(1, 10);
    Serial.println(String("rand ") + radomais);
    delay(3000);
    if (radomais == 1)
    {
        t0 = 1;
        setBright();
        Serial.println(String("t0 ") + t0);
    }
}
