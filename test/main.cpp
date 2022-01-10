
//! ============== Blynk block =================================

#define BLYNK_TEMPLATE_ID "TMPLEAFuc2FF"                    // для коректной работы данные необходимо внести актуальные
#define BLYNK_DEVICE_NAME "ESP Meteo"                       // для коректной работы данные необходимо внести актуальные
#define BLYNK_AUTH_TOKEN "X6a0vuSe3wdQmPrk_vKY898fChF5LEIx" // для коректной работы данные необходимо внести актуальные

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

//! =============== setPin block ================================
#define DHTPIN 2       //ПИН D4 DHT термодатчика 2
#define DHTTYPE DHT11  //тип DHT термодатчика
#define DS_PIN 16      // ПИН D0 термо датчика DS18B20
#define NTC_PIN A0     // ПИН термо датчика NTC A0
#define INIT_ADDR 1023 // номер резервной ячейки. EEPROM
#define INIT_KEY 50    // ключ первого запуска. 0-254, на выбор. EEPROM

const long utcOffsetInSeconds = 10800; // коректировка часового пояса +3

uint8_t s1_addr[] = {0x28, 0x28, 0xF, 0x4, 0x0, 0x0, 0x0, 0x68}; // Адресс термодатчика DS18B20

DHT dht(DHTPIN, DHTTYPE);                     // Создаём DHT термодатчик
LiquidCrystal_I2C lcd(0x27, 16, 2);           // set the LCD address to 0x27 for a 16 chars and 2 line display
MicroDS18B20<DS_PIN, s1_addr> DS18B20_sensor; // Создаем термометр с адресацией
DS1302 rtc(0, 12, 14);                        //Создаём МОДУЛЬ РЕАЛЬНОГО ВРЕМЕНИ
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds); // Сайт этолонного времени
GyverBME280 bme;                                                  // Создаём датчик BME280
GyverNTC therm(NTC_PIN, 10500, 3950);                             // пин, сопротивление при 25 градусах (R термистора = R резистора!), бета-коэффициент
                                                                  // GyverNTC therm(0, 10000, 3435, 25, 10000);	// пин, R термистора, B термистора, базовая температура, R резистора
                                                                  // серый 4300
                                                                  // проводной 3950

float DHT_humidity;                                          // variable for humidity with DHT
float DHT_temperature;                                       // variable for temperature with DHT
float temperatureHome;                                       // variable for temperature in home DS18B20
float outdoorTemperature;                                    // variable for temperature outdoor DS18B20
float BMP280_temperature;                                    // variable for temperature with BMP280
float t0, t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12; // переменные для значения температуры на каждый час
float t13, t14, t15, t16, t17, t18, t19, t20, t21, t22, t23; // переменные для значения температуры на каждый час
float val_max = 0;                                           // опорное значения для функции max
float val_min;                                               // опорное значения для функции min
int BMP280_atmosphericPressure;                              // переменная атмосферного давления с датчика BMP280
int hour;                                                    // переменная ЧАСЫ
int minu;                                                    // переменная МИНУТЫ
int sek;                                                     // переменная СЕКУНДЫ
bool DS, NTC, DHT, BMP;

uint32_t eepromTimer = 0;   // EEPROM таймер
boolean eepromFlag = false; // EEPROM флаг = 0

void setBright()
{
    eepromFlag = true;      // поднять флаг
    eepromTimer = millis(); // сбросить таймер
    Serial.println(String("setBright eepromFlag") + eepromFlag);
    Serial.println(String("setBright ") + t0);
}
//!=========================== checkEEPROM ===============================================
void checkEEPROM()
{
    Serial.println("checkEEPROM");
    if (eepromFlag)
    {
        Serial.println(String("checkEEPROM TRUE t0 ") + t0);
        Serial.println(String("checkEEPROM TRUE t1 ") + t1);
        Serial.println(String("checkEEPROM TRUE t2 ") + t2);
        eepromFlag = false;  // опустили флаг
        EEPROM.write(0, t0); // записали в EEPROM
        EEPROM.write(2, t1); // записали в EEPROM
        EEPROM.write(4, t2); // записали в EEPROM
        EEPROM.commit();
    }
    else
    {
        Serial.println(String("checkEEPROM  FOLSE t0 ") + t0);
        Serial.println(String("checkEEPROM FOLSE t1 ") + t1);
        Serial.println(String("checkEEPROM FOLSE t2 ") + t2);
    }
}
//!=========================== EEPROMRead ================================================
void EEPROMRead()
{
    if (EEPROM.read(INIT_ADDR) != INIT_KEY)
    {                                      // первый запуск (ЕСЛИ INIT_ADDR (1023)не равен INIT_KEY (50) то записать EEPROM.write(INIT_ADDR, INIT_KEY);EEPROM.put(0, izmenenieTemp);
        EEPROM.write(INIT_ADDR, INIT_KEY); // записали ключ
        EEPROM.write(0, t0);               // записали в EEPROM
        EEPROM.write(2, t1);               // записали в EEPROM
        EEPROM.write(4, t2);               // записали в EEPROM
        Serial.println("EEPROMRead successfully committed");
        EEPROM.commit();
    }
    t0 = EEPROM.read(0); // прочитали температуру
    t1 = EEPROM.read(2); // прочитали температуру
    t2 = EEPROM.read(4); // прочитали температуру
    Serial.println("EEPROMRead read memory values ");
}

//!================== функция считываний значений с датчиков =============================
void readingValuesSensors()
{
    DS18B20_sensor.requestTemp();                                    // считывание ТЕМПЕРАТУРЫ с DS18B20 датчика №1
    DHT_humidity = dht.readHumidity();                               // считывание ВЛАЖНОСТИ с DHT
    DHT_temperature = dht.readTemperature();                         // считывание ТЕМПЕРАТУРЫ с DHT
    BMP280_temperature = bme.readTemperature();                      // считывание ТЕМПЕРАТУРЫ с BMP280
    temperatureHome = DS18B20_sensor.getTemp();                      // преобразование значений с термодатчика №1
    outdoorTemperature = therm.getTempAverage();                     // преобразование значений с термодатчика №1
    BMP280_atmosphericPressure = pressureToMmHg(bme.readPressure()); // считывание и преобразование ДАВЛЕНИЯ с BMP280
}
//!======= функция присвоения температуры переменным на каждый час =======================
void hour_temp()
{
    Serial.println(String("hour_temp  0  ") + t0);
    switch (hour)
    {
    case 0:
        t0 = DS18B20_sensor.getTemp();
        setBright();
        Serial.println(String("hour_temp  0  ") + t0); //!
        break;
    case 1:
        t1 = DS18B20_sensor.getTemp();
        setBright();
        Serial.println(String("hour_temp 1  ") + t1); //!
        break;
    case 2:
        t2 = DS18B20_sensor.getTemp();
        setBright();
        Serial.println(String("hour_temp 2  ") + t2); //!
        break;
    case 3:
        t3 = DS18B20_sensor.getTemp();
        setBright();
        Serial.println(String("hour_temp 3  ") + t3); //!//!
        break;
    case 4:
        t4 = DS18B20_sensor.getTemp();
        setBright();
        Serial.println(String("hour_temp 4  ") + t4); //!//!
        break;
    case 5:
        t5 = DS18B20_sensor.getTemp();
        setBright();
        Serial.println(String("hour_temp 5  ") + t5); //!//!
        break;
    case 6:
        t6 = DS18B20_sensor.getTemp();
        setBright();
        Serial.println(String("hour_temp 6  ") + t6); //!//!
        break;
    case 7:
        t7 = DS18B20_sensor.getTemp();
        setBright();
        Serial.println(String("hour_temp 7  ") + t7); //!//!
        break;
    case 8:
        t8 = DS18B20_sensor.getTemp();
        setBright();
        Serial.println(String("hour_temp 8  ") + t8); //!//!
        break;
    case 9:
        t9 = DS18B20_sensor.getTemp();
        setBright();
        Serial.println(String("hour_temp 9  ") + t9); //!//!
        break;
    case 10:
        t10 = DS18B20_sensor.getTemp();
        setBright();
        Serial.println(String("hour_temp 10  ") + t10); //!//!
        break;
    case 11:
        t11 = DS18B20_sensor.getTemp();
        setBright();
        Serial.println(String("hour_temp 11  ") + t11); //!//!
        break;
    case 12:
        t12 = DS18B20_sensor.getTemp();
        setBright();
        Serial.println(String("hour_temp 12  ") + t12); //!//!
        break;
    case 13:
        t13 = DS18B20_sensor.getTemp();
        setBright();
        Serial.println(String("hour_temp 13  ") + t13); //!//!
        break;
    case 14:
        t14 = DS18B20_sensor.getTemp();
        setBright();
        Serial.println(String("hour_temp 14  ") + t14); //!//!
        break;
    case 15:
        t15 = DS18B20_sensor.getTemp();
        setBright();
        Serial.println(String("hour_temp 15  ") + t15); //!//!
        break;
    case 16:
        t16 = DS18B20_sensor.getTemp();
        setBright();
        Serial.println(String("hour_temp 16  ") + t16); //!//!
        break;
    case 17:
        t17 = DS18B20_sensor.getTemp();
        setBright();
        Serial.println(String("hour_temp 17  ") + t17); //!//!
        break;
    case 18:
        t18 = DS18B20_sensor.getTemp();
        setBright();
        Serial.println(String("hour_temp 18  ") + t18); //!//!
        break;
    case 19:
        t19 = DS18B20_sensor.getTemp();
        setBright();
        Serial.println(String("hour_temp 19  ") + t19); //!//!
        break;
    case 20:
        t20 = DS18B20_sensor.getTemp();
        setBright();
        Serial.println(String("hour_temp 20  ") + t20); //!//!
        break;
    case 21:
        t21 = DS18B20_sensor.getTemp();
        setBright();
        Serial.println(String("hour_temp 21  ") + t21); //!//!
        break;
    case 22:
        t22 = DS18B20_sensor.getTemp();
        setBright();
        Serial.println(String("hour_temp 22  ") + t22); //!//!
        break;
    case 23:
        t23 = DS18B20_sensor.getTemp();
        setBright();
        Serial.println(String("hour_temp 23  ") + t23); //!
        break;
    }
}

//!=========================== tEEPROM.read ==============================================
void tEEPROMRead()
{
    t0 = EEPROM.read(0); // прочитали температуру
    t1 = EEPROM.read(2); // прочитали температуру
    t2 = EEPROM.read(4); // прочитали температуру

    Serial.println(String("setup t0 ") + t0);
    Serial.println(String("setup t1 ") + t1);
    Serial.println(String("setup t2 ") + t2);
}
//!=========================== void setup() ==============================================
void setup()
{
    Serial.begin(115200);
    EEPROM.begin(128); // активация функции EEPROM
                       //   EEPROMRead();      //!
    tEEPROMRead();     //!

    //! ------------ работа с модулем РЕАЛЬНОГО ВРЕМЕНИ DS1302 --------------------------------

    rtc.halt(false);
    rtc.writeProtect(false);
    hour = timeClient.getHours();   // получение значения ЧАСов из Blynk
    minu = timeClient.getMinutes(); // получение значения МИНут из Blynk
    sek = timeClient.getSeconds();  // получение значения СЕКУнд из Blynk
    // rtc.setTime(hour, minu, sek);   // запись в модуль РЕАЛЬНОГО ВРЕМЕНИ значения ЧАСА, МИНУТ, СЕКУНД
}
//!=========================== void loop() ===============================================
void loop()
{
    readingValuesSensors(); //!

    //  checkSensors();                 //!
    hour = timeClient.getMinutes(); // считывание часа (0....23) для дальнейшего присвоения температуры
    switch (hour)
    {
    case 0:
        t0 = DS18B20_sensor.getTemp();
        EEPROM.write(0, t0);
        EEPROM.commit();                               // записали в EEPROM
        Serial.println(String("hour_temp  0  ") + t0); //!
        break;
    case 1:
        t1 = DS18B20_sensor.getTemp();
        EEPROM.write(2, t1);
        EEPROM.commit();                              // записали в EEPROM
        Serial.println(String("hour_temp 1  ") + t1); //!
        break;
    case 2:
        t2 = DS18B20_sensor.getTemp();
        EEPROM.write(4, t2);                          // записали в EEPROM
        EEPROM.commit();                              // записали в EEPROM
        Serial.println(String("hour_temp 2  ") + t2); //!
        break;
    }
    //  variablesForBlynk(); //! функция передачи данных в  Blynk
    tEEPROMRead();
    delay(1000);
}
