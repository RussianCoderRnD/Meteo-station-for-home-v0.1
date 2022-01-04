
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

char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "itel A16 Plus_plus";
char pass[] = "Acer5560g!";

uint32_t eepromTimer = 0;   // EEPROM таймер
boolean eepromFlag = false; // EEPROM флаг = 0

byte degree1[8] = // кодируем символ градуса
    {
        B00111,
        B00101,
        B00111,
        B00000,
        B00000,
        B00000,
        B00000,
};
byte degree2[8] = // кодируем символ " стрелка вверх"
    {
        B00100,
        B01110,
        B10101,
        B00100,
        B00100,
        B00100,
        B00100,
};
byte degree3[8] = // кодируем символ " стрелка вниз"
    {
        B00100,
        B00100,
        B00100,
        B00100,
        B10101,
        B01110,
        B00100,
};
byte degree4[8] = // кодируем символ " стрелка в право"
    {
        B11000,
        B01100,
        B00110,
        B00011,
        B00110,
        B01100,
        B11000,
};
void setBright()
{
    eepromFlag = true;      // поднять флаг
    eepromTimer = millis(); // сбросить таймер
}
//!=========================== checkEEPROM ===============================================
void checkEEPROM()
{
    if (eepromFlag && (millis() - eepromTimer >= 1))
    {                          // если флаг поднят и с последнего нажатия прошло 10 секунд (10 000 мс)
        eepromFlag = false;    // опустили флаг
        EEPROM.write(0, t0);   // записали в EEPROM
        EEPROM.write(2, t1);   // записали в EEPROM
        EEPROM.write(4, t2);   // записали в EEPROM
        EEPROM.write(6, t3);   // записали в EEPROM
        EEPROM.write(8, t4);   // записали в EEPROM
        EEPROM.write(10, t5);  // записали в EEPROM
        EEPROM.write(12, t6);  // записали в EEPROM
        EEPROM.write(14, t7);  // записали в EEPROM
        EEPROM.write(16, t8);  // записали в EEPROM
        EEPROM.write(18, t9);  // записали в EEPROM
        EEPROM.write(20, t10); // записали в EEPROM
        EEPROM.write(22, t11); // записали в EEPROM
        EEPROM.write(24, t12); // записали в EEPROM
        EEPROM.write(26, t13); // записали в EEPROM
        EEPROM.write(28, t14); // записали в EEPROM
        EEPROM.write(30, t15); // записали в EEPROM
        EEPROM.write(32, t16); // записали в EEPROM
        EEPROM.write(34, t17); // записали в EEPROM
        EEPROM.write(36, t18); // записали в EEPROM
        EEPROM.write(38, t19); // записали в EEPROM
        EEPROM.write(40, t20); // записали в EEPROM
        EEPROM.write(42, t21); // записали в EEPROM
        EEPROM.write(44, t22); // записали в EEPROM
        EEPROM.write(46, t23); // записали в EEPROM
        EEPROM.commit();
        if (EEPROM.commit())
        {
            Serial.println("checkEEPROM successfully committed");
        }
        else
        {
            Serial.println("ERROR! checkEEPROM commit failed");
        }
        /*  Serial.println(String("checkEEPROM t0 ") + t0);
          Serial.println(String("checkEEPROM t1 ") + t1);
          Serial.println(String("checkEEPROM t2 ") + t2);
          Serial.println(String("checkEEPROM t3 ") + t3);
          Serial.println(String("checkEEPROM t4 ") + t4);
          Serial.println(String("checkEEPROM t5 ") + t5);
          Serial.println(String("checkEEPROM t6 ") + t6);
          Serial.println(String("checkEEPROM t7 ") + t7);
          Serial.println(String("checkEEPROM t8 ") + t8);
          Serial.println(String("checkEEPROM t9 ") + t9);
          Serial.println(String("checkEEPROM t10 ") + t10);
          Serial.println(String("checkEEPROM t11 ") + t11);
          Serial.println(String("checkEEPROM t12 ") + t12);
          Serial.println(String("checkEEPROM t13 ") + t13);
          Serial.println(String("checkEEPROM t14 ") + t14);
          Serial.println(String("checkEEPROM t15 ") + t15);
          Serial.println(String("checkEEPROM t16 ") + t16);
          Serial.println(String("checkEEPROM t17 ") + t17);
          Serial.println(String("checkEEPROM t18 ") + t18);
          Serial.println(String("checkEEPROM t19 ") + t19);
          Serial.println(String("checkEEPROM t20 ") + t20);
          Serial.println(String("checkEEPROM t21 ") + t21);
          Serial.println(String("checkEEPROM t22 ") + t22);
          Serial.println(String("checkEEPROM t23 ") + t23);
          delay(1000);
          */
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
        EEPROM.write(6, t3);               // записали в EEPROM
        EEPROM.write(8, t4);               // записали в EEPROM
        EEPROM.write(10, t5);              // записали в EEPROM
        EEPROM.write(12, t6);              // записали в EEPROM
        EEPROM.write(14, t7);              // записали в EEPROM
        EEPROM.write(16, t8);              // записали в EEPROM
        EEPROM.write(18, t9);              // записали в EEPROM
        EEPROM.write(20, t10);             // записали в EEPROM
        EEPROM.write(22, t11);             // записали в EEPROM
        EEPROM.write(24, t12);             // записали в EEPROM
        EEPROM.write(26, t13);             // записали в EEPROM
        EEPROM.write(28, t14);             // записали в EEPROM
        EEPROM.write(30, t15);             // записали в EEPROM
        EEPROM.write(32, t16);             // записали в EEPROM
        EEPROM.write(34, t17);             // записали в EEPROM
        EEPROM.write(36, t18);             // записали в EEPROM
        EEPROM.write(38, t19);             // записали в EEPROM
        EEPROM.write(40, t20);             // записали в EEPROM
        EEPROM.write(42, t21);             // записали в EEPROM
        EEPROM.write(44, t22);             // записали в EEPROM
        EEPROM.write(46, t23);             // записали в EEPROM
        EEPROM.commit();
        if (EEPROM.commit())
        {
            Serial.println("EEPROMRead successfully committed");
        }
        else
        {
            Serial.println("ERROR! EEPROMRead commit failed");
        }
    }
    t0 = EEPROM.read(0);   // прочитали температуру
    t1 = EEPROM.read(2);   // прочитали температуру
    t2 = EEPROM.read(4);   // прочитали температуру
    t3 = EEPROM.read(6);   // прочитали температуру
    t4 = EEPROM.read(8);   // прочитали температуру
    t5 = EEPROM.read(10);  // прочитали температуру
    t6 = EEPROM.read(12);  // прочитали температуру
    t7 = EEPROM.read(14);  // прочитали температуру
    t8 = EEPROM.read(16);  // прочитали температуру
    t9 = EEPROM.read(18);  // прочитали температуру
    t10 = EEPROM.read(20); // прочитали температуру
    t11 = EEPROM.read(22); // прочитали температуру
    t12 = EEPROM.read(24); // прочитали температуру
    t13 = EEPROM.read(26); // прочитали температуру
    t14 = EEPROM.read(28); // прочитали температуру
    t15 = EEPROM.read(30); // прочитали температуру
    t16 = EEPROM.read(32); // прочитали температуру
    t17 = EEPROM.read(34); // прочитали температуру
    t18 = EEPROM.read(36); // прочитали температуру
    t19 = EEPROM.read(38); // прочитали температуру
    t20 = EEPROM.read(40); // прочитали температуру
    t21 = EEPROM.read(42); // прочитали температуру
    t22 = EEPROM.read(44); // прочитали температуру
    t23 = EEPROM.read(46); // прочитали температуру
}
//!====================== функция анимации проверки датчика ==============================
void FOR_LOAD(void)
{
    for (int i = 0; i < 16; i++)
    {
        lcd.setCursor(i, 0); // Устанавливаем курсор в начало 2 строки
        lcd.print("\4");     // Выводим символ " > "
        delay(50);
    }
    for (int i = 0; i < 16; i++)
    {
        lcd.setCursor(i, 0); // Устанавливаем курсор в начало 2 строки
        lcd.print("  ");     // Затираем символ
        delay(50);
    }
    lcd.setCursor(0, 0);           // Устанавливаем курсор в начало 2 строки
    lcd.print("                "); // Затираем строку
}
//! =============== функция проверки датчиков на исправность =============================
void checkSensors()
{
    if (DS18B20_sensor.readTemp()) // ЕСЛИ датчик DS18B20 считался == TRUE
    {
        DS = true;
        Serial.println("DS18B20 OK...");
    }
    else
    {
        DS = false;
        Serial.println("DS18B20 ERROR!!");
    }
    float ntc;
    ntc = therm.getTempAverage();
    if (ntc <= 0.0) // ЕСЛИ датчик NTC считался == TRUE
    {
        NTC = false;
        Serial.println("NTC ERROR!!!    ");
    }
    else
    {
        NTC = true;
        Serial.println("NTC OK...       ");
    }
    if (isnan(dht.readHumidity())) // ЕСЛИ датчик считался TRUE
    {
        DHT = false;
        Serial.println("DHT11 ERROR!!! ");
    }
    else
    {
        DHT = true;
        Serial.println("DHT11 OK...");
    }
    if (bme.readTemperature()) // ЕСЛИ датчик считался TRUE
    {
        BMP = true;
        Serial.println("BMP280 OK...");
    }
    else
    {
        BMP = false;
        Serial.println("BMP280 ERROR!!! ");
    }
}
//! =============== функция проверки DS18B20 датчика на исправность ======================
void LCDPrintSensor()
{
    checkSensors();
    if (DS == true) // ЕСЛИ датчик DS18B20 считался == TRUE
    {
        lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
        lcd.print("DS18B20         "); // Выводим текст
        FOR_LOAD();
        lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
        lcd.print("DS18B20 OK...   "); // Выводим текст
        delay(2000);
    }
    else
    {
        lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
        lcd.print("DS18B20         "); // Выводим текст
        FOR_LOAD();
        lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
        lcd.print("DS18B20 ERROR!!!"); // Выводим текст
        delay(2000);
    }
    if (NTC == false) // ЕСЛИ датчик NTC считался == TRUE
    {
        lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
        lcd.print("NTC             "); // Выводим текст
        FOR_LOAD();
        lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
        lcd.print("NTC ERROR!!!    "); // Выводим текст
        delay(2000);
    }
    else
    {
        lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
        lcd.print("NTC             "); // Выводим текст
        FOR_LOAD();
        lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
        lcd.print("NTC OK...       "); // Выводим текст
        delay(2000);
    }
    if (DHT == false) // ЕСЛИ датчик считался TRUE
    {
        lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
        lcd.print("DHT11           "); // Выводим текст
        FOR_LOAD();
        lcd.setCursor(0, 1);                  // Устанавливаем курсор в начало 2 строки
        lcd.print(String("DHT11 ERROR!!! ")); // Выводим текст
        delay(2000);
    }
    else
    {
        lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
        lcd.print("DHT11           "); // Выводим текст
        FOR_LOAD();
        lcd.setCursor(0, 1);                   // Устанавливаем курсор в начало 2 строки
        lcd.print(String("DHT11 OK...     ")); // Выводим текст
        delay(2000);
    }
    if (BMP == true) // ЕСЛИ датчик считался TRUE
    {
        lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
        lcd.print("BMP280          "); // Выводим текст
        FOR_LOAD();
        lcd.setCursor(0, 1);                   // Устанавливаем курсор в начало 2 строки
        lcd.print(String("BMP280 OK...    ")); // Выводим текст
        delay(2000);
    }
    else
    {
        lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
        lcd.print("BMP280          "); // Выводим текст
        FOR_LOAD();
        lcd.setCursor(0, 1);                  // Устанавливаем курсор в начало 2 строки
        lcd.print(String("BMP280 ERROR!!!")); // Выводим текст
        delay(2000);
    }
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
    switch (hour)
    {
    case 0:
        t0 = DS18B20_sensor.getTemp();
        setBright(); //!
        break;
    case 1:
        t1 = DS18B20_sensor.getTemp();
        setBright(); //!
        break;
    case 2:
        t2 = DS18B20_sensor.getTemp();
        setBright(); //!
        break;
    case 3:
        t3 = DS18B20_sensor.getTemp();
        setBright(); //!
        break;
    case 4:
        t4 = DS18B20_sensor.getTemp();
        setBright(); //!
        break;
    case 5:
        t5 = DS18B20_sensor.getTemp();
        setBright(); //!
        break;
    case 6:
        t6 = DS18B20_sensor.getTemp();
        setBright(); //!
        break;
    case 7:
        t7 = DS18B20_sensor.getTemp();
        setBright(); //!
        break;
    case 8:
        t8 = DS18B20_sensor.getTemp();
        setBright(); //!
        break;
    case 9:
        t9 = DS18B20_sensor.getTemp();
        setBright(); //!
        break;
    case 10:
        t10 = DS18B20_sensor.getTemp();
        setBright(); //!
        break;
    case 11:
        t11 = DS18B20_sensor.getTemp();
        setBright(); //!
        break;
    case 12:
        t12 = DS18B20_sensor.getTemp();
        setBright(); //!
        break;
    case 13:
        t13 = DS18B20_sensor.getTemp();
        setBright(); //!
        break;
    case 14:
        t14 = DS18B20_sensor.getTemp();
        setBright(); //!
        break;
    case 15:
        t15 = DS18B20_sensor.getTemp();
        setBright(); //!
        break;
    case 16:
        t16 = DS18B20_sensor.getTemp();
        setBright(); //!
        break;
    case 17:
        t17 = DS18B20_sensor.getTemp();
        setBright(); //!
        break;
    case 18:
        t18 = DS18B20_sensor.getTemp();
        setBright(); //!
        break;
    case 19:
        t19 = DS18B20_sensor.getTemp();
        setBright(); //!
        break;
    case 20:
        t20 = DS18B20_sensor.getTemp();
        setBright(); //!
        break;
    case 21:
        t21 = DS18B20_sensor.getTemp();
        setBright(); //!
        break;
    case 22:
        t22 = DS18B20_sensor.getTemp();
        setBright(); //!
        break;
    case 23:
        t23 = DS18B20_sensor.getTemp();
        setBright(); //!
        break;
    }
}
//!============================= функция вывода времени на дисплей =======================
void timeLCD()
{
    lcd.setCursor(8, 1);         // Устанавливаем курсор в начало 2 строки
    lcd.print(rtc.getTimeStr()); // Выводим ВРЕМЯ на LCD дисплей
    delay(1000);
    lcd.setCursor(8, 1);         // Устанавливаем курсор в начало 2 строки
    lcd.print(rtc.getTimeStr()); // Выводим ВРЕМЯ на LCD дисплей
    delay(1000);
    lcd.setCursor(8, 1);         // Устанавливаем курсор в начало 2 строки
    lcd.print(rtc.getTimeStr()); // Выводим ВРЕМЯ на LCD дисплей
    delay(1000);
    lcd.setCursor(8, 1);         // Устанавливаем курсор в начало 2 строки
    lcd.print(rtc.getTimeStr()); // Выводим ВРЕМЯ на LCD дисплей
    delay(1000);
}
//!============================= функция первого экрана ==================================
void firstScreen()
{
    lcd.setCursor(0, 0);                       // Устанавливаем курсор в начало 1 строки
    lcd.print(String("\2") + val_max + "\1 "); // Выводим MAX значение температуры на LCD дисплей
    lcd.setCursor(0, 1);                       // Устанавливаем курсор в начало 2 строки
    lcd.print(String("\3") + val_min + "\1 "); // Выводим МИН значение температуры на LCD дисплей

    //!-----------------------------------------------------------------------------------
    if (BMP == false) // ЕСЛИ датчик считался TRUE
    {
        lcd.setCursor(8, 0);      // Устанавливаем курсор в начало 2 строки
        lcd.print("BMP280 ERR!"); // Выводим текст на LCD дисплей
    }
    else
    {
        lcd.setCursor(7, 0);                                           // Устанавливаем курсор в начало 2 строки
        lcd.print(String(" ") + BMP280_atmosphericPressure + "mm Hg"); // Выводим текст на LCD дисплей
    }
}
//!============================= функция второго экрана ==================================
void secondScreen()
{
    if (DS == false) // ЕСЛИ датчик считался TRUE
    {
        lcd.setCursor(0, 0);   // Устанавливаем курсор в начало 2 строки
        lcd.print("ERROR!!!"); // Выводим текст // lcd.print("ERR: DS18B20 IN!"); // Выводим текст
    }
    else
    {
        temperatureHome = DS18B20_sensor.getTemp();
        lcd.setCursor(0, 0);                              // Устанавливаем курсор в начало 2 строки
        lcd.print(String(" ") + temperatureHome + "\1 "); // Выводим текст на LCD дисплей
    }
    //!-----------------------------------------------------------------------------------
    if (NTC == false) // ЕСЛИ датчик считался TRUE
    {
        lcd.setCursor(0, 1);   // Устанавливаем курсор в начало 2 строки
        lcd.print("ERROR!!!"); // Выводим текст // lcd.print("ERR: DS18B20 IN!"); // Выводим текст
    }
    else
    {
        outdoorTemperature = therm.getTempAverage();
        lcd.setCursor(0, 1);                                 // Устанавливаем курсор в начало 2 строки
        lcd.print(String(" ") + outdoorTemperature + "\1 "); // Выводим текст на LCD дисплей
    }
    //!-----------------------------------------------------------------------------------
    if (DHT == false) // ЕСЛИ датчик считался TRUE
    {
        lcd.setCursor(8, 0);  // Устанавливаем курсор в начало 2 строки
        lcd.print("DHT ERR"); // Выводим текст на LCD дисплей
    }
    else
    {
        float DHT_hum = DHT_humidity;
        if (DHT_hum > 100.0)
        {
            DHT_humidity = 100.0;
        }
        Serial.println(String("Влжность ") + DHT_humidity + "%");
        lcd.setCursor(8, 0);                          // Устанавливаем курсор в начало 2 строки
        lcd.print(String(" ") + DHT_humidity + "% "); // Выводим текст на LCD дисплей
    }
}
//!===================== функция передачи данных в Blynk =================================
void variablesForBlynk()
{
    Blynk.virtualWrite(V0, DHT_humidity);               // передача значений в Blynk
    Blynk.virtualWrite(V1, temperatureHome);            // передача значений в Blynk
    Blynk.virtualWrite(V2, outdoorTemperature);         // передача значений в Blynk
    Blynk.virtualWrite(V3, BMP280_atmosphericPressure); // передача значений в Blynk
    Blynk.virtualWrite(V4, BMP280_temperature);         // передача значений в Blynk
    Blynk.virtualWrite(V5, DHT_temperature);            // передача значений в Blynk
    Blynk.virtualWrite(V6, val_min);                    // передача значений МИН значений температуры в Blynk
    Blynk.virtualWrite(V7, val_max);                    // передача значений МАХ значений температуры в Blynk
    Blynk.run();                                        // запуск передачи значений в Blynk
}
//!=========================== tEEPROM.read ==============================================
void tEEPROMRead()
{
    t0 = EEPROM.read(0);   // прочитали температуру
    t1 = EEPROM.read(2);   // прочитали температуру
    t2 = EEPROM.read(4);   // прочитали температуру
    t3 = EEPROM.read(6);   // прочитали температуру
    t4 = EEPROM.read(8);   // прочитали температуру
    t5 = EEPROM.read(10);  // прочитали температуру
    t6 = EEPROM.read(12);  // прочитали температуру
    t7 = EEPROM.read(14);  // прочитали температуру
    t8 = EEPROM.read(16);  // прочитали температуру
    t9 = EEPROM.read(18);  // прочитали температуру
    t10 = EEPROM.read(20); // прочитали температуру
    t11 = EEPROM.read(22); // прочитали температуру
    t12 = EEPROM.read(24); // прочитали температуру
    t13 = EEPROM.read(26); // прочитали температуру
    t14 = EEPROM.read(28); // прочитали температуру
    t15 = EEPROM.read(30); // прочитали температуру
    t16 = EEPROM.read(32); // прочитали температуру
    t17 = EEPROM.read(34); // прочитали температуру
    t18 = EEPROM.read(36); // прочитали температуру
    t19 = EEPROM.read(38); // прочитали температуру
    t20 = EEPROM.read(40); // прочитали температуру
    t21 = EEPROM.read(42); // прочитали температуру
    t22 = EEPROM.read(44); // прочитали температуру
    t23 = EEPROM.read(46); // прочитали температуру
    Serial.println(String("setup t0 ") + t0);
    Serial.println(String("setup t1 ") + t1);
    Serial.println(String("setup t2 ") + t2);
    Serial.println(String("setup t3 ") + t3);
    Serial.println(String("setup t4 ") + t4);
    Serial.println(String("setup t5 ") + t5);
    Serial.println(String("setup t6 ") + t6);
    Serial.println(String("setup t7 ") + t7);
    Serial.println(String("setup t8 ") + t8);
    Serial.println(String("setup t9 ") + t9);
    Serial.println(String("setup t10 ") + t10);
    Serial.println(String("setup t11 ") + t11);
    Serial.println(String("setup t12 ") + t12);
    Serial.println(String("setup t13 ") + t13);
    Serial.println(String("setup t14 ") + t14);
    Serial.println(String("setup t15 ") + t15);
    Serial.println(String("setup t16 ") + t16);
    Serial.println(String("setup t17 ") + t17);
    Serial.println(String("setup t18 ") + t18);
    Serial.println(String("setup t19 ") + t19);
    Serial.println(String("setup t20 ") + t20);
    Serial.println(String("setup t21 ") + t21);
    Serial.println(String("setup t22 ") + t22);
    Serial.println(String("setup t23 ") + t23);
}
//!=========================== void setup() ==============================================
void setup()
{
    Serial.begin(115200);
    EEPROMRead();     //!
    EEPROM.begin(64); // активация функции EEPROM
    tEEPROMRead();    //!
    bme.begin();      // инициализация BME  датчика
    dht.begin();      // инициализация DHT11  датчика
    lcd.init();       // инициализация LCD

    lcd.backlight(); // Включаем подсветку LCD дислея

    lcd.createChar(1, degree1); //  Загружаем 2 символ ГРАДУСА в ОЗУ дисплея
    lcd.createChar(2, degree2); //  Загружаем 3 символ СТРЕЛКА ВВЕРХ в ОЗУ дисплея
    lcd.createChar(3, degree3); //  Загружаем 4 символ СТРЕЛКА ВНИЗ в ОЗУ дисплея
    lcd.createChar(4, degree4); //  Загружаем 4 символ СТРЕЛКА В ПРАВО в ОЗУ дисплея

    lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
    lcd.print("Blynk started..."); // Выводим текст
    Blynk.begin(auth, ssid, pass); // подклчение к Blynk
    lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
    lcd.print("Blynk conect... "); // Выводим текст

    //! ------------ работа с модулем РЕАЛЬНОГО ВРЕМЕНИ DS1302 --------------------------------
    timeClient.begin();  // инициализация МОДУЛЯ РЕАЛЬНОГО ВРЕМЕНИ
    timeClient.update(); // обновление МОДУЛЯ РЕАЛЬНОГО ВРЕМЕНИ
    rtc.halt(false);
    rtc.writeProtect(false);
    hour = timeClient.getHours();   // получение значения ЧАСов из Blynk
    minu = timeClient.getMinutes(); // получение значения МИНут из Blynk
    sek = timeClient.getSeconds();  // получение значения СЕКУнд из Blynk
    // rtc.setTime(hour, minu, sek);   // запись в модуль РЕАЛЬНОГО ВРЕМЕНИ значения ЧАСА, МИНУТ, СЕКУНД

    //!------------------------ проверка датчинов на TRUE ------------------------------
    delay(2000);      //  задержка в 2 сек. перед проверкой датчиков на наличие и испрвность
    LCDPrintSensor(); //! функция опроса датчиков

    //!---------------------------------------------------------------------------------

    lcd.setCursor(0, 1);                   // Устанавливаем курсор в начало 2 строки
    lcd.print(String(".......GO.......")); // Выводим текст

    //!------------------------ затирание строк на дисплее------------------------------
    delay(2000);                   //  задержка в 2 сек. перед затиранием строк
    lcd.setCursor(0, 0);           // Устанавливаем курсор в начало 2 строки
    lcd.print("                "); // Выводим текст
    lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
    lcd.print("                "); // Выводим текст
}
//!=========================== void loop() ===============================================
void loop()
{
    checkEEPROM();                //! проверка EEPROM
    readingValuesSensors();       //!
    checkSensors();               //!
    hour = timeClient.getHours(); // считывание часа (0....23) для дальнейшего присвоения температуры
    hour_temp();                  //! функция присвоения значения тепмператры переменным t0...t23

    int vertualPinBlynk[] = {V8, V9, V10, V11, V12, V13, V14, V15, V16, V17, V18, V19, V20, V21, V22, V23, V24, V25, V26, V27, V28, V29, V30, V31};   // массив виртуальных пинов Blynk
    float variableTemperatureHour[] = {t0, t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t13, t14, t15, t16, t17, t18, t19, t20, t21, t22, t23}; // массив данных о температуре за каждый час

    secondScreen(); //! функция второго экрана
    timeLCD();      //!
    firstScreen();  //! функция первого экрана
    timeLCD();      //!

    static uint32_t tmr;
    if (millis() - tmr >= 300000) // обработка блока раз в 0 минут
    {
        for (int8_t q = 0; q < 24; q++)
        {
            Blynk.virtualWrite(vertualPinBlynk[q], variableTemperatureHour[q]); // передача в Blynk  почасовых переменных температуры
            Serial.println(String("vertualPinBlynk[q] = ") + vertualPinBlynk[q] + ("  variableTemperatureHour[q] = ") + variableTemperatureHour[q]);
        }
        for (int8_t i = 0; i < 24; i++) // цикл вычисления MAX и MIN значений температуры
        {
            val_max = max(variableTemperatureHour[i], val_max);
            val_min = min(variableTemperatureHour[i], val_min);
        }
        tmr = millis();
    }
    variablesForBlynk(); //! функция передачи данных в  Blynk
}
