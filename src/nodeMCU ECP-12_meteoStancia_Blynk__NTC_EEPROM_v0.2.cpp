
//! ============== Blynk block =================================

#define BLYNK_TEMPLATE_ID "TMPLEAFuc2FFЮ"                    // для коректной работы данные необходимо внести актуальные
#define BLYNK_DEVICE_NAME "ESP MeteoЮ"                       // для коректной работы данные необходимо внести актуальные
#define BLYNK_AUTH_TOKEN "X6a0vuSe3wdQmPrk_vKY898fChF5LEIxЮ" // для коректной работы данные необходимо внести актуальные

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
#define DHTPIN 2       //ПИН D4 DHT термодатчика
#define DHTTYPE DHT11  //тип DHT термодатчика
#define DS_PIN 16      // ПИН D0 термо датчика DS18B20
#define NTC_PIN A0     // ПИН термо датчика NTC
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
int delayScreenOne = 10;                                     // задержка отображения информации на экране №1
int delayScreenTwo = 20;                                     // задержка отображения информации на экране №2
bool flag = 1;
int screenOne, screenTwo;
bool ntc, BME_status, DHT_status, NTC_status, DS18B20_status;
uint32_t eepromTimer = 0;   // EEPROM таймер
boolean eepromFlag = false; // EEPROM флаг = 0

char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "itel A16 Plus_plus";
char pass[] = "Acer5560g!";

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
//!======================= setBright =====================================================
void setBright()
{
    eepromFlag = true;      // поднять флаг
    eepromTimer = millis(); // сбросить таймер
}
//!======================= checkEEPROM ===================================================
void checkEEPROM()
{

    if (eepromFlag && (millis() - eepromTimer >= 1)) // если флаг поднят и с последнего нажатия прошло 1 мс
    {
        Serial.println();
        Serial.println("--- ПРОИЗВЕЛАСЬ ЗАПИСЬ ЗНАЧЕНИЙ В EEPROM ---");
        Serial.println();
        eepromFlag = false;   // опустили флаг
        EEPROM.put(0, t0);    // записали в EEPROM
        EEPROM.put(5, t1);    // записали в EEPROM
        EEPROM.put(10, t2);   // записали в EEPROM
        EEPROM.put(15, t3);   // записали в EEPROM
        EEPROM.put(20, t4);   // записали в EEPROM
        EEPROM.put(25, t5);   // записали в EEPROM
        EEPROM.put(30, t6);   // записали в EEPROM
        EEPROM.put(35, t7);   // записали в EEPROM
        EEPROM.put(40, t8);   // записали в EEPROM
        EEPROM.put(45, t9);   // записали в EEPROM
        EEPROM.put(50, t10);  // записали в EEPROM
        EEPROM.put(55, t11);  // записали в EEPROM
        EEPROM.put(60, t12);  // записали в EEPROM
        EEPROM.put(65, t13);  // записали в EEPROM
        EEPROM.put(70, t14);  // записали в EEPROM
        EEPROM.put(75, t15);  // записали в EEPROM
        EEPROM.put(80, t16);  // записали в EEPROM
        EEPROM.put(85, t17);  // записали в EEPROM
        EEPROM.put(90, t18);  // записали в EEPROM
        EEPROM.put(95, t19);  // записали в EEPROM
        EEPROM.put(100, t20); // записали в EEPROM
        EEPROM.put(105, t21); // записали в EEPROM
        EEPROM.put(110, t22); // записали в EEPROM
        EEPROM.put(115, t23); // записали в EEPROM
    }
}
//!======================= EEPROMRead ====================================================
void EEPROMRead()
{
    if (EEPROM.read(INIT_ADDR) != INIT_KEY)
    {                                      // первый запуск (ЕСЛИ INIT_ADDR (1023)не равен INIT_KEY (50) то записать EEPROM.write(INIT_ADDR, INIT_KEY);EEPROM.put(0, izmenenieTemp);
        EEPROM.write(INIT_ADDR, INIT_KEY); // записали ключ

        EEPROM.write(0, t0);    // записали стандартное значение температуры. в данном случае это значение переменной, объявленное выше
        EEPROM.write(5, t1);    // записали стандартное значение температуры. в данном случае это значение переменной, объявленное выше
        EEPROM.write(10, t2);   // записали стандартное значение температуры. в данном случае это значение переменной, объявленное выше
        EEPROM.write(15, t3);   // записали стандартное значение температуры. в данном случае это значение переменной, объявленное выше
        EEPROM.write(20, t4);   // записали стандартное значение температуры. в данном случае это значение переменной, объявленное выше
        EEPROM.write(25, t6);   // записали стандартное значение температуры. в данном случае это значение переменной, объявленное выше
        EEPROM.write(30, t7);   // записали стандартное значение температуры. в данном случае это значение переменной, объявленное выше
        EEPROM.write(35, t8);   // записали стандартное значение температуры. в данном случае это значение переменной, объявленное выше
        EEPROM.write(40, t9);   // записали стандартное значение температуры. в данном случае это значение переменной, объявленное выше
        EEPROM.write(45, t10);  // записали стандартное значение температуры. в данном случае это значение переменной, объявленное выше
        EEPROM.write(50, t11);  // записали стандартное значение температуры. в данном случае это значение переменной, объявленное выше
        EEPROM.write(55, t12);  // записали стандартное значение температуры. в данном случае это значение переменной, объявленное выше
        EEPROM.write(60, t13);  // записали стандартное значение температуры. в данном случае это значение переменной, объявленное выше
        EEPROM.write(65, t14);  // записали стандартное значение температуры. в данном случае это значение переменной, объявленное выше
        EEPROM.write(70, t15);  // записали стандартное значение температуры. в данном случае это значение переменной, объявленное выше
        EEPROM.write(75, t16);  // записали стандартное значение температуры. в данном случае это значение переменной, объявленное выше
        EEPROM.write(80, t17);  // записали стандартное значение температуры. в данном случае это значение переменной, объявленное выше
        EEPROM.write(85, t18);  // записали стандартное значение температуры. в данном случае это значение переменной, объявленное выше
        EEPROM.write(90, t19);  // записали стандартное значение температуры. в данном случае это значение переменной, объявленное выше
        EEPROM.write(95, t20);  // записали стандартное значение температуры. в данном случае это значение переменной, объявленное выше
        EEPROM.write(100, t21); // записали стандартное значение температуры. в данном случае это значение переменной, объявленное выше
        EEPROM.write(105, t22); // записали стандартное значение температуры. в данном случае это значение переменной, объявленное выше
        EEPROM.write(110, t23); // записали стандартное значение температуры. в данном случае это значение переменной, объявленное выше
        EEPROM.commit();
    }
    t0 = EEPROM.read(0);    //считывание из EEPROM значений
    t1 = EEPROM.read(5);    //считывание из EEPROM значений
    t2 = EEPROM.read(10);   //считывание из EEPROM значений
    t3 = EEPROM.read(15);   //считывание из EEPROM значений
    t4 = EEPROM.read(20);   //считывание из EEPROM значений
    t5 = EEPROM.read(25);   //считывание из EEPROM значений
    t6 = EEPROM.read(30);   //считывание из EEPROM значений
    t7 = EEPROM.read(35);   //считывание из EEPROM значений
    t8 = EEPROM.read(40);   //считывание из EEPROM значений
    t9 = EEPROM.read(45);   //считывание из EEPROM значений
    t10 = EEPROM.read(50);  //считывание из EEPROM значений
    t11 = EEPROM.read(55);  //считывание из EEPROM значений
    t12 = EEPROM.read(60);  //считывание из EEPROM значений
    t13 = EEPROM.read(65);  //считывание из EEPROM значений
    t14 = EEPROM.read(70);  //считывание из EEPROM значений
    t15 = EEPROM.read(75);  //считывание из EEPROM значений
    t16 = EEPROM.read(80);  //считывание из EEPROM значений
    t17 = EEPROM.read(85);  //считывание из EEPROM значений
    t18 = EEPROM.read(90);  //считывание из EEPROM значений
    t19 = EEPROM.read(95);  //считывание из EEPROM значений
    t20 = EEPROM.read(100); //считывание из EEPROM значений
    t21 = EEPROM.read(105); //считывание из EEPROM значений
    t22 = EEPROM.read(110); //считывание из EEPROM значений
    t23 = EEPROM.read(115); //считывание из EEPROM значений
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
//! ===================== функция опроса статуса датчиков true/false =====================
void checkingSensor()
{
    if (DS18B20_sensor.readTemp()) // ЕСЛИ датчик DS18B20 считался == TRUE
    {
        DS18B20_status = true;
    }
    else
    {
        DS18B20_status = false;
    }

    bool ntc = therm.getTempAverage();
    if (ntc == 1) // ЕСЛИ датчик NTC считался == TRUE
    {
        NTC_status = false;
    }
    else
    {
        NTC_status = true;
    }

    if (isnan(dht.readHumidity())) // ЕСЛИ датчик считался TRUE
    {
        DHT_status = false;
    }
    else
    {
        DHT_status = true;
    }
    if (bme.readTemperature()) // ЕСЛИ датчик считался TRUE
    {
        BME_status = true;
    }
    else
    {
        BME_status = false;
    }
}
//! =============== функция индикации статусов датчиков на загрузочном экране ============
void setupScreenSensor()
{
    if (DS18B20_status == true) // ЕСЛИ датчик DS18B20 считался == TRUE
    {
        lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
        lcd.print("DS18B20         "); // Выводим текст
        FOR_LOAD();
        Serial.println("DS18B20 OK...");
        lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
        lcd.print("DS18B20 OK...   "); // Выводим текст
        delay(2000);
    }
    else
    {
        lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
        lcd.print("DS18B20         "); // Выводим текст
        FOR_LOAD();
        Serial.println("DS18B20 ERROR!!");
        lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
        lcd.print("DS18B20 ERROR!!!"); // Выводим текст
        delay(2000);
    }
    if (NTC_status == false) // ЕСЛИ датчик NTC считался == TRUE
    {
        lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
        lcd.print("NTC             "); // Выводим текст
        FOR_LOAD();
        Serial.println("NTC ERROR!!!    ");
        lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
        lcd.print("NTC ERROR!!!    "); // Выводим текст
        delay(2000);
    }
    else
    {
        lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
        lcd.print("NTC             "); // Выводим текст
        FOR_LOAD();
        Serial.println("NTC OK...       ");
        lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
        lcd.print("NTC OK...       "); // Выводим текст
        delay(2000);
    }

    if (DHT_status == false) // ЕСЛИ датчик считался TRUE
    {
        lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
        lcd.print("DHT11           "); // Выводим текст
        FOR_LOAD();
        Serial.println("DHT11 ERROR!!! ");
        lcd.setCursor(0, 1);                  // Устанавливаем курсор в начало 2 строки
        lcd.print(String("DHT11 ERROR!!! ")); // Выводим текст
        delay(2000);
    }
    else
    {
        lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
        lcd.print("DHT11           "); // Выводим текст
        FOR_LOAD();
        Serial.println("DHT11 OK");
        lcd.setCursor(0, 1);                   // Устанавливаем курсор в начало 2 строки
        lcd.print(String("DHT11 OK...     ")); // Выводим текст
        delay(2000);
    }

    if (BME_status == true) // ЕСЛИ датчик считался TRUE
    {
        lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
        lcd.print("BMP280          "); // Выводим текст
        FOR_LOAD();
        Serial.println("BMP280 OK");
        lcd.setCursor(0, 1);                   // Устанавливаем курсор в начало 2 строки
        lcd.print(String("BMP280 OK...    ")); // Выводим текст
        delay(2000);
    }
    else
    {
        lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
        lcd.print("BMP280          "); // Выводим текст
        FOR_LOAD();
        Serial.println("BMP280 ERROR!!! ");
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
//!======= функция присвоения температуры переменным на каждый час========================
void hour_temp()
{
    switch (hour)
    {
    case 0:
        t0 = DS18B20_sensor.getTemp();
        break;
    case 1:
        t1 = DS18B20_sensor.getTemp();
        break;
    case 2:
        t2 = DS18B20_sensor.getTemp();
        break;
    case 3:
        t3 = DS18B20_sensor.getTemp();
        break;
    case 4:
        t4 = DS18B20_sensor.getTemp();
        break;
    case 5:
        t5 = DS18B20_sensor.getTemp();
        break;
    case 6:
        t6 = DS18B20_sensor.getTemp();
        break;
    case 7:
        t7 = DS18B20_sensor.getTemp();
        break;
    case 8:
        t8 = DS18B20_sensor.getTemp();
        break;
    case 9:
        t9 = DS18B20_sensor.getTemp();
        break;
    case 10:
        t10 = DS18B20_sensor.getTemp();
        break;
    case 11:
        t11 = DS18B20_sensor.getTemp();
        break;
    case 12:
        t12 = DS18B20_sensor.getTemp();
        break;
    case 13:
        t13 = DS18B20_sensor.getTemp();
        break;
    case 14:
        t14 = DS18B20_sensor.getTemp();
        break;
    case 15:
        t15 = DS18B20_sensor.getTemp();
        break;
    case 16:
        t16 = DS18B20_sensor.getTemp();
        break;
    case 17:
        t17 = DS18B20_sensor.getTemp();
        break;
    case 18:
        t18 = DS18B20_sensor.getTemp();
        break;
    case 19:
        t19 = DS18B20_sensor.getTemp();
        break;
    case 20:
        t20 = DS18B20_sensor.getTemp();
        break;
    case 21:
        t21 = DS18B20_sensor.getTemp();
        break;
    case 22:
        t22 = DS18B20_sensor.getTemp();
        break;
    case 23:
        t23 = DS18B20_sensor.getTemp();
        break;
    }
}
//!============================= функция первого экрана ==================================
void firstScreen()
{
    Serial.println("===================== FirstScreen ====================================");
    Serial.println(rtc.getTimeStr());
    lcd.setCursor(8, 1);                       // Устанавливаем курсор в начало 2 строки
    lcd.print(rtc.getTimeStr());               // Выводим ВРЕМЯ на LCD дисплей
    lcd.setCursor(0, 0);                       // Устанавливаем курсор в начало 1 строки
    lcd.print(String("\2") + val_max + "\1 "); // Выводим MAX значение температуры на LCD дисплей
    lcd.setCursor(0, 1);                       // Устанавливаем курсор в начало 2 строки
    lcd.print(String("\3") + val_min + "\1 "); // Выводим МИН значение температуры на LCD дисплей
    checkingSensor();                          //!
    if (BME_status == 0)                       // ЕСЛИ датчик считался TRUE
    {
        lcd.setCursor(8, 0);      // Устанавливаем курсор в начало 2 строки
        lcd.print("BMP280 ERR!"); // Выводим текст на LCD дисплей
        Serial.println("BMP280 ERR!");
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
    Serial.println("========================= SecondScreen ===============================");
    Serial.println(rtc.getTimeStr());
    lcd.setCursor(8, 1);         // Устанавливаем курсор в начало 2 строки
    lcd.print(rtc.getTimeStr()); // Выводим ВРЕМЯ на LCD дисплей

    if (DS18B20_status == false) // ЕСЛИ датчик считался TRUE
    {
        Serial.println("Failed to read from DS18B20 sensor!");
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
    if (NTC_status == false) // ЕСЛИ датчик считался TRUE
    {
        Serial.println("Failed to read from NTC sensor!");
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
    if (DHT_status == false) // ЕСЛИ датчик считался TRUE
    {
        lcd.setCursor(8, 0);   // Устанавливаем курсор в начало 2 строки
        lcd.print("DHT ERR!"); // Выводим текст на LCD дисплей
        Serial.println("DHT ERR!");
    }
    else
    {
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
}
//!==================== функция считывания значений температуры из EEPROM ================
void readEEPROMTemp()
{
    t0 = EEPROM.read(0);    //считывание из EEPROM значений
    t1 = EEPROM.read(5);    //считывание из EEPROM значений
    t2 = EEPROM.read(10);   //считывание из EEPROM значений
    t3 = EEPROM.read(15);   //считывание из EEPROM значений
    t4 = EEPROM.read(20);   //считывание из EEPROM значений
    t5 = EEPROM.read(25);   //считывание из EEPROM значений
    t6 = EEPROM.read(30);   //считывание из EEPROM значений
    t7 = EEPROM.read(35);   //считывание из EEPROM значений
    t8 = EEPROM.read(40);   //считывание из EEPROM значений
    t9 = EEPROM.read(45);   //считывание из EEPROM значений
    t10 = EEPROM.read(50);  //считывание из EEPROM значений
    t11 = EEPROM.read(55);  //считывание из EEPROM значений
    t12 = EEPROM.read(60);  //считывание из EEPROM значений
    t13 = EEPROM.read(65);  //считывание из EEPROM значений
    t14 = EEPROM.read(70);  //считывание из EEPROM значений
    t15 = EEPROM.read(75);  //считывание из EEPROM значений
    t16 = EEPROM.read(80);  //считывание из EEPROM значений
    t17 = EEPROM.read(85);  //считывание из EEPROM значений
    t18 = EEPROM.read(90);  //считывание из EEPROM значений
    t19 = EEPROM.read(95);  //считывание из EEPROM значений
    t20 = EEPROM.read(100); //считывание из EEPROM значений
    t21 = EEPROM.read(105); //считывание из EEPROM значений
    t22 = EEPROM.read(110); //считывание из EEPROM значений
    t23 = EEPROM.read(115); //считывание из EEPROM значений
}
//!=========================== void setup() ==============================================
void setup()
{
    Serial.begin(115200);
    EEPROMRead();      //! функция считывания значений и записи в/из EEPROM при изменении значений
    EEPROM.begin(256); // активация функции EEPROM
    bme.begin();       // инициализация BME  датчика
    dht.begin();       // инициализация DHT11  датчика
    lcd.init();        // инициализация LCD

    lcd.backlight(); // Включаем подсветку LCD дислея

    lcd.createChar(1, degree1); //  Загружаем 2 символ ГРАДУСА в ОЗУ дисплея
    lcd.createChar(2, degree2); //  Загружаем 3 символ СТРЕЛКА ВВЕРХ в ОЗУ дисплея
    lcd.createChar(3, degree3); //  Загружаем 4 символ СТРЕЛКА ВНИЗ в ОЗУ дисплея
    lcd.createChar(4, degree4); //  Загружаем 4 символ СТРЕЛКА В ПРАВО в ОЗУ дисплея

    lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
    lcd.print("Blynk started..."); // Выводим текст
    Serial.println("Blynk started...");
    Blynk.begin(auth, ssid, pass); // подклчение к Blynk
    Serial.println("Blynk conect... ");
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

    delay(2000);         //  задержка в 2 сек. перед проверкой датчиков на наличие и испрвность
    checkingSensor();    //! функция опроса статуса датчиков true/false
    setupScreenSensor(); //! функция индикации статусов датчиков на загрузочном экране
    //!---------------------------------------------------------------------------------
    lcd.setCursor(0, 1);                   // Устанавливаем курсор в начало 2 строки
    lcd.print(String(".......GO.......")); // Выводим текст
    //!------------------------ затирание строк на дисплее------------------------------
    delay(2000);                   //  задержка в 2 сек. перед затиранием строк
    lcd.setCursor(0, 0);           // Устанавливаем курсор в начало 2 строки
    lcd.print("                "); // Выводим текст
    lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
    lcd.print("                "); // Выводим текст
    readEEPROMTemp();              //! функция считывания значений температуры из EEPROM
}
//!=======================================================================================
void loop()
{
    checkEEPROM();                //! функция проверки EEPROM после setBright() и записи новых значени
    readingValuesSensors();       //! функция опроса датчиков
    hour = timeClient.getHours(); // считывание часа (0....23) для дальнейшего присвоения температуры
    hour_temp();                  //! функция присвоения значения тепмператры переменным t0...t23

    int vertualPinBlynk[] = {V8, V9, V10, V11, V12, V13, V14, V15, V16, V17, V18, V19, V20, V21, V22, V23, V24, V25, V26, V27, V28, V29, V30, V31};   // массив виртуальных пинов Blynk
    float variableTemperatureHour[] = {t0, t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t13, t14, t15, t16, t17, t18, t19, t20, t21, t22, t23}; // массив данных о температуре за каждый час

    checkingSensor(); //! функция опроса статуса датчиков true/false                                                                                                                           //!
    firstScreen();    //! функция вызова первого экрана
    delay(1000);
    secondScreen(); //! функция вызова второго экрана
    delay(1000);

    static uint32_t tmr;
    if (millis() - tmr >= 600000) // обработка блока раз в 60 минут
    {
        setBright();                 //! функция поднятие флага и сброса таймера для записи в EEPROM
        EEPROMRead();                //! функция считывания значений и записи в/из EEPROM при изменении значений
        readEEPROMTemp();            //! функция считывания значений температуры из EEPROM
        lcd.setCursor(8, 1);         // Устанавливаем курсор в начало 2 строки
        lcd.print(rtc.getTimeStr()); // Выводим ВРЕМЯ на LCD дисплей

        //!--------------------------- цикл передачи в Blynk почасовых значений температуры --------------------------
        for (int8_t q = 0; q < 24; q++)
        {
            lcd.setCursor(8, 1);                                                // Устанавливаем курсор в начало 2 строки
            lcd.print(rtc.getTimeStr());                                        // Выводим ВРЕМЯ на LCD дисплей
            Blynk.virtualWrite(vertualPinBlynk[q], variableTemperatureHour[q]); // передача в Blynk  почасовых переменных температуры
            Serial.println(String("vertualPinBlynk[q] = ") + vertualPinBlynk[q] + ("  variableTemperatureHour[q] = ") + variableTemperatureHour[q]);
        }
        //!--------------------------- цикл вычисления MAX и MIN значений температуры --------------------------------
        for (int8_t i = 0; i < 24; i++)
        {
            val_max = max(variableTemperatureHour[i], val_max);
            val_min = min(variableTemperatureHour[i], val_min);
        }
        tmr = millis(); // сброс таймера
    }
    variablesForBlynk();         //! функция передачи данных в  Blynk
    lcd.setCursor(8, 1);         // Устанавливаем курсор в начало 2 строки
    lcd.print(rtc.getTimeStr()); // Выводим ВРЕМЯ на LCD дисплей
    Blynk.run();                 // запуск передачи значений в Blynk
    lcd.setCursor(8, 1);         // Устанавливаем курсор в начало 2 строки
    lcd.print(rtc.getTimeStr()); // Выводим ВРЕМЯ на LCD дисплей
}
