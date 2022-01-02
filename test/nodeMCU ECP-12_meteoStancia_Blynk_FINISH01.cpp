#define BLYNK_TEMPLATE_ID "TMPLEAFuc2FF"
#define BLYNK_DEVICE_NAME "ESP Meteo"
#define BLYNK_AUTH_TOKEN "X6a0vuSe3wdQmPrk_vKY898fChF5LEIx"

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h> // библиотека Blynk
#include <DHT.h>                // библиотека DHT датчика
#include <microDS18B20.h>       // библиотека управления термо регулятором
#include <Wire.h>
#include <NTPClient.h>
#include <LiquidCrystal_I2C.h> // библиотека LCD 1602 дисплея
#include <DS1302.h>            // библиотека модуля РЕАЛЬНОГО ВРМЕНИ
#include <WiFiUdp.h>
#include <GyverBME280.h> // библиотека термодатчика BME280
#include <GyverNTC.h>

#define DHTPIN 2      //ПИН DHT термодатчика
#define DHTTYPE DHT11 //тип DHT термодатчика
#define DS_PIN 16     // ПИН термо датчика

const long utcOffsetInSeconds = 10800; // коректировка часового пояса +3

// Уникальные адреса датчиков - считать можно в примере address_read
uint8_t s2_addr[] = {0x28, 0x3B, 0xD6, 0x95, 0xF0, 0x1, 0x3C, 0xFD}; // Адресс термодатчика DS18B20 №1
uint8_t s1_addr[] = {0x28, 0x28, 0xF, 0x4, 0x0, 0x0, 0x0, 0x68};     // Адресс термодатчика DS18B20 №1

DHT dht(DHTPIN, DHTTYPE);              // Создаём DHT термодатчик
LiquidCrystal_I2C lcd(0x27, 16, 2);    // set the LCD address to 0x27 for a 16 chars and 2 line display
MicroDS18B20<DS_PIN, s1_addr> sensor1; // Создаем термометр с адресацией
MicroDS18B20<DS_PIN, s2_addr> sensor2; // Создаем термометр с адресацией
DS1302 rtc(0, 12, 14);                 //Создаём МОДУЛЬ РЕАЛЬНОГО ВРЕМЕНИ
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds); // Сайт этолонного времени
GyverBME280 bme;
GyverNTC therm(0, 10000, 3435); // пин, сопротивление при 25 градусах (R термистора = R резистора!), бета-коэффициент
// GyverNTC therm(0, 10000, 3435, 25, 10000);	// пин, R термистора, B термистора, базовая температура, R резистора
// серый 4300
// проводной 3950                                            // Создаём датчик BME280

float DHT_h;                                                 // переменная для влажности с DHT
float DHT_t;                                                 // переменная для температуры с DHT
float temp_home;                                             // переменння для температуры в доме DS18B20
float temp_out;                                              // переменная для температуры на улице DS18B20
float BMP280_t;                                              // переменная для температуры BMP280
float t0, t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12; // переменные для значения температуры на каждый час
float t13, t14, t15, t16, t17, t18, t19, t20, t21, t22, t23; // переменные для значения температуры на каждый час
int BMP280_h;                                                // переменная давления BMP280
int hour;                                                    // переменная ЧАСЫ
int minu;                                                    // переменная МИНУТЫ
int sek;                                                     // переменная СЕКУНДЫ
int del1 = 10;                                               // задержка отображения информации на экране №1
int del2 = 20;                                               // задержка отображения информации на экране №2
bool flag = 1;
int dis11, dis12;

//! ========= BLYNK ============
char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "itel A16 Plus_plus";
char pass[] = "Acer5560g!";

uint8_t symbol[6][8] = {            //  Объявляем массив из 6 собственных символов (к и й я з ы), каждый символ состоит из 8 байт
    {0, 0, 18, 20, 24, 20, 18, 0},  //  к
    {0, 0, 17, 19, 21, 25, 17, 0},  //  и
    {10, 4, 17, 19, 21, 25, 17, 0}, //  й
    {0, 0, 15, 17, 15, 5, 9, 0},    //  я
    {0, 0, 14, 17, 6, 17, 14, 0},   //  з
    {0, 0, 17, 17, 29, 19, 29, 0}}; //  ы

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
byte degree2[8] = // кодируем символ градуса
    {
        B00100,
        B01110,
        B10101,
        B00100,
        B00100,
        B00100,
        B00100,
};
byte degree3[8] = // кодируем символ градуса
    {
        B00100,
        B00100,
        B00100,
        B00100,
        B10101,
        B01110,
        B00100,
};
byte degree4[8] = // кодируем символ градуса
    {
        B11000,
        B01100,
        B00110,
        B00011,
        B00110,
        B01100,
        B11000,
};

//!===================================================================================
void FOR_LOAD(void)
{
  for (int i = 0; i < 16; i++)
  {
    lcd.setCursor(i, 0); // Устанавливаем курсор в начало 2 строки
    lcd.print("\4");     // Выводим текст
    delay(50);
  }
  for (int i = 0; i < 16; i++)
  {
    lcd.setCursor(i, 0); // Устанавливаем курсор в начало 2 строки
    lcd.print("  ");     // Выводим текст
    delay(50);
  }
  lcd.setCursor(0, 0);           // Устанавливаем курсор в начало 2 строки
  lcd.print("                "); // Выводим текст
}
//!===================================================================================
void setup()
{
#ifdef serialenabled
  Serial.begin(115200);
#endif

  // bme.setFilter(FILTER_COEF_8);              // Настраиваем коофициент фильтрации
  // bme.setTempOversampling(OVERSAMPLING_8);   // Настраиваем передискретизацию для датчика температуры
  // bme.setPressOversampling(OVERSAMPLING_16); // Настраиваем передискретизацию для датчика давления
  // bme.setStandbyTime(STANDBY_500MS);         // Устанавливаем время сна между измерениями (у нас обычный циклический режим)
  bme.begin(); // Если на этом настройки окончены - инициализируем датчик
  dht.begin(); // инициализация DHT11  датчика
  lcd.init();  // инициализация LCD

  lcd.backlight(); // Включаем подсветку LCD дислея

  lcd.createChar(1, degree1); //  Загружаем 2 символ "и" в ОЗУ дисплея
  lcd.createChar(2, degree2); //  Загружаем 3 символ "й" в ОЗУ дисплея
  lcd.createChar(3, degree3); //  Загружаем 4 символ "я" в ОЗУ дисплея
  lcd.createChar(4, degree4); //  Загружаем 4 символ "я" в ОЗУ дисплея

  lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
  lcd.print("Blynk started..."); // Выводим текст
  Blynk.begin(auth, ssid, pass); // подклчение к Blynk
  lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
  lcd.print("Blynk conect... "); // Выводим текст
                                 //! ------------работа с модулем РЕАЛЬНОГО ВРЕМЕНИ------------------------
  timeClient.begin();            // инициализация МОДУЛЯ РЕАЛЬНОГО ВРЕМЕНИ
  timeClient.update();           // обновление МОДУЛЯ РЕАЛЬНОГО ВРЕМЕНИ
  rtc.halt(false);
  rtc.writeProtect(false);
  hour = timeClient.getHours();   // получение значения ЧАСов из Blynk
  minu = timeClient.getMinutes(); // получение значения МИНут из Blynk
  sek = timeClient.getSeconds();  // получение значения СЕКУнд из Blynk
  rtc.setTime(hour, minu, sek);   // запись в модуль РЕАЛЬНОГО ВРЕМЕНИ значения ЧАСА, МИНУТ, СЕКУНД

  //! ------------опрос датчиков температуры/давления/влажности------------
  DHT_h = dht.readHumidity();                    // считывание ВЛАЖНОСТИ с DHT датчика
  DHT_t = dht.readTemperature();                 // считывание ТЕМПЕРАТУРЫ с DHT датчика
  BMP280_t = bme.readTemperature();              // считывание ТЕМПЕРАТУРЫ с BMP280 датчика
  BMP280_h = pressureToMmHg(bme.readPressure()); // считывание ДАВЛЕНИЯ с BMP280 датчика
  sensor1.requestTemp();                         // считывание ТЕМПЕРАТУРЫ с DS18B20 датчика №1
  sensor2.requestTemp();                         // считывание ТЕМПЕРАТУРЫ с DS18B20 датчика №2

  delay(2000); // пауза перед проверкой датчиков на наличие и испрвность TRUE

  //!==================== проверка датчинов на TRUE===================================

  if (isnan(sensor1.readTemp())) // ЕСЛИ датчик считался TRUE
  {
    lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
    lcd.print("DS18B20 IN      "); // Выводим текст
    FOR_LOAD();

    Serial.println("DS18B20 1 IN ERR");
    lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
    lcd.print("DS18B20 IN ERR!!"); // Выводим текст
    delay(2000);
  }
  else
  {
    lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
    lcd.print("DS18B20 IN      "); // Выводим текст
    FOR_LOAD();
    Serial.println("DS18B20 1 IN OK");
    lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
    lcd.print("DS18B20 IN OK..."); // Выводим текст
    delay(2000);
  }
  //!---------------------------------------------------------------------------------

  if (isnan(sensor2.readTemp())) // ЕСЛИ датчик считался TRUE
  {
    lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
    lcd.print("DS18B20 OUT     "); // Выводим текст
    FOR_LOAD();
    Serial.println("DS18B20 2 OUT ERR!!");
    lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
    lcd.print("DS18B20 OUT ERR!"); // Выводим текст
    delay(2000);
  }
  else
  {
    lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
    lcd.print("DS18B20 OUT     "); // Выводим текст
    FOR_LOAD();
    Serial.println("DS18B20 2 OUT OK..");
    lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
    lcd.print("DS18B20 OUT OK.."); // Выводим текст
    delay(2000);
  }
  //!----------------------------------------------------------------------------------

  if (isnan(dht.readHumidity())) // ЕСЛИ датчик считался TRUE
  {
    lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
    lcd.print("DHT11           "); // Выводим текст
    FOR_LOAD();

    Serial.println("DHT11 ERROR!!! ");
    lcd.setCursor(0, 1);                   // Устанавливаем курсор в начало 2 строки
    lcd.print(String("DHT11  ERROR!!! ")); // Выводим текст
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
  //!----------------------------------------------------------------------------------

  if (bme.readTemperature()) // ЕСЛИ датчик считался TRUE
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
  //!----------------------------------------------------------------------------------

  lcd.setCursor(0, 1);                   // Устанавливаем курсор в начало 2 строки
  lcd.print(String(".......GO.......")); // Выводим текст

  //!----------------------------------------------------------------------------------
  delay(2000);
  lcd.setCursor(0, 0);           // Устанавливаем курсор в начало 2 строки
  lcd.print("                "); // Выводим текст
  lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
  lcd.print("                "); // Выводим текст
}
//!======= функция присвоения температуры переменным на каждый час=====================
void hour_temp()
{
  hour = timeClient.getHours(); // считывание часа (0....23) для дальнейшего присвоения температуры
  switch (hour)
  {
  case 0:
    t0 = sensor1.getTemp();
    break;
  case 1:
    t1 = sensor1.getTemp();
    break;
  case 2:
    t2 = sensor1.getTemp();
    break;
  case 3:
    t3 = sensor1.getTemp();
    break;
  case 4:
    t4 = sensor1.getTemp();
    break;
  case 5:
    t5 = sensor1.getTemp();
    break;
  case 6:
    t6 = sensor1.getTemp();
    break;
  case 7:
    t7 = sensor1.getTemp();
    break;
  case 8:
    t8 = sensor1.getTemp();
    break;
  case 9:
    t9 = sensor1.getTemp();
    break;
  case 10:
    t10 = sensor1.getTemp();
    break;
  case 11:
    t11 = sensor1.getTemp();
    break;
  case 12:
    t12 = sensor1.getTemp();
    break;
  case 13:
    t13 = sensor1.getTemp();
    break;
  case 14:
    t14 = sensor1.getTemp();
    break;
  case 15:
    t15 = sensor1.getTemp();
    break;
  case 16:
    t16 = sensor1.getTemp();
    break;
  case 17:
    t17 = sensor1.getTemp();
    break;
  case 18:
    t18 = sensor1.getTemp();
    break;
  case 19:
    t19 = sensor1.getTemp();
    break;
  case 20:
    t20 = sensor1.getTemp();
    break;
  case 21:
    t21 = sensor1.getTemp();
    break;
  case 22:
    t22 = sensor1.getTemp();
    break;
  case 23:
    t23 = sensor1.getTemp();
    break;
  }
}
//!===================================================================================
void loop()
{
  sensor1.requestTemp();                         // считывание ТЕМПЕРАТУРЫ с DS18B20 датчика №1
  sensor2.requestTemp();                         // считывание ТЕМПЕРАТУРЫ с DS18B20 датчика №2
  DHT_h = dht.readHumidity();                    // считывание ВЛАЖНОСТИ с DHT
  DHT_t = dht.readTemperature();                 // считывание ТЕМПЕРАТУРЫ с DHT
  BMP280_t = bme.readTemperature();              // считывание ТЕМПЕРАТУРЫ с BMP280
  temp_home = sensor1.getTemp();                 // преобразование значений с термодатчика №1
  temp_out = sensor2.getTemp();                  // преобразование значений с термодатчика №1
  BMP280_h = pressureToMmHg(bme.readPressure()); // считывание и преобразование ДАВЛЕНИЯ с BMP280

  hour_temp(); // вызов функции сохранения температуры по часам

  float vals[24] = {t0, t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t13, t14, t15, t16, t17, t18, t19, t20, t21, t22, t23}; // массив данных о температуре за каждый час
  float val_max = 0;                                                                                                               // опорное значения для функции max
  float val_min = 60;                                                                                                              // опорное значения для функции min

  //!----------------цикл вычисления MAX и MIN значений температуры -------------------------------
  for (int8_t i = 0; i < 24; i++)
  {
    val_max = max(vals[i], val_max);
    val_min = min(vals[i], val_min);
    Serial.print(String("t") + i);
    Serial.println(String(" = ") + vals[i]);
  }
  //!------------ функция первого экрана -----------------------------------------------------------
  if (flag == 1)
  {
    dis12 = 0; // обнуление значения счёчика второго экрана
    dis11++;
    lcd.setCursor(0, 0);                       // Устанавливаем курсор в начало 1 строки
    lcd.print(String("\2") + val_max + "\1 "); // Выводим MAX значение температуры на LCD дисплей
    lcd.setCursor(0, 1);                       // Устанавливаем курсор в начало 2 строки
    lcd.print(String("\3") + val_min + "\1 "); // Выводим МИН значение температуры на LCD дисплей
    lcd.setCursor(8, 1);                       // Устанавливаем курсор в начало 2 строки
    lcd.print(rtc.getTimeStr());               // Выводим ВРЕМЯ на LCD дисплей

    if (isnan(bme.readTemperature())) // ЕСЛИ датчик считался TRUE
    {
      lcd.setCursor(8, 0);      // Устанавливаем курсор в начало 2 строки
      lcd.print("BMP280 ERR!"); // Выводим текст на LCD дисплей
      Serial.println("BMP280 ERR!");
    }
    else
    {
      lcd.setCursor(7, 0);                         // Устанавливаем курсор в начало 2 строки
      lcd.print(String(" ") + BMP280_h + "mm Hg"); // Выводим текст на LCD дисплей
    }

    Serial.println(String("val_max  ") + val_max);
    Serial.println(String("val_min  ") + val_min);
    if (dis11 == del1)
      flag = 0;
  }

  //!-----------------------функция второго экрана--------------------------------------------------
  if (flag == 0)
  {
    dis11 = 0; // обнуление значения счёчика первого экрана
    dis12++;

    if (isnan(sensor1.readTemp())) // ЕСЛИ датчик считался TRUE
    {
      Serial.println("Failed to read from DS18B20 sensor!");
      lcd.setCursor(0, 0);   // Устанавливаем курсор в начало 2 строки
      lcd.print("ERROR!!!"); // Выводим текст // lcd.print("ERR: DS18B20 IN!"); // Выводим текст
    }
    else
    {
      temp_home = sensor1.getTemp();
      lcd.setCursor(0, 0);                        // Устанавливаем курсор в начало 2 строки
      lcd.print(String(" ") + temp_home + "\1 "); // Выводим текст на LCD дисплей
    }
    //!-----------------------------------------------------------------------------------
    if (isnan(sensor2.readTemp())) // ЕСЛИ датчик считался TRUE
    {
      Serial.println("Failed to read from DS18B20 sensor!");
      lcd.setCursor(0, 1);   // Устанавливаем курсор в начало 2 строки
      lcd.print("ERROR!!!"); // Выводим текст // lcd.print("ERR: DS18B20 IN!"); // Выводим текст
    }
    else
    {
      temp_out = sensor2.getTemp();
      lcd.setCursor(0, 1);                       // Устанавливаем курсор в начало 2 строки
      lcd.print(String(" ") + temp_out + "\1 "); // Выводим текст на LCD дисплей
    }
    //!-----------------------------------------------------------------------------------
    lcd.setCursor(8, 1);         // Устанавливаем курсор в начало 2 строки
    lcd.print(rtc.getTimeStr()); // Выводим текст на LCD дисплей

    if (isnan(dht.readTemperature())) // ЕСЛИ датчик считался TRUE
    {
      lcd.setCursor(8, 0);   // Устанавливаем курсор в начало 2 строки
      lcd.print("DHT ERR!"); // Выводим текст на LCD дисплей
      Serial.println("DHT ERR!");
    }
    else
    {
      Serial.println(String("Влжность ") + DHT_h + "%");
      lcd.setCursor(8, 0);                   // Устанавливаем курсор в начало 2 строки
      lcd.print(String(" ") + DHT_h + "% "); // Выводим текст на LCD дисплей
    }
    if (dis12 == del2)
      flag = 1;
  }
  //!==================================================================================
  Blynk.virtualWrite(V0, DHT_h);     // передача значений в Blynk
  Blynk.virtualWrite(V1, temp_home); // передача значений в Blynk
  Blynk.virtualWrite(V2, temp_out);  // передача значений в Blynk
  Blynk.virtualWrite(V3, BMP280_h);  // передача значений в Blynk
  Blynk.virtualWrite(V4, BMP280_t);  // передача значений в Blynk
  Blynk.virtualWrite(V5, DHT_t);     // передача значений в Blynk
  Blynk.run();                       // запуск передачи значений в Blynk
}
// lcd.setCursor(9, 1);         // Устанавливаем курсор в начало 2 строки
// lcd.print(rtc.getDateStr()); // Выводим текст
//Serial.println(rtc.getDOWStr(FORMAT_SHORT));
//Serial.println(rtc.getTimeStr());
//Serial.println(rtc.getDateStr()); // Start the Serial Monitor
// Serial.println(timeClient.getFormattedTime());