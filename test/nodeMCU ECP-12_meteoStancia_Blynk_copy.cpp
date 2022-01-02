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
#include <LiquidCrystal_I2C.h>
#include <DS1302.h>
#include <WiFiUdp.h>
#include <GyverBME280.h>

#define DHTPIN 2 // Digital pin D1 connected to the DHT sensor
#define DHTTYPE DHT11
#define DS_PIN 16 // ПИН термо датчика

const long utcOffsetInSeconds = 10800;

// Уникальные адреса датчиков - считать можно в примере address_read
uint8_t s1_addr[] = {0x28, 0x3B, 0xD6, 0x95, 0xF0, 0x1, 0x3C, 0xFD};
uint8_t s2_addr[] = {0x28, 0x28, 0xF, 0x4, 0x0, 0x0, 0x0, 0x68};

DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);    // set the LCD address to 0x27 for a 16 chars and 2 line display
MicroDS18B20<DS_PIN, s1_addr> sensor1; // Создаем термометр с адресацией
MicroDS18B20<DS_PIN, s2_addr> sensor2; // Создаем термометр с адресацией
DS1302 rtc(0, 12, 14);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);
GyverBME280 bme;

float DHT_h;
float newH1;
float DHT_t;
float temp_home;
float temp_out;
float BMP280_t;
int BMP280_h;
int hour;
int minu;
int sek;
int day;
int month;
int year;
bool flag = 1;
bool flag1 = 1;

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
byte degree[8] =                    // кодируем символ градуса
    {
        B00111,
        B00101,
        B00111,
        B00000,
        B00000,
        B00000,
        B00000,
};

byte degree1[8] = // кодируем символ градуса
    {
        B00000,
        B00000,
        B00011,
        B00101,
        B01001,
        B01001,
        B01001,
};
byte degree2[8] = // кодируем символ градуса
    {
        B00000,
        B00000,
        B10101,
        B01110,
        B00100,
        B01110,
        B10101,
};
byte degree3[8] = // кодируем символ градуса
    {
        B00000,
        B00000,
        B01010,
        B01010,
        B01110,
        B01010,
        B01010,
};
byte degree4[8] = // кодируем символ градуса
    {
        B00000,
        B00000,
        B01111,
        B01001,
        B01001,
        B01001,
        B01001,
};

byte degree5[8] = // кодируем символ градуса
    {
        0b00000,
        0b00011,
        0b00101,
        0b01001,
        0b10001,
        0b10001,
        0b10001,
        0b10001};
byte degree6[8] = // кодируем символ градуса
    {
        0b00000,
        0b10001,
        0b10001,
        0b01001,
        0b00101,
        0b00010,
        0b00100,
        0b01000};
//!-----------------------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);

  bme.setFilter(FILTER_COEF_8);              // Настраиваем коофициент фильтрации
  bme.setTempOversampling(OVERSAMPLING_8);   // Настраиваем передискретизацию для датчика температуры
  bme.setPressOversampling(OVERSAMPLING_16); // Настраиваем передискретизацию для датчика давления
  bme.setStandbyTime(STANDBY_500MS);         // Устанавливаем время сна между измерениями (у нас обычный циклический режим)
  bme.begin();                               // Если на этом настройки окончены - инициализируем датчик
  dht.begin();                               // активация DHT11  датчика
  lcd.init();                                // initialize the lcd

  lcd.backlight(); // Включаем подсветку
                   // lcd.createChar(1, symbol[0]); //  Загружаем 1 символ "к" в ОЗУ дисплея

  lcd.createChar(2, degree5); //  Загружаем 2 символ "и" в ОЗУ дисплея
  lcd.createChar(3, degree4); //  Загружаем 3 символ "й" в ОЗУ дисплея
  lcd.createChar(4, degree3); //  Загружаем 4 символ "я" в ОЗУ дисплея
  lcd.createChar(5, degree2); //  Загружаем 5 символ "з" в ОЗУ дисплея
  lcd.createChar(6, degree1); //  Загружаем 6 символ "ы" в ОЗУ дисплея
  lcd.createChar(7, degree);  //  Загружаем 6 символ "ы" в ОЗУ дисплея
  lcd.createChar(1, degree6); //  Загружаем 2 символ "и" в ОЗУ дисплея

  rtc.halt(false);
  rtc.writeProtect(false);
  lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
  lcd.print("Blynk started..."); // Выводим текст
  //Blynk.begin(auth, ssid, pass);
  lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
  lcd.print("Blynk conect... "); // Выводим текст

  timeClient.begin();
  timeClient.update();
  hour = timeClient.getHours();
  minu = timeClient.getMinutes();
  sek = timeClient.getSeconds();
  rtc.setTime(hour, minu, sek);

  DHT_h = dht.readHumidity();

  DHT_t = dht.readTemperature();
  BMP280_t = bme.readTemperature();
  BMP280_h = pressureToMmHg(bme.readPressure());
  sensor1.requestTemp();
  sensor2.requestTemp(); // Запрашиваем преобразование температуры

  // temp_home = sensor1.getTemp();
  //temp_out = sensor2.getTemp();

  delay(5000);
  //!==================================================================================

  if (sensor1.getTemp())
  {
    Serial.println("DS18B20 IN OK");
    lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
    lcd.print("DS18B20 IN OK..."); // Выводим текст
  }
  else
  {
    Serial.println("DS18B20 IN ERR");
    lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
    lcd.print("DS18B20 IN ERR!!"); // Выводим текст
  }
  //!==================================================================================

  if (isnan(sensor2.getTemp()))
  {
    Serial.println("DS18B20 OUT ER!!");
    lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
    lcd.print("DS18B20 OUT ER!!"); // Выводим текст
  }
  else
  {
    Serial.println("DS18B20 OUT OK..");
    lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
    lcd.print("DS18B20 OUT OK.."); // Выводим текст
  }
  //!==================================================================================

  if (dht.readHumidity())
  {
    Serial.println("DHT11 ERROR!!! ");
    lcd.setCursor(0, 1);                   // Устанавливаем курсор в начало 2 строки
    lcd.print(String("DHT11  ERR!!!   ")); // Выводим текст
  }
  else
  {
    Serial.println("DHT11 OK");
    lcd.setCursor(0, 1);                   // Устанавливаем курсор в начало 2 строки
    lcd.print(String("DHT11 OK...     ")); // Выводим текст
  }
  //!==================================================================================

  if (bme.readTemperature())
  {
    Serial.println("BMP280 ERROR!!! ");
    lcd.setCursor(0, 1);                    // Устанавливаем курсор в начало 2 строки
    lcd.print(String("BMP280  ERR!!!   ")); // Выводим текст
  }
  else
  {
    Serial.println("BMP280 OK");
    lcd.setCursor(0, 1);                    // Устанавливаем курсор в начало 2 строки
    lcd.print(String("BMP280 OK...     ")); // Выводим текст
  }
  //!==================================================================================
  delay(2000);
  lcd.setCursor(0, 1);                        // Устанавливаем курсор в начало 2 строки
  lcd.print(String(".......GO.......     ")); // Выводим текст

  //!==================================================================================
  delay(2000);
  lcd.setCursor(0, 1);                            // Устанавливаем курсор в начало 2 строки
  lcd.print("                                 "); // Выводим текст
}
//!-----------------------------------------------------------------------------------
void loop()
{

  DHT_h = dht.readHumidity();
  sensor1.requestTemp(); // Запрашиваем преобразование температуры
  sensor2.requestTemp();
  temp_home = sensor1.getTemp();
  temp_out = sensor2.getTemp();
  DHT_t = dht.readTemperature();
  BMP280_t = bme.readTemperature();
  BMP280_h = pressureToMmHg(bme.readPressure());

  //!==================================================================================
  if (sensor1.readTemp())
  {
    temp_home = sensor1.getTemp();
    Serial.println(String("Температура ") + temp_home + " Cͦͦ");
    lcd.setCursor(0, 0);                        // Устанавливаем курсор в начало 2 строки
    lcd.print(String("") + temp_home + "C\7 "); // Выводим текст
  }
  else
  {
    //  Serial.println("Failed to read from DS18B20 sensor!");
    lcd.setCursor(0, 0);   // Устанавливаем курсор в начало 2 строки
    lcd.print("ERROR!!!"); // Выводим текст // lcd.print("ERR: DS18B20 IN!"); // Выводим текст
  }
  //!==================================================================================
  if (sensor2.readTemp())
  {
    temp_out = sensor2.getTemp();
    Serial.println(String("Температура ") + temp_out + " Cͦͦ");
    lcd.setCursor(0, 1);                       // Устанавливаем курсор в начало 2 строки
    lcd.print(String("") + temp_out + "C\7 "); // Выводим текст
  }
  else
  {
    // Serial.println("Failed to read from DS18B20 sensor!");
    lcd.setCursor(0, 1);   // Устанавливаем курсор в начало 2 строки
    lcd.print("ERROR!!!"); // Выводим текст // lcd.print("ERR: DS18B20 IN!"); // Выводим текст
  }
  //!==================================================================================
  // Read Humidity
  // if humidity read failed, don't change h value
  lcd.setCursor(8, 1);         // Устанавливаем курсор в начало 2 строки
  lcd.print(rtc.getTimeStr()); // Выводим текст
  if (isnan(dht.readTemperature()))
  {
    lcd.setCursor(8, 0);   // Устанавливаем курсор в начало 2 строки
    lcd.print("DHT ERR!"); // Выводим текст
    Serial.println("DHT ERR!");
  }
  else
  {
    delay(3000);
    Serial.println(String("Влжность ") + DHT_h + "%");
    lcd.setCursor(8, 0);                   // Устанавливаем курсор в начало 2 строки
    lcd.print(String(" ") + DHT_h + "% "); // Выводим текст
  }

  //!==================================================================================
  lcd.setCursor(8, 1);         // Устанавливаем курсор в начало 2 строки
  lcd.print(rtc.getTimeStr()); // Выводим текст
  if (isnan(bme.readTemperature()))
  {
    lcd.setCursor(8, 0);      // Устанавливаем курсор в начало 2 строки
    lcd.print("BMP280 ERR!"); // Выводим текст
    Serial.println("BMP280 ERR!");
  }
  else
  {
    delay(3000);

    lcd.setCursor(8, 0);                        // Устанавливаем курсор в начало 2 строки
    lcd.print(String("") + BMP280_h + "mm Hg"); // Выводим текст
    Serial.println(BMP280_h);
  }
  static uint32_t myTimer = 0;
  if (millis() - myTimer > 5000)
  {
    flag = 1;
    flag1 = 0;
    myTimer = millis();
  }
  else
  {
    flag = 0;
    flag1 = 1;
  }
  //!==================================================================================
  Blynk.virtualWrite(V0, DHT_h);
  Blynk.virtualWrite(V1, temp_home);
  Blynk.virtualWrite(V2, temp_out);
  Blynk.virtualWrite(V3, BMP280_h);
  Blynk.virtualWrite(V4, BMP280_t);
  Blynk.virtualWrite(V5, DHT_t);

  lcd.setCursor(8, 1);         // Устанавливаем курсор в начало 2 строки
  lcd.print(rtc.getTimeStr()); // Выводим текст
                               // lcd.setCursor(9, 1);         // Устанавливаем курсор в начало 2 строки
                               // lcd.print(rtc.getDateStr()); // Выводим текст
                               /*
  Serial.println(rtc.getDOWStr(FORMAT_SHORT));
  Serial.println(rtc.getTimeStr());
  Serial.println(rtc.getDateStr()); // Start the Serial Monitor
*/
                               // Serial.println(timeClient.getFormattedTime());

  Blynk.run();
}
