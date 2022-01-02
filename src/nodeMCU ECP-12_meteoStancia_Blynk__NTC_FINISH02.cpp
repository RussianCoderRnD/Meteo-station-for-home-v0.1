
//! ============== Blynk block =================================

#define BLYNK_TEMPLATE_ID "TMPLEAFuc2FF"
#define BLYNK_DEVICE_NAME "ESP Meteo"
#define BLYNK_AUTH_TOKEN "X6a0vuSe3wdQmPrk_vKY898fChF5LEIx"

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

//! =============== setPin block ================================
#define DHTPIN 2      //ПИН D4 DHT термодатчика
#define DHTTYPE DHT11 //тип DHT термодатчика
#define DS_PIN 16     // ПИН D0 термо датчика DS18B20
#define NTC_PIN A0    // ПИН термо датчика NTC

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
bool ntc;

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
//! =============== функция проверки DS18B20 датчика на исправность ======================
void checkingDS18B20_sensor()
{
    if (DS18B20_sensor.readTemp()) // ЕСЛИ датчик DS18B20 считался == TRUE
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
}
//! =============== функция проверки NTC датчика на исправность ==========================
void checkingNTC_sensor()
{
    bool ntc = therm.getTempAverage();
    if (ntc == 0) // ЕСЛИ датчик NTC считался == TRUE
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
}
//! =============== функция проверки DHT датчика на исправность ==========================
void checkingDHT_sensor()
{
    if (isnan(dht.readHumidity())) // ЕСЛИ датчик считался TRUE
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
}
//! =============== функция проверки BME датчика на исправность ==========================
void checkingBME_sensor()
{
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
    if (flag == 1)
    {
        lcd.setCursor(8, 1);         // Устанавливаем курсор в начало 2 строки
        lcd.print(rtc.getTimeStr()); // Выводим ВРЕМЯ на LCD дисплей
        screenTwo = 0;               // обнуление значения счёчика второго экрана
        screenOne++;
        lcd.setCursor(0, 0);                       // Устанавливаем курсор в начало 1 строки
        lcd.print(String("\2") + val_max + "\1 "); // Выводим MAX значение температуры на LCD дисплей
        lcd.setCursor(0, 1);                       // Устанавливаем курсор в начало 2 строки
        lcd.print(String("\3") + val_min + "\1 "); // Выводим МИН значение температуры на LCD дисплей

        if (isnan(bme.readTemperature())) // ЕСЛИ датчик считался TRUE
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

        if (screenOne == delayScreenOne)
            flag = 0;
    }
}
//!============================= функция второго экрана ==================================
void secondScreen()
{
    if (flag == 0)
    {
        lcd.setCursor(8, 1);         // Устанавливаем курсор в начало 2 строки
        lcd.print(rtc.getTimeStr()); // Выводим ВРЕМЯ на LCD дисплей
        screenOne = 0;               // обнуление значения счёчика первого экрана
        screenTwo++;

        if (isnan(DS18B20_sensor.readTemp())) // ЕСЛИ датчик считался TRUE
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
        if (isnan(therm.getTempAverage())) // ЕСЛИ датчик считался TRUE
        {
            Serial.println("Failed to read from DS18B20 sensor!");
            lcd.setCursor(0, 1);   // Устанавливаем курсор в начало 2 строки
            lcd.print("ERROR!!!"); // Выводим текст // lcd.print("ERR: DS18B20 IN!"); // Выводим текст
        }
        else
        {
            outdoorTemperature = therm.getTempAverage();
            lcd.setCursor(0, 1);                                 // Устанавливаем курсор в начало 2 строки
            lcd.print(String(" ") + outdoorTemperature + "\1 "); // Выводим текст на LCD дисплей
        }
    }
    if (isnan(dht.readTemperature())) // ЕСЛИ датчик считался TRUE
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
    if (screenTwo == delayScreenTwo)
        flag = 1;
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
//!=========================== void setup() ==============================================
void setup()
{
    Serial.begin(115200);

    bme.begin(); // инициализация BME  датчика
    dht.begin(); // инициализация DHT11  датчика
    lcd.init();  // инициализация LCD

    lcd.backlight(); // Включаем подсветку LCD дислея

    lcd.createChar(1, degree1); //  Загружаем 2 символ ГРАДУСА в ОЗУ дисплея
    lcd.createChar(2, degree2); //  Загружаем 3 символ СТРЕЛКА ВВЕРХ в ОЗУ дисплея
    lcd.createChar(3, degree3); //  Загружаем 4 символ СТРЕЛКА ВНИЗ в ОЗУ дисплея
    lcd.createChar(4, degree4); //  Загружаем 4 символ СТРЕЛКА В ПРАВО в ОЗУ дисплея

    lcd.setCursor(0, 1);           // Устанавливаем курсор в начало 2 строки
    lcd.print("Blynk started..."); // Выводим текст
    //Blynk.begin(auth, ssid, pass); // подклчение к Blynk
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
    //rtc.setTime(hour, minu, sek);   // запись в модуль РЕАЛЬНОГО ВРЕМЕНИ значения ЧАСА, МИНУТ, СЕКУНД

    //!------------------------ проверка датчинов на TRUE ------------------------------
    delay(2000); //  задержка в 2 сек. перед проверкой датчиков на наличие и испрвность
    checkingDS18B20_sensor();
    checkingNTC_sensor();
    checkingDHT_sensor();
    checkingBME_sensor();
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

//!=======================================================================================
void loop()
{
    readingValuesSensors();       //! функция опроса датчиков
    hour = timeClient.getHours(); // считывание часа (0....23) для дальнейшего присвоения температуры
    hour_temp();                  //! функция присвоения значения тепмператры переменным t0...t23

    int vertualPinBlynk[] = {V8, V9, V10, V11, V12, V13, V14, V15, V16, V17, V18, V19, V20, V21, V22, V23, V24, V25, V26, V27, V28, V29, V30, V31};   // массив виртуальных пинов Blynk
    float variableTemperatureHour[] = {t0, t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t13, t14, t15, t16, t17, t18, t19, t20, t21, t22, t23}; // массив данных о температуре за каждый час

    firstScreen();  //! функция первого экрана
    secondScreen(); //! функция второго экрана

    static uint32_t tmr;
    if (millis() - tmr >= 59000) // обработка блока раз в 59 минут
    {
        lcd.setCursor(8, 1);         // Устанавливаем курсор в начало 2 строки
        lcd.print(rtc.getTimeStr()); // Выводим ВРЕМЯ на LCD дисплей

        for (int8_t q = 0; q < 24; q++)
        {
            lcd.setCursor(8, 1);                                                // Устанавливаем курсор в начало 2 строки
            lcd.print(rtc.getTimeStr());                                        // Выводим ВРЕМЯ на LCD дисплей
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
    variablesForBlynk();         //! функция передачи данных в  Blynk
    lcd.setCursor(8, 1);         // Устанавливаем курсор в начало 2 строки
    lcd.print(rtc.getTimeStr()); // Выводим ВРЕМЯ на LCD дисплей
    Blynk.run();                 // запуск передачи значений в Blynk
    lcd.setCursor(8, 1);         // Устанавливаем курсор в начало 2 строки
    lcd.print(rtc.getTimeStr()); // Выводим ВРЕМЯ на LCD дисплей
}
