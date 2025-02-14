//Мой MAC адресс: 34:98:7A:B9:F2:39

// Настройки GPRS (если не требуются, оставьте пустыми)
const char apn[]      = "internet.mts.ru"; //APN
const char gprsUser[] = "mts"; // имя пользователя
const char gprsPass[] = "mts"; // пароль
 
unsigned long myChannelNumber = 2;
const char * myWriteAPIKey = "Q0337TE8T23TVSYI";

// пины платы TTGO T-Call
#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26
#define I2C_SDA              21
#define I2C_SCL              22
 
// указываем порт для монитора порта
#define SerialMon Serial
// устанавливаем порт для команд AT
#define SerialAT Serial1
 
// настраиваем библиотеку TinyGSM
#define TINY_GSM_MODEM_SIM800      // SIM800 - модем
#define TINY_GSM_RX_BUFFER   1024  // Устанавливаем буфер равным 1Кб
 
// Определяем команды для монитора порта
//#define DUMP_AT_COMMANDS
 
#include <Wire.h>
#include <TinyGsmClient.h>
#include <ThingSpeak.h>
#include <Arduino.h>

#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, SerialMon);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif

#include <Adafruit_BMP280.h>
#include <AHT10.h>
#include <Adafruit_INA219.h>
#include <WiFi.h>

Adafruit_INA219 ina219; 
Adafruit_BMP280 bmp;
AHT10 myAHT20(AHT10_ADDRESS_0X38, AHT20_SENSOR);

float temperature_SERVER;
float humidity_SERVER;
float pressure_SERVER;
float Voltage;

//UART
struct Str {
  float temperatura;
  float humidity;
  float pressure;
};

Str buf;

bool status_AHT20;
bool status_BMP280;
bool status_INA219;

#define MY_PERIOD 60000  // период в мс
uint32_t tmr1;         // переменная таймера

// клиент TinyGSM для подключения к интернету
TinyGsmClient client(modem);
 
#define uS_TO_S_FACTOR 1000000UL   /* преобразуем микросекунды в секунды */
#define TIME_TO_SLEEP  3600      /* время спящего режима 1 час = 3600 секунд == 57 минут = 3420*/

void setup() {
  // запускаем монитор порта
  SerialMon.begin(115200);
  pinMode(13,OUTPUT);

  WiFi.mode(WIFI_AP_STA);


  WiFi.disconnect();
 
  // Сброс, включение и контакты питания
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);
 
  // Устанавливаем скорость передачи данных модуля GSM и контакты UART.
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);

  Serial2.begin(115200, SERIAL_8N1, 25, 12); 

  delay(2000);

  if (ina219.begin() != true) {       
    Serial.println("INA219 ERROR");
    status_INA219=1;
  }

  if (myAHT20.begin() != true) {
    Serial.println("AHT20 ERROR"); 
    status_AHT20=1;
  }
  
  if (bmp.begin() != true) {
    Serial.println("BMP280 ERROR");
    status_BMP280=1;
  }

  ina219.setCalibration_16V_400mA ();
  
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  // Перезапускаем модуль SIM800
  // Для пропуска вместо restart() напишите init(0
  SerialMon.println("Initializing modem...");
  //modem.restart();
  modem.init();

  ThingSpeak.begin(client);

  // Выставляем пробуждение по таймеру  
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
}
 
void loop() {
  Voltage = ina219.getBusVoltage_V();
  pressure_SERVER=bmp.readPressure();
  if (Serial2.available()){
    Serial2.readBytes((byte*)&buf, sizeof(buf));
    SerialMon.println(buf.temperatura);
    SerialMon.println(buf.humidity);
  }
  SerialMon.print("Connecting to APN: ");
  SerialMon.print(apn);  
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" fail");
    esp_deep_sleep_start();
  }
  else {
    SerialMon.println(" OK");
    digitalWrite(13,HIGH);
    temperature_SERVER = myAHT20.readTemperature();
    ThingSpeak.setField(1, temperature_SERVER);
    humidity_SERVER = myAHT20.readHumidity();
    ThingSpeak.setField(2, humidity_SERVER);
    ThingSpeak.setField(3, int(millis()));
    ThingSpeak.setField(4, Voltage);
  
    ThingSpeak.setField(7, buf.temperatura);
    ThingSpeak.setField(8, buf.humidity);
    

  }
  int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  digitalWrite(13,LOW);
  esp_deep_sleep_start();
}