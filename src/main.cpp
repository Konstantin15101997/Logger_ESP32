//Мой MAC адресс: 34:98:7A:B9:F2:39

// Настройки GPRS (если не требуются, оставьте пустыми)
const char apn[]      = "internet.mts.ru"; //APN
const char gprsUser[] = "mts"; // имя пользователя
const char gprsPass[] = "mts"; // пароль
 
unsigned long myChannelNumber = 2;
const char * myWriteAPIKey = "Q0337TE8T23TVSYI";

int send;

// пины платы TTGO T-Call
#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26
#define I2C_SDA              21
#define I2C_SCL              22
// пины BME280
#define I2C_SDA_2            18
#define I2C_SCL_2            19
 
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
#include <esp_now.h>
#include <WiFi.h>

// I2C для SIM800 
//TwoWire I2CPower = TwoWire(0);

Adafruit_INA219 ina219; 
Adafruit_BMP280 bmp;
AHT10 myAHT20(AHT10_ADDRESS_0X38, AHT20_SENSOR);

#define MY_PERIOD 120000  // период в мс
uint32_t tmr1;         // переменная таймера
int count;

float temperature;
float humidity;
float pressure;
float Voltage;
float Current; 

typedef struct climate {
  float temperature_esp8266;
  float humidity_esp8266;
  float pressure_esp8266;
} climate;
 
climate Data_climate;


bool status_AHT20;
bool status_BMP280;
bool status_INA219;


// клиент TinyGSM для подключения к интернету
TinyGsmClient client(modem);
 
#define uS_TO_S_FACTOR 1000000UL   /* преобразуем микросекунды в секунды */
#define TIME_TO_SLEEP  480       /* время спящего режима 1 час = 3600 секунд */
 
/*#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00
 
bool setPowerBoostKeepOn(int en){
  I2CPower.beginTransmission(IP5306_ADDR);
  I2CPower.write(IP5306_REG_SYS_CTL0);
  if (en) {
    I2CPower.write(0x37); // если 1 включаем плату, если 0 - выключаем
  } else {
    I2CPower.write(0x35); // 0x37 – значение по умолчанию
  }
  return I2CPower.endTransmission() == 0;
}*/
 
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&Data_climate, incomingData, sizeof(Data_climate));
  Serial.println(Data_climate.temperature_esp8266);
  Serial.println(Data_climate.humidity_esp8266);
  Serial.println(Data_climate.pressure_esp8266);
}

void setup() {
  // запускаем монитор порта
  SerialMon.begin(115200);
  WiFi.mode(WIFI_AP_STA);
  
  // Запускаем протокол ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  WiFi.disconnect();
  // Начинаем подключение I2C
  /*I2CPower.begin(I2C_SDA, I2C_SCL, 400000);

  // Не выключаем плату при питании от батареи
  bool isOk = setPowerBoostKeepOn(1);
  SerialMon.println(String("IP5306 KeepOn ") + (isOk ? "OK" : "FAIL"));*/
 
  // Сброс, включение и контакты питания
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);
 
  // Устанавливаем скорость передачи данных модуля GSM и контакты UART.
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
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
  tmr1=millis();
}
 
void loop() {
  Serial.println("Wait");
  while(millis()-tmr1<=MY_PERIOD){
    Serial.println("Wait 120 секунд");
    if (status_INA219==1){
      Voltage = 0;
    } else{
      Voltage = ina219.getBusVoltage_V();
    }

    if (status_BMP280==1){
      pressure = 0;
    } else{
      pressure=bmp.readPressure();
    }
    
    if (millis()-tmr1>=(MY_PERIOD-20000)){
      Serial.println("Прошло 100 секунд");
      delay(5000);
      count+=1;
      if (count==1){
        SerialMon.print("Connecting to APN: ");
        SerialMon.print(apn);
      
        if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
          SerialMon.println(" fail");
          ESP.restart();
        }
        else {
          SerialMon.println(" OK");

          if (status_AHT20 == 1){
            temperature = 0;
            humidity = 0;
          } else{
            temperature = myAHT20.readTemperature();
            humidity = myAHT20.readHumidity();
          }

          ThingSpeak.setField(1, temperature);
          ThingSpeak.setField(2, humidity);
          ThingSpeak.setField(3, pressure);
          ThingSpeak.setField(4, Voltage);

          ThingSpeak.setField(5, Data_climate.temperature_esp8266);
          ThingSpeak.setField(6, Data_climate.humidity_esp8266);
          ThingSpeak.setField(7, Data_climate.pressure_esp8266);

        }
        int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
      }
    }
  }

  // Переходим в спящий режим
  esp_deep_sleep_start();
}