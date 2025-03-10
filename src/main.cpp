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
 
// настраиваем библиотеку TinyGSM
#define TINY_GSM_MODEM_SIM800      // SIM800 - модем
#define TINY_GSM_RX_BUFFER   1024  // Устанавливаем буфер равным 1Кб
 
#include <Wire.h>
#include <TinyGsmClient.h>
#include <ThingSpeak.h>
#include <Arduino.h>
#include <HTTPClient.h>
#include <Adafruit_BMP280.h>
#include <AHT10.h>
#include <Adafruit_INA219.h>
#include <WiFi.h>

const char server[] = "tchart.perepetsky.ru";
String serverName = "http://tchart.perepetsky.ru/t.php";

Adafruit_INA219 ina219; 
Adafruit_BMP280 bmp;
AHT10 myAHT20(AHT10_ADDRESS_0X38, AHT20_SENSOR);

float temperature_SERVER;
float humidity_SERVER;
float Voltage;

//UART
struct Str {
  float temperatura_AHT20;
  float humidity_AHT20;
  float temperatura_BME280;
  float humidity_BME280;
};

Str buf;

TinyGsm modem(Serial1);
// клиент TinyGSM для подключения к интернету
TinyGsmClient client(modem);
 
#define uS_TO_S_FACTOR 1000000UL   /* преобразуем микросекунды в секунды */
#define TIME_TO_SLEEP  3600      /* время спящего режима 1 час = 3600 секунд == 57 минут = 3420*/

void setup() {
  // запускаем монитор порта
  Serial.begin(115200);
  pinMode(13,OUTPUT);
  pinMode(33,OUTPUT);
  digitalWrite(33, HIGH);
  WiFi.mode(WIFI_AP_STA);
 
  // Сброс, включение и контакты питания
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);
 
  // Устанавливаем скорость передачи данных модуля GSM и контакты UART.
  Serial1.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  Serial2.begin(115200, SERIAL_8N1, 25, 12); 

  delay(2000);

  if (ina219.begin() != true) {       
    Serial.println("INA219 ERROR");
  }

  if (myAHT20.begin() != true) {
    Serial.println("AHT20 ERROR"); 
  }
  
  // Перезапускаем модуль SIM800
  // Для пропуска вместо restart() напишите init(0
  Serial.println("Initializing modem...");
  modem.restart();
  //modem.init();

  ThingSpeak.begin(client);

  // Выставляем пробуждение по таймеру  
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
}
 
void loop() {
  Voltage = ina219.getBusVoltage_V();
  
  if (Serial2.available()){
    Serial2.readBytes((byte*)&buf, sizeof(buf));
  }
  Serial.print("Connecting to APN: ");
  Serial.print(apn);  
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    Serial.println(" fail");
    esp_deep_sleep_start();
  }
  else {
    Serial.println(" OK");
    digitalWrite(13,HIGH);

    temperature_SERVER = myAHT20.readTemperature();
    humidity_SERVER = myAHT20.readHumidity();

    Serial.print("Connecting to ");
    Serial.print(server);
    if (!client.connect(server, 80)) {
        Serial.println(" fail");
        delay(10000);
        return;
    }
    Serial.println(" OK");
    String serverPath = serverName + "?s_t="+ String(temperature_SERVER) + "&s_h="+ String(humidity_SERVER) + "&s_p="+ String(0) + "&s_v="+ String(Voltage) + "&s1_t="+ String(buf.temperatura_AHT20) + "&s1_h="+ String(buf.humidity_AHT20) + "&s1_p="+ String(0) + "&s2_t="+ String(buf.temperatura_BME280) + "&s2_h="+ String(buf.humidity_BME280) + "&s2_p="+ String(0);
    
    // Make a HTTP GET request:
    Serial.println("Performing HTTP GET request...");
    client.print(String("GET ") + serverPath + " HTTP/1.1\r\n");
    client.print(String("Host: ") + server + "\r\n");
    client.print("Connection: close\r\n\r\n");
    client.println();

    unsigned long timeout = millis();
    while (client.connected() && millis() - timeout < 10000L) {
        // Print available data
        while (client.available()) {
            char c = client.read();
            Serial.print(c);
            timeout = millis();
        }
    }
    Serial.println();

    // Shutdown
    client.stop();
    Serial.println(F("Server disconnected"));

    ThingSpeak.setField(1, temperature_SERVER);
    ThingSpeak.setField(2, humidity_SERVER);
    ThingSpeak.setField(3, int(millis()));
    ThingSpeak.setField(4, Voltage);
  
    ThingSpeak.setField(5, buf.temperatura_AHT20);
    ThingSpeak.setField(6, buf.humidity_AHT20);
    ThingSpeak.setField(7, buf.temperatura_BME280);
    ThingSpeak.setField(8, buf.humidity_BME280);

    int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  }
  
  digitalWrite(33, LOW);
  digitalWrite(13,LOW);
  esp_deep_sleep_start();
}