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

// I2C для SIM800 
//TwoWire I2CPower = TwoWire(0);

Adafruit_BMP280 bmp;
AHT10 myAHT20(AHT10_ADDRESS_0X38, AHT20_SENSOR);

// клиент TinyGSM для подключения к интернету
TinyGsmClient client(modem);
 
#define uS_TO_S_FACTOR 1000000UL   /* преобразуем микросекунды в секунды */
#define TIME_TO_SLEEP  15        /* время спящего режима 1 час = 3600 секунд */
 
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
 
void setup() {
  // запускаем монитор порта
  SerialMon.begin(115200);

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
  delay(3000);

  while (myAHT20.begin() != true) {
    Serial.println(F("AHT20 not connected or fail to load calibration coefficient")); //(F()) save string to flash & keeps dynamic memory free
    delay(5000);
  }
  Serial.println(F("AHT20 OK"));
  
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  // Перезапускаем модуль SIM800
  // Для пропуска вместо restart() напишите init(0
  SerialMon.println("Initializing modem...");
  modem.restart();
 
  ThingSpeak.begin(client);
  // Выставляем пробуждение по таймеру  
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

}
 
void loop() {
  float temperature;
  float humidity;
  float pressure;

  SerialMon.print("Connecting to APN: ");
  SerialMon.print(apn);

  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" fail");
    ESP.restart();
  }
  else {
    SerialMon.println(" OK");
    temperature = myAHT20.readTemperature();
    ThingSpeak.setField(1, temperature);
    humidity = myAHT20.readHumidity();
    ThingSpeak.setField(2, humidity);
    pressure = bmp.readPressure();
    ThingSpeak.setField(3, pressure);
  }
  int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
    /*int x = ThingSpeak.writeField(myChannelNumber, 1, myAHT20.readTemperature(), myWriteAPIKey);
    Serial.printf("Temperature: %.02f *C\n", myAHT20.readTemperature());
    int y = ThingSpeak.writeField(myChannelNumber, 2, myAHT20.readHumidity(), myWriteAPIKey);
    Serial.printf("Humidity: %.02f %RH\n", myAHT20.readHumidity());
    int z = ThingSpeak.writeField(myChannelNumber, 3, bmp.readPressure(), myWriteAPIKey);
    Serial.printf("Pressure: %.02f hPa\n", bmp.readPressure());*/
  SerialMon.println("Данные отправлены");
  
  // Переходим в спящий режим
  esp_deep_sleep_start();
}