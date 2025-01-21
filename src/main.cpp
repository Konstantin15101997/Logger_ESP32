// Настройки GPRS (если не требуются, оставьте пустыми)
const char apn[]      = "internet.mts.ru"; //APN
const char gprsUser[] = "mts"; // имя пользователя
const char gprsPass[] = "mts"; // пароль
 
unsigned long myChannelNumber = 1;
const char * myWriteAPIKey = "JWULUMV8FTTZSW69";

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

#include <Adafruit_Sensor.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>

// I2C для SIM800 
TwoWire I2CPower = TwoWire(0);

TwoWire I2CBMP = TwoWire(1);
Adafruit_BMP280 bmp;

// клиент TinyGSM для подключения к интернету
TinyGsmClient client(modem);
 
#define uS_TO_S_FACTOR 1000000UL   /* преобразуем микросекунды в секунды */
#define TIME_TO_SLEEP  15        /* время спящего режима 1 час = 3600 секунд */
 
#define IP5306_ADDR          0x75
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
}
 
void setup() {
  // запускаем монитор порта
  SerialMon.begin(115200);

  // Начинаем подключение I2C
  I2CPower.begin(I2C_SDA, I2C_SCL, 400000);
  I2CBMP.begin(I2C_SDA_2, I2C_SCL_2, 400000);
  // Не выключаем плату при питании от батареи
  bool isOk = setPowerBoostKeepOn(1);
  SerialMon.println(String("IP5306 KeepOn ") + (isOk ? "OK" : "FAIL"));
 
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

  if (!bmp.begin(0x77, &I2CBMP)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     // Режим работы
                  Adafruit_BMP280::SAMPLING_X2,     // Точность изм. температуры
                  Adafruit_BMP280::SAMPLING_X16,    // Точность изм. давления
                  Adafruit_BMP280::FILTER_X16,      // Уровень фильтрации
                  Adafruit_BMP280::STANDBY_MS_500); // Период просыпания, мСек
  // Перезапускаем модуль SIM800
  // Для пропуска вместо restart() напишите init(0
  SerialMon.println("Initializing modem...");
  modem.restart();
 
  ThingSpeak.begin(client);
  // Выставляем пробуждение по таймеру  
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

}
 
void loop() {
  Serial.print(F("Pressure = "));
  Serial.print(bmp.readPressure());  // Функция измерения атм. давления
  Serial.println(" Pa");

  SerialMon.print("Connecting to APN: ");
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" fail");
    ESP.restart();
  }
  else {
    SerialMon.println(" OK");
    int x = ThingSpeak.writeField(myChannelNumber, 1, 500, myWriteAPIKey);
    SerialMon.println("Данные отправлены");
  }

  
  // Переходим в спящий режим
  esp_deep_sleep_start();
}