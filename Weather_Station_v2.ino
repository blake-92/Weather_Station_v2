/*
Placa: ESP32 T-Call v1.4 SIM800L
*/

/////////////////////////////////////////////////////////////////////////////////////
// Serial Monitor
#define SerialMon Serial

// Variables SIM800L
#define TINY_GSM_MODEM_SIM800
#define TINY_GSM_DEBUG SerialMon
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false

#define MODEM_RST 5
#define MODEM_PWKEY 4
#define MODEM_POWER_ON 23
#define MODEM_TX 27
#define MODEM_RX 26
#define SerialAT Serial1

// Variables GPRS
const char apn[]      = "internet.tigo.bo";
const char gprsUser[] = "";
const char gprsPass[] = "";

// Variables Wifi
const char wifiSSID[] = "YourSSID";
const char wifiPass[] = "YourWiFiPass";

// Variables MQTT
const char* broker = "146.190.156.216";
const char* mqtt_id = "ThingName";
const char* mqtt_user = "";
const char* mqtt_pass = "";
uint32_t lastReconnectAttempt = 0;

const char* topicLed       = "topic/led";
const char* topicInit      = "topic/init";
const char* topicLedStatus = "topic/ledStatus";
const char* topicData = "topic/weather/thing1";

// Variables GPS NEO8M
#define RX_PIN 14
#define TX_PIN 12
#define GPS_BAUD 9600
#define SerialGPS Serial2

// Variables BME280
#define SEALEVELPRESSURE_HPA (1013.25)
#define BME_DIR   0x76
#define I2C_SDA_2 19
#define I2C_SCL_2 18
#define I2C_BAUD  400000

// Variables SD Card
#define MY_CS       25
#define MY_SCLK     0
#define MY_MOSI     2
#define MY_MISO     15

/////////////////////////////////////////////////////////////////////////////////////
// Libraries GPRS
#include <TinyGsmClient.h>
// Libraries MQTT
#include <PubSubClient.h>
// Libraries JSON
#include <ArduinoJson.h>
// Libraries GPS NEO8M
#include <TinyGPSPlus.h>
// Libraries BME280
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
// Libraries SD Card
#include <SPI.h>
#include <SD.h>

/////////////////////////////////////////////////////////////////////////////////////
// Objects GPRS
TinyGsm        modem(SerialAT);
TinyGsmClient client(modem);

// Objects MQTT
PubSubClient  mqtt(client);

// Objects GPS NEO8M
TinyGPSPlus gps;

// Objects BME280
TwoWire I2CBME = TwoWire(1);
Adafruit_BME280 bme;

// Objects SD Card
File myFile;

/////////////////////////////////////////////////////////////////////////////////////

#define LED_PIN 13
int ledStatus = LOW;

String satellites_gps = "";
String latitude_gps = "";
String longitude_gps = "";
String date_gps = "";
String time_gps = "";
String altitude1_gps = "";
String temperature_bme = "";
String humidity_bme = "";
String pressure_bme = "";
String altitude2_bme = "";

/////////////////////////////////////////////////////////////////////////////////////

// Listen for incoming messages
void mqttCallback(char* topic, byte* payload, unsigned int len) {
  SerialMon.print("Message arrived [");
  SerialMon.print(topic);
  SerialMon.print("]: ");
  SerialMon.write(payload, len);
  SerialMon.println();

  // Only proceed if incoming message's topic matches
  if (String(topic) == topicLed) {
    ledStatus = !ledStatus;
    digitalWrite(LED_PIN, ledStatus);
    mqtt.publish(topicLedStatus, ledStatus ? "1" : "0");
  }
}

// Conects this client to your broker
boolean mqttConnect() {
  SerialMon.print("Connecting to ");
  SerialMon.print(broker);

  boolean status = mqtt.connect(mqtt_id);
  // Or, if you want to authenticate MQTT:
  // boolean status = mqtt.connect(mqtt_id, mqtt_user, mqtt_pass);
  if (status == false) {
    SerialMon.println(" fail");
    return false;
  }
  SerialMon.println(" success");

  publishStatus("started", mqtt_id);
  mqtt.subscribe(topicLed);
  return mqtt.connected();
}

void setup() {

  // Set console baud rate
  SerialMon.begin(115200);
  delay(10);
  SerialMon.println(String("Initializing: ") + mqtt_id);

  // Initializing GPS
  SerialGPS.begin(GPS_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  I2CBME.begin(I2C_SDA_2, I2C_SCL_2, I2C_BAUD);

  // Initializing BME280
  if (!bme.begin(BME_DIR, &I2CBME)) {
    SerialMon.println("BME280 Not found.");
    while(1);
  } else {
    SerialMon.println("BME280 OK.");
  }

  // Initializing MicroSD
  SPI.begin(MY_SCLK, MY_MISO, MY_MOSI, MY_CS);
  if(!SD.begin(MY_CS)) {
    SerialMon.println("MicroSD card mounting failed.");
    while(1);
  } else {
    SerialMon.println("MicroSD OK.");
  }

  pinMode(LED_PIN, OUTPUT);

  // Initializing SIM800L
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);
  delay(1000);
  SerialMon.println("Wait...");

  SerialAT.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(6000);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  modem.restart();
  // modem.init();

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);

#if TINY_GSM_USE_WIFI
  // Wifi connection parameters must be set before waiting for the network
  SerialMon.print(F("Setting SSID/password..."));
  if (!modem.networkConnect(wifiSSID, wifiPass)) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");
#endif

#if TINY_GSM_USE_GPRS && defined TINY_GSM_MODEM_XBEE
  // The XBee must run the gprsConnect function BEFORE waiting for network!
  modem.gprsConnect(apn, gprsUser, gprsPass);
#endif

  SerialMon.print("Waiting for network...");
  if (!modem.waitForNetwork()) {
    SerialMon.println(" fail");
    delay(300000);
    return;
  }
  SerialMon.println(" success");

  if (modem.isNetworkConnected()) { SerialMon.println("Network connected"); }

#if TINY_GSM_USE_GPRS
  // GPRS connection parameters are usually set after network registration
  SerialMon.print(F("Connecting to "));
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");

  if (modem.isGprsConnected()) { SerialMon.println("GPRS connected"); }
#endif

  // MQTT Broker setup
  mqtt.setServer(broker, 1883);
  mqtt.setCallback(mqttCallback);
}

void loop() {

  // Update data variables
  sampling();

  // Print data to serial
  printToSerial();

  // Write data to SD
  writeToSD();
  
  // Publish data to broker
  publishData();

  // Data sampling every X Milliseconds
  smartDelay(10000);

  // GPS is active?
  if (millis() > 5000 && gps.charsProcessed() < 10)
    SerialMon.println(F("No GPS data received: check wiring"));

}

/////////////////////////////////////////////////////////////////////

// Processes GPS data while counting a delay, without blocking the program
static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    // Make sure we're still registered on the network
    if (!modem.isNetworkConnected()) {
      SerialMon.println("Network disconnected");
      if (!modem.waitForNetwork(180000L, true)) {
        SerialMon.println(" fail");
        delay(10000);
        return;
      }
      if (modem.isNetworkConnected()) {
        SerialMon.println("Network re-connected");
      }

#if TINY_GSM_USE_GPRS
      // and make sure GPRS/EPS is still connected
      if (!modem.isGprsConnected()) {
        SerialMon.println("GPRS disconnected!");
        SerialMon.print(F("Connecting to "));
        SerialMon.print(apn);
        if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
          SerialMon.println(" fail");
          delay(10000);
          return;
        }
        if (modem.isGprsConnected()) { SerialMon.println("GPRS reconnected"); }
      }
#endif
    }

    if (!mqtt.connected()) {
      SerialMon.println("=== MQTT NOT CONNECTED ===");
      // Reconnect every 10 seconds
      uint32_t t = millis();
      if (t - lastReconnectAttempt > 10000L) {
        lastReconnectAttempt = t;
        if (mqttConnect()) { lastReconnectAttempt = 0; }
      }
      delay(100);
      return;
    }

    mqtt.loop();
    while (SerialGPS.available())
      gps.encode(SerialGPS.read());
  } while (millis() - start < ms);
}

// Build the JSON DATA and publish it
static void publishData() {
  StaticJsonDocument<200> doc;
  doc["sat"] = satellites_gps;
  doc["lat"] = latitude_gps;
  doc["lng"] = longitude_gps;
  doc["date"] = date_gps;
  doc["time"] = time_gps;
  doc["alt1"] = altitude1_gps;
  doc["temp"] = temperature_bme;
  doc["hum"] = humidity_bme;
  doc["pre"] = pressure_bme;
  doc["alt2"] = altitude2_bme;
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer);
 
  mqtt.publish(topicData, jsonBuffer);
}

static void publishStatus(const char* status, const char* id) {
  StaticJsonDocument<200> doc;
  doc["status"] = status;
  doc["id"] = id;
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer);
 
  mqtt.publish(topicInit, jsonBuffer);
}

// Logs recording to SD card
static void writeToSD() {
  myFile = SD.open("/log.txt", FILE_APPEND);

  if (myFile) {
    myFile.print(formatForSD());
    myFile.print(",");
    myFile.println();
    myFile.close();
  } else {
    SerialMon.println("Error opening FILE");
  }
  smartDelay(0);
}

// Build a CSV format
static String formatForSD() {
  String dataString = "";
  dataString += satellites_gps;
  dataString += ",";
  dataString += latitude_gps;
  dataString += ",";
  dataString += longitude_gps;
  dataString += ",";
  dataString += date_gps;
  dataString += ",";
  dataString += time_gps;
  dataString += ",";
  dataString += altitude1_gps;
  dataString += ",";
  dataString += temperature_bme;
  dataString += ",";
  dataString += humidity_bme;
  dataString += ",";
  dataString += pressure_bme;
  dataString += ",";
  dataString += altitude2_bme;
  return dataString;
}

//Build a Serial Format
static void printToSerial() {
  printInt(gps.satellites.value(), gps.satellites.isValid());
  SerialMon.print(",");
  printFloat(gps.location.lat(), gps.location.isValid(), 6);
  SerialMon.print(",");
  printFloat(gps.location.lng(), gps.location.isValid(), 6);
  SerialMon.print(",");
  printDateTime(gps.date, gps.time);
  SerialMon.print(",");
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 2);
  SerialMon.print(",");
  SerialMon.print(bme.readTemperature());
  SerialMon.print(",");
  SerialMon.print(bme.readHumidity());
  SerialMon.print(",");
  SerialMon.print(bme.readPressure() / 100.0F);
  SerialMon.print(",");
  SerialMon.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  SerialMon.print(",");
  SerialMon.println();
  smartDelay(0);
}

// GPS data formatting functions for serial port
static void printFloat(float val, bool valid, int prec) {
  if (!valid)
    SerialMon.print("*");
  else
    SerialMon.print(val, prec);

  smartDelay(0);
}

static void printInt(unsigned long val, bool valid) {
  if (!valid)
    SerialMon.print("*");
  else
    SerialMon.print(val);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t) {
  if (!d.isValid())
    SerialMon.print(F("**/**/**"));
  else
    SerialMon.printf("%02d/%02d/%02d", d.month(), d.day(), d.year());

  SerialMon.print(",");

  if (!t.isValid())
    SerialMon.print(F("**:**:**"));
  else
    SerialMon.printf("%02d:%02d:%02d", t.hour(), t.minute(), t.second());

  smartDelay(0);
}

static void sampling() {
  satellites_gps = String(gps.satellites.value());
  latitude_gps = String(gps.location.lat(), 6);
  longitude_gps = String(gps.location.lng(), 6);
  date_gps = String(gps.date.month()) + "/" + String(gps.date.day()) + "/" + String(gps.date.year());
  time_gps = String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second());
  altitude1_gps = String(gps.altitude.meters(), 2);
  temperature_bme = String(bme.readTemperature());
  humidity_bme = String(bme.readHumidity());
  pressure_bme = String(bme.readPressure() / 100.0F);
  altitude2_bme = String(bme.readAltitude(SEALEVELPRESSURE_HPA));
  smartDelay(0);
}
