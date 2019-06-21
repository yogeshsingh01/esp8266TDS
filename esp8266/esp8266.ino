
#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>         // https://github.com/tzapu/WiFiManager
#include "GravityTDS.h"

#define TdsSensorPin A0
GravityTDS gravityTds;
float temperature = 25,tdsValue = 0;
/****************************************************************************
 *       AWS
 **************************************************************************/
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h> //https://github.com/bblanchon/ArduinoJson (use v6.xx)
#include <time.h>
#define emptyString String()


//Enter values in secrets.h ▼
#include "secrets.h"

#if !(ARDUINOJSON_VERSION_MAJOR == 6 and ARDUINOJSON_VERSION_MINOR >= 7)
#error "Install ArduinoJson v6.7.0-beta or higher"
#endif

const int MQTT_PORT = 8883;
const char MQTT_SUB_TOPIC[] = "$aws/things/" THINGNAME "/shadow/update";
const char MQTT_PUB_TOPIC[] = "$aws/things/" THINGNAME "/shadow/update";

#ifdef USE_SUMMER_TIME_DST
uint8_t DST = 1;
#else
uint8_t DST = 0;
#endif

WiFiClientSecure net;

#ifdef ESP8266
BearSSL::X509List cert(cacert);
BearSSL::X509List client_crt(client_cert);
BearSSL::PrivateKey key(privkey);
#endif

PubSubClient client(net);

unsigned long lastMillis   = 0;
unsigned long lastMillisTDS = 0;
time_t now;
time_t nowish = 1510592825;


#define VREF       3.3

int pinState = HIGH;

///////////////////////////////////////////
static float sensorValue ;

const unsigned long longPressThreshold = 5000; // the threshold (in milliseconds) before a long press is detected
const unsigned long debounceThreshold = 50; // the threshold (in milliseconds) for a button press to be confirmed (i.e. not "noise")

unsigned long buttonTimer = 0; // stores the time that the button was pressed (relative to boot time)
unsigned long buttonPressDuration = 0; // stores the duration (in milliseconds) that the button was pressed/held down for

boolean buttonActive = false; // indicates if the button is active/pressed
boolean longPressActive = false; // indicate if the button has been long-pressed

// constants won't change. They're used here to set pin numbers:
const int buttonPin = 2;    // the number of the pushbutton pin
const int ledPin = 4;      // the number of the LED pin

// Variables will change:
int ledState = HIGH;         // the current state of the output pin
int wifiState = HIGH;
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers
unsigned long longPressTime = 500;

/////////////////////////////////////////////



// Set web server port number to 80
WiFiServer server(80);

// Variable to store the HTTP request
String header;

// Auxiliar variables to store the current output state
String output5State = "off";
String output4State = "off";

// Assign output variables to GPIO pins
const int output5 = 5;
const int input4 = 2;


////////////////////////////////////////////////////////////////

void connectToWiFi(String init_str)
{
  if (init_str != emptyString)
    Serial.print(init_str);
  if (WiFi.status() != WL_CONNECTED)
  {
    wifiState = !wifiState;
    digitalWrite(output5,wifiState);
    Serial.print(".");
    delay(1000);
  }
  wifiState = HIGH;
  digitalWrite(output5,wifiState);
  if (init_str != emptyString)
    Serial.println("ok!");
}

void setUpWifi()
{
    // WiFiManager
  // Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  
  // Uncomment and run it once, if you want to erase all the stored information
  //wifiManager.resetSettings();
  
  // set custom ip for portal
  //wifiManager.setAPConfig(IPAddress(10,0,1,1), IPAddress(10,0,1,1), IPAddress(255,255,255,0));

  // fetches ssid and pass from eeprom and tries to connect
  // if it does not connect it starts an access point with the specified name
  // here  "AutoConnectAP"
  // and goes into a blocking loop awaiting configuration
  wifiManager.autoConnect("IotAWS");
  // or use this for auto generated name ESP + ChipID
  //wifiManager.autoConnect();
  
  // if you get here you have connected to the WiFi
  Serial.println("Connected.");
  
  server.begin();
}

void NTPConnect(void)
{
  Serial.print("Setting time using SNTP");
  configTime(TIME_ZONE * 3600, DST * 3600, "pool.ntp.org", "time.nist.gov");
  now = time(nullptr);
  while (now < nowish)
  {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
  }
  Serial.println("done!");
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  Serial.print("Current time: ");
  Serial.print(asctime(&timeinfo));
}

void messageReceived(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Received [");
  Serial.print(topic);
  Serial.print("]: ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  //ledState = payload[0];
  Serial.println();
}
void pubSubErr(int8_t MQTTErr)
{
  if (MQTTErr == MQTT_CONNECTION_TIMEOUT)
    Serial.print("Connection tiemout");
  else if (MQTTErr == MQTT_CONNECTION_LOST)
    Serial.print("Connection lost");
  else if (MQTTErr == MQTT_CONNECT_FAILED)
    Serial.print("Connect failed");
  else if (MQTTErr == MQTT_DISCONNECTED)
    Serial.print("Disconnected");
  else if (MQTTErr == MQTT_CONNECTED)
    Serial.print("Connected");
  else if (MQTTErr == MQTT_CONNECT_BAD_PROTOCOL)
    Serial.print("Connect bad protocol");
  else if (MQTTErr == MQTT_CONNECT_BAD_CLIENT_ID)
    Serial.print("Connect bad Client-ID");
  else if (MQTTErr == MQTT_CONNECT_UNAVAILABLE)
    Serial.print("Connect unavailable");
  else if (MQTTErr == MQTT_CONNECT_BAD_CREDENTIALS)
    Serial.print("Connect bad credentials");
  else if (MQTTErr == MQTT_CONNECT_UNAUTHORIZED)
    Serial.print("Connect unauthorized");
}

void connectToMqtt(bool nonBlocking = false)
{
  Serial.print("MQTT connecting ");
  while (!client.connected())
  {
    if (client.connect(THINGNAME))
    {
      Serial.println("connected!");
      if (!client.subscribe(MQTT_SUB_TOPIC))
        pubSubErr(client.state());
    }
    else
    {
      Serial.print("failed, reason -> ");
      pubSubErr(client.state());
      if (!nonBlocking)
      {
        Serial.println(" < try again in 5 seconds");
        delay(5000);
      }
      else
      {
        Serial.println(" <");
      }
    }
    if (nonBlocking)
      break;
  }
}

void checkWiFiThenMQTT(void)
{
  connectToWiFi("Checking WiFi");
  connectToMqtt(true);
}

void sendData(void)
{
  DynamicJsonDocument jsonBuffer(JSON_OBJECT_SIZE(3) + 100);
  JsonObject root = jsonBuffer.to<JsonObject>();
  JsonObject state = root.createNestedObject("state");
  JsonObject state_reported = state.createNestedObject("reported");
  state_reported["value"] = ledState;
  Serial.printf("Sending  [%s]: ", MQTT_PUB_TOPIC);
  serializeJson(root, Serial);
  Serial.println();
  char shadow[measureJson(root) + 1];
  serializeJson(root, shadow, sizeof(shadow));
  if (!client.publish(MQTT_PUB_TOPIC, shadow, false))
    pubSubErr(client.state());
}

void sendDataTDS(void)
{
  DynamicJsonDocument jsonBuffer(JSON_OBJECT_SIZE(3) + 100);
  JsonObject root = jsonBuffer.to<JsonObject>();
  JsonObject state = root.createNestedObject("state");
  JsonObject state_reported = state.createNestedObject("reported");
  state_reported["TDSvalue"] = sensorValue;
  Serial.printf("Sending  [%s]: ", MQTT_PUB_TOPIC);
  serializeJson(root, Serial);
  Serial.println();
  char shadow[measureJson(root) + 1];
  serializeJson(root, shadow, sizeof(shadow));
  if (!client.publish(MQTT_PUB_TOPIC, shadow, false))
    pubSubErr(client.state());
}
////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200);

  gravityTds.setPin(TdsSensorPin);
  gravityTds.setAref(3.3);  //reference voltage on ADC, default 5.0V on Arduino UNO
  gravityTds.setAdcRange(1024);  //1024 for 10bit ADC;4096 for 12bit ADC
  gravityTds.begin();  //initialization
  
  // Initialize the output variables as outputs
  pinMode(output5, OUTPUT);
  pinMode(input4, INPUT_PULLUP);
  pinMode(ledPin,OUTPUT);
  // Set outputs to LOW
  //digitalWrite(output5, LOW);
  digitalWrite(output5, wifiState);
  //digitalWrite(output5,wifiState);


  WiFiManager wifiManager;
  
  wifiManager.autoConnect("IotAWS");
  Serial.println("Connected.");
  server.begin();

  connectToWiFi("Checking WiFi");

  ////////////////////////////////////////////////////////////
  
  NTPConnect();
  
#ifdef ESP32
  net.setCACert(cacert);
  net.setCertificate(client_cert);
  net.setPrivateKey(privkey);
#else
  net.setTrustAnchors(&cert);
  net.setClientRSACert(&client_crt, &key);
#endif

  client.setServer(MQTT_HOST, MQTT_PORT);
  client.setCallback(messageReceived);

    connectToMqtt();
}


float getTDSSensorData()
{
  /*float analogValue = analogRead(A0);

  float voltage = analogValue/1024*VREF;
  float ecValue=(133.42*voltage*voltage*voltage - 255.86*voltage*voltage + 857.39*voltage)*1;
  float ecValue25  =  ecValue / (1.0+0.02*(25-25.0));  //temperature compensation
  float tdsValue = ecValue25 * 0.5;
  analogValue    = (int)tdsValue;
  // map it to the range of the analog out:
  //int outputValue = map(sensorValue, 0, 1023, 0, 255);
  //Serial.println(sensorValue);
  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:
*/
    gravityTds.setTemperature(temperature);  // set the temperature and execute temperature compensation
    gravityTds.update();  //sample and calculate
    tdsValue = gravityTds.getTdsValue();  // then get the value
  return tdsValue;
}
int previous = LOW;    // the previous reading from the input pin
// the follow variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long timeCount1 = 0;         // the last time the output pin was toggled
long timeCount2 = 0;         // the last time the output pin was toggled
long debounce = 200;   // the debounce time, increase if the output flickers


void loop(){

  sensorValue = getTDSSensorData();
    // read the state of the switch into a local variable:
  // set the LED:
  digitalWrite(ledPin, ledState);

  // save the reading. Next time through the loop, it'll be the lastButtonState:
  //lastButtonState = reading;

  ////////////////////////////////////////////////////////////////////////
  // if the button pin reads LOW, the button is pressed (negative/ground switch)
  if (digitalRead(buttonPin) == LOW)
 {
  // mark the button as active, and start the timer
    if (buttonActive == false)
   {
       buttonActive = true;
       buttonTimer = millis();
   }

   // calculate the button press duration by subtracting the button time from the boot time
    buttonPressDuration = millis() - buttonTimer;
    
    // mark the button as long-pressed if the button press duration exceeds the long press threshold
    if ((buttonPressDuration > longPressThreshold) && (longPressActive == false))
    {
    longPressActive = true;
    Serial.print("Long press detected: ");
    Serial.println(buttonPressDuration);
    setUpWifi();
    connectToWiFi("Checking WiFi");
    
    }
    }

    // button either hasn't been pressed, or has been released
    else
    {
    // if the button was marked as active, it was recently pressed
    if (buttonActive == true)
    {
    // reset the long press active state
    if (longPressActive == true)
    {
    longPressActive = false;
    }
    
    // we either need to debounce the press (noise) or register a normal/short press
    else
    {
    // if the button press duration exceeds our bounce threshold, then we register a short press
    if (buttonPressDuration > debounceThreshold)
    {
    Serial.print("Short press detected: ");
    Serial.println(buttonPressDuration);
    ledState = !ledState;
    }
    
    // if the button press is less than our bounce threshold, we debounce (i.e. ignore as noise)
    else
    {
    Serial.print("Debounced: ");
    Serial.println(buttonPressDuration);
    }
    }
    
    // reset the button active status
    buttonActive = false;
    }
    }
  //////////////////////////////////////////////////////////////////////////
  
  now = time(nullptr);
  if (!client.connected())
  {
    checkWiFiThenMQTT();
    //checkWiFiThenMQTTNonBlocking();
    //checkWiFiThenReboot();
  }
  else
  {
    client.loop();
    if (millis() - lastMillis > 5000)
    {
      lastMillis = millis();
      sendData();
    }
    if (millis() - lastMillisTDS > 25000)
    {
      lastMillisTDS = millis();
      sendDataTDS();
    }
  }
  
}
