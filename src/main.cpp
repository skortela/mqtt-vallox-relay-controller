#include <Arduino.h>
#include "Ticker.h"


//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>         //https://github.com/tzapu/WiFiManager

#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#include <ArduinoJson.h>

#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_MCP23X17.h>


#include "eeprom_utils.h"
#include "vallox_driver.h"

const int KStatusUpdateInterval = 60000;  // 60 sec
const char* KAvailabilityTopic = "vallox/availability";
const char* KOnline = "online";
const char* KOffline = "offline";
const char* KMqttTopicCommandListen = "vallox/cmd/#";
const char* KStateTopic = "vallox/state"; // for status updates

const int KTickerIntervalWifiConfig = 200;
const int KTickerIntervalMQTTConnect = 1000;

#define MCP_PIN_B7 15
const uint8_t KResetPinB7 = MCP_PIN_B7;

#define ONE_WIRE_PIN D6
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_PIN);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

Adafruit_MCP23X17 m_mcp;

void blink();
void sendState(bool refreshTemperature = false);

// Magic number to check if eeprom config is valid
int KConfigValidationMagic = 0x12457899;

char m_mqtt_server[40];
int m_mqtt_port = 1883;
char m_mqtt_user[40];
char m_mqtt_passw[40];

//flag for saving data
bool m_shouldSaveConfig = false;

Ticker m_led_ticker;

WiFiClient m_espClient;
PubSubClient m_mqttClient(m_espClient);

ValloxDriver m_vallox;

unsigned long m_lastStatusUpdate;
unsigned long m_takkaTimerLastStarted = 0;
const int KTakkaTimerAutoOff = 5*60000; // 5 minutes

int m_extInputState = 99; // active LOW



// Connect pin #12 of the expander to Analog 5 (i2c clock)
// Connect pin #13 of the expander to Analog 4 (i2c data)
// Connect pins #15, 16 and 17 of the expander to ground (address selection)
// Connect pin #9 of the expander to 5V (power)
// Connect pin #10 of the expander to ground (common ground)
// Connect pin #18 through a ~10kohm resistor to 5V (reset pin, active low)

// Input #0 is on pin 21 so connect a button or switch from there to ground



void blink() {
    //toggle state
    int state = digitalRead(LED_BUILTIN);  // get the current state of GPIO1 pin
    digitalWrite(LED_BUILTIN, !state);     // set pin to the opposite state
}

void configModeCallback(WiFiManager*) {
    Serial.println("Entering config mode");
    m_led_ticker.attach_ms(KTickerIntervalWifiConfig, blink);
}
//callback notifying us of the need to save config
void saveConfigCallback () {
    Serial.println("Should save config");
    m_shouldSaveConfig = true;
}


void loadConfig() {
    int32_t magic = eeprom_read_int32(0);
    if (magic != KConfigValidationMagic)
        return; // no configs saved
    int address = 4;
    eeprom_read_string(address, m_mqtt_server, 40);
    address += strlen(m_mqtt_server) + 1;
    m_mqtt_port = eeprom_read_int32(address);
    address += 4;
    eeprom_read_string(address, m_mqtt_user, 40);
    address += strlen(m_mqtt_user) + 1;
    eeprom_read_string(address, m_mqtt_passw, 40);
    address += strlen(m_mqtt_passw) + 1;
}
void saveConfig() {
    eeprom_write_int32(0, KConfigValidationMagic);
    int address = 4;
    eeprom_write_string(address, m_mqtt_server);
    address += strlen(m_mqtt_server) + 1;
    eeprom_write_int32(address, m_mqtt_port);
    address += 4;
    eeprom_write_string(address, m_mqtt_user);
    address += strlen(m_mqtt_user) + 1;
    eeprom_write_string(address, m_mqtt_passw);
    address += strlen(m_mqtt_passw) + 1;
    EEPROM.commit();
}
void resetConfig() {
    // clear first 100 bytes. it's enought to clear first int32 val, but erasing user sensitive data is more secure aproach.
    for (int i=0; i< 100; i++) {
        EEPROM.write(i, 0);
    }
    EEPROM.commit();
}

void wifiSetup()
{
    loadConfig();

    char strPort[10];
    sprintf(strPort, "%d", m_mqtt_port);

    // The extra parameters to be configured (can be either global or just in the setup)
    // After connecting, parameter.getValue() will get you the configured value
    // id/name placeholder/prompt default length
    WiFiManagerParameter custom_mqtt_server("server", "mqtt server", m_mqtt_server, 40);
    WiFiManagerParameter custom_mqtt_port("port", "mqtt port", strPort, 6);
    WiFiManagerParameter custom_mqtt_user("username", "username", m_mqtt_user, 40);
    WiFiManagerParameter custom_mqtt_passw("password", "password", m_mqtt_passw, 40);

    //m_led_ticker.interval(1000);
    //m_led_ticker.start();

    WiFiManager wifiManager;

    //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
    wifiManager.setAPCallback(configModeCallback);
    //set config save notify callback
    wifiManager.setSaveConfigCallback(saveConfigCallback);

    if (WiFi.SSID() != "")
    {
        Serial.println("Saved SSID found, set timeout for config portal");
        wifiManager.setConfigPortalTimeout(300); // 5 minutes timeout
    }

    //add all your parameters here
    wifiManager.addParameter(&custom_mqtt_server);
    wifiManager.addParameter(&custom_mqtt_port);
    wifiManager.addParameter(&custom_mqtt_user);
    wifiManager.addParameter(&custom_mqtt_passw);

    if(!wifiManager.autoConnect("ValloxController")) {
        Serial.println("failed to connect and hit timeout");
        //reset and try again, or maybe put it to deep sleep
        ESP.reset();
        delay(1000);
    }

     //read updated parameters
    strcpy(m_mqtt_server, custom_mqtt_server.getValue());
    m_mqtt_port = atoi(custom_mqtt_port.getValue());
    strcpy(m_mqtt_user, custom_mqtt_user.getValue());
    strcpy(m_mqtt_passw, custom_mqtt_passw.getValue());

    if (m_shouldSaveConfig) {
        // Save custom params
        saveConfig();
        Serial.println("config saved");
    }

    Serial.println("Connected");
    Serial.println("local ip");
    Serial.println(WiFi.localIP());

    m_led_ticker.detach();
}

void mqtt_message_received(char* topic, byte* payload, unsigned int length)
{
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    /*for (int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();
    */
    // set null termination as it may be missing
    payload[length] = 0;

    const char* strPayload = (const char*) payload;

    Serial.println(strPayload);
    // Switch on the LED if an 1 was received as first character
    /*if ((char)payload[0] == '1') {
        digitalWrite(LED_BUILTIN, LOW);   // Turn the LED on (Note that LOW is the voltage level
        // but actually the LED is on; this is because
        // it is active low on the ESP-01)
    } else {
        digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED off by making the voltage HIGH
    }*/

    //m_mqttClient.publish("vallox/","");
    char* pos = strrchr(topic, '/');
    if (!pos)
        return;

    pos++;
    
    Serial.println(pos);
    if (strcmp(pos, "setSpeed") == 0) {
        Serial.println("setting speed");
        int speed = atoi(strPayload);
        Serial.println(speed);
        m_vallox.setSpeed(speed);
    }
    else if (strcmp(pos, "setSpeedIncoming") == 0) {
        Serial.println("setting speed incoming");
        int speed = atoi(strPayload);
        Serial.println(speed);
        m_vallox.setSpeedIncoming(speed);
    }
    else if (strcmp(pos, "setSpeedExhaust") == 0) {
        Serial.println("setting speed exhaust");
        int speed = atoi(strPayload);
        Serial.println(speed);
        m_vallox.setSpeedExhaust(speed);
    }
    else if (strcmp(pos, "setPower") == 0) {
        if (strcasecmp(strPayload, "ON") == 0) {
            m_vallox.setPower(true);
        }
        else if (strcasecmp(strPayload, "OFF") == 0) {
            m_vallox.setPower(false);
        }
        else {
            Serial.print("invalid payload: ");
            Serial.println(strPayload);
        }
    }
    else if (strcmp(pos, "setTakkatehostus") == 0) {
        if (strcasecmp(strPayload, "ON") == 0) {
            m_vallox.setTakkatehostus(true);
            m_takkaTimerLastStarted = millis();
        }
        else if (strcasecmp(strPayload, "OFF") == 0) {
            m_vallox.setTakkatehostus(false);
            m_takkaTimerLastStarted = 0;
        }
        else if (strcasecmp(strPayload, "TOGGLE") == 0) {
            m_vallox.setTakkatehostus(!m_vallox.hasTakkatehostus());
        }
        else {
            Serial.print("invalid payload: ");
            Serial.println(strPayload);
        }
    }
    else{
        Serial.print("invalid command: ");
        Serial.println(pos);
    }

    sendState();

}

char* addressToStr(DeviceAddress deviceAddress, char* pBuffer)
{
    sprintf(pBuffer, "0x");
    int pos = 2;
    for (uint8_t i = 0; i < 8; i++)
    {
        sprintf(pBuffer + pos, "%02x", deviceAddress[i]);
        pos += 2;
    }
    pBuffer[pos] = '\0';
    return pBuffer;
}
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

// rounds a number to 2 decimal places
// example: round(3.14159) -> 3.14
double round2(double value) {
   return (int)(value * 100 + 0.5) / 100.0;
}
// rounds a number to 1 decimal places
// example: round(3.14159) -> 3.1
double round1(double value) {
   return (int)(value * 10 + 0.5) / 10.0;
}

void sendState(bool refreshTemperature) {
    StaticJsonDocument<1500> doc;
    JsonObject root = doc.to<JsonObject>();

    const char* KStrOn = "ON";
    const char* KStrOff = "OFF";

    if ( m_vallox.hasPower())
        root["power"] = KStrOn;
    else
        root["power"] = KStrOff;
    
    root["speed"] = m_vallox.speed();
    root["speed_incoming"] = m_vallox.speedIncoming();
    root["speed_exhaust"] = m_vallox.speedExhaust();

    if (m_vallox.hasTakkatehostus())
        root["takkatehostus"] = KStrOn;
    else
        root["takkatehostus"] = KStrOff;

    if (m_extInputState == LOW)
        root["ext_input"] = KStrOn;
    else
        root["ext_input"] = KStrOff;


    uint8_t devCount = sensors.getDeviceCount();
    // locate devices on the bus
    //Serial.print("Found ");
    //Serial.print(devCount, DEC);
    //Serial.println(" devices.");

    if (devCount) {
        if (refreshTemperature)
            sensors.requestTemperatures();

        JsonObject sensorsObj = root.createNestedObject("sensors");

        // Loop all found addresses, do not rely on devCount as it is not refreshed.
        DeviceAddress addr;
        char addrStr[19];
        for (uint8_t i=0; sensors.getAddress(addr, i); i++) {
            float value = sensors.getTempC(addr);
            if (value != DEVICE_DISCONNECTED_C) {
                addressToStr(addr, addrStr);
                sensorsObj[addrStr] = round1(value);
            }
        }
    }
    
    char* jsonBuffer = new char[1500];

    serializeJson(root, jsonBuffer, 1500);
    Serial.println("sending state");
    Serial.println(jsonBuffer);

    m_mqttClient.publish(KStateTopic, jsonBuffer);
    delete jsonBuffer;

    // request temperatures after status send
    if (devCount && !refreshTemperature)
        sensors.requestTemperatures();
}

void mqtt_reconnect() {
    m_led_ticker.attach_ms(KTickerIntervalMQTTConnect, blink);
    // Loop until we're reconnected
    while (!m_mqttClient.connected()) {
        Serial.print("Attempting MQTT connection...");
        // Create a random client ID
        String clientId = "Vallox-";
        clientId += String(random(0xffff), HEX);
        // Attempt to connect


        if (m_mqttClient.connect(clientId.c_str(), m_mqtt_user, m_mqtt_passw, KAvailabilityTopic, MQTTQOS0, true, KOffline)) {
            Serial.println("connected");
            // Once connected, publish an announcement...
            m_mqttClient.publish(KAvailabilityTopic, KOnline, true);
            // ... and resubscribe
            m_mqttClient.subscribe(KMqttTopicCommandListen);

            sendState();
        } else {
            Serial.print("failed, rc=");
            Serial.print(m_mqttClient.state());
            Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
    m_led_ticker.detach();
}

void setup()
{
    Serial.begin(115200);
    Serial.println("\n\nStarted");

    EEPROM.begin(150);

    if (!m_mcp.begin_I2C()) {
        Serial.println("ERROR: Failed to connect MCP23X17");
        while (1);
    }
    m_vallox.begin(&m_mcp);

    m_vallox.setPower(true);
    

    m_lastStatusUpdate = 0;
    
    // initialize LED digital pin as an output.
    pinMode(LED_BUILTIN, OUTPUT);

    pinMode(D5, OUTPUT);
    digitalWrite(D5, LOW); // set A0 to LOW = ground
    pinMode(D3, INPUT_PULLUP); // Connect external switch between D3 and D5.


    m_mcp.pinMode(KResetPinB7, INPUT_PULLUP); // turn on a 100K pullup internally


    sensors.begin();
    sensors.setResolution(11);

    Serial.print("last used SSID: ");
    Serial.println(WiFi.SSID());

    wifiSetup();

    m_mqttClient.setServer(m_mqtt_server, m_mqtt_port);
    m_mqttClient.setCallback(mqtt_message_received);
    delay(1000);
    Serial.println("Started!");
    digitalWrite(D4, HIGH);
}

void checkResetButton()
{
    if (m_mcp.digitalRead(KResetPinB7) == LOW)
    {
        Serial.println("reset pressed");
        delay(50);
        unsigned long starttime = millis();
        while (m_mcp.digitalRead(KResetPinB7) == LOW)
        {
            delay(50);
            if ((unsigned long)millis() - starttime > 5000)
            {
                // 5 sec pressed, do reset settings and reboot
                Serial.println("Reset settings");
                Serial.flush();
                //turn off led
                digitalWrite(LED_BUILTIN, HIGH); // active LOW
                delay(2000);
                // turn on led
                digitalWrite(LED_BUILTIN, LOW);
                delay(100);
                
                resetConfig();

                WiFiManager wifiManager;
                wifiManager.resetSettings();
                
                ESP.reset();
                delay(1000);

            }
        }
        Serial.println("reset aborted");
    }
    /*else
    {
        Serial.println("no reset");
        delay(500);
    }*/
    
}

void loop()
{
    checkResetButton();

    if (!m_mqttClient.connected()) {
        mqtt_reconnect();
        digitalWrite(LED_BUILTIN, LOW); // Turn led on
    }
    m_mqttClient.loop();

    
    int switchState = digitalRead(D3); // active LOW

    if (switchState != m_extInputState) {

        // the following variables are unsigned longs because the time, measured in
        // milliseconds, will quickly become a bigger number than can be stored in an int.
        unsigned long lastDebounceTime = millis();  // the last time the output pin was toggled
        unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

        int lastState = switchState;
        while (true) {

            switchState = digitalRead(D3);

            // If the switch changed, due to noise or pressing:
            if (switchState != lastState) {
                // reset the debouncing timer
                lastDebounceTime = millis();
            }
            if ((millis() - lastDebounceTime) > debounceDelay) {
                // whatever the reading is at, it's been there for longer than the debounce
                // delay, so take it as the actual current state:

                break;
            }
        }

        Serial.print("switchState: ");
        Serial.println(switchState, DEC);
        // State changed
        if (switchState == HIGH) { // active low
            // External switch switched off
            m_vallox.setTakkatehostus(false);
        }
        else {
            // switched on
            m_vallox.setTakkatehostus(true);
        }
        m_extInputState = switchState;

        sendState();
        m_lastStatusUpdate = millis();
    }

    
    if (m_takkaTimerLastStarted != 0 && ((unsigned long) (millis() - m_takkaTimerLastStarted) > KTakkaTimerAutoOff)) {
        m_takkaTimerLastStarted = 0;
        if (m_extInputState == HIGH) // turn off takkatehostus if also no external input
            m_vallox.setTakkatehostus(false);
    }

    if ((unsigned long) (millis() - m_lastStatusUpdate) > KStatusUpdateInterval) {
        // send status update
        sendState(true);
        m_lastStatusUpdate = millis();
    }
}
