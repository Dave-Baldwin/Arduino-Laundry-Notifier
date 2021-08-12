/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
////// LAUNDRY FINISH NOTIFIER //////////////////////////
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
// INITIAL WORK 9/18/2020 and 9/19/2020 by DB
// 10/10/2020 add user ability to change vibration threshold via web server form..
// 10/11/2020 add new WiFi Web server library for simpler processing of web server requests, and processing/parsing of POST requests from the served web page
// 8/7/2021 replaced WiFi Web server with reporting via MQTT (more robust; web server had some stability issues)
// 8/11/2021 add Pushover notifications when comms from MQTT/OH go down; enhanced blinking behaviour for WD fail, as well as "shoulder" vibration behaviour -- i.e. vibrating before latching flag goes on, or not vibrating before latching flag falls off..
//      also made filtering more aggressive to help unlatch the vibration flag after laundry finishes, and raised vibration threshold slightly.

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

#include "defines.h"
#include <ArduinoHttpClient.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <Arduino_LSM6DS3.h>

// declaration for Arduino-hosted web server..
int reqCount = 0;                // number of requests received
long webMillis = 0;

const String s = "";

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int statusWiFi = WL_IDLE_STATUS;     // the Wifi radio's status
int reconnectCounter = 0;
char hostname[] = "ardu-laundryvib";

// Pushover settings
char pushoversite[] = "api.pushover.net";
char apitoken[] = SECRET_PUSHOVER_APITOKEN;
char userkey[] = SECRET_PUSHOVER_USERKEY;
int pushLength;

// MQTT definitions
const char* mqttServer = SECRET_MQTTSERVERIP;
int intRetryWifi = 0;
boolean reconnectTried = false;
long millisWifiReconnectTime = 30000;
long millisSinceWifiReconnect = 0;
boolean firstRun = true;
long millisMQTTUpdate = 6000;
long millisLastMQTTUpdate = 0;

int port = 80;
String tsLastNotif = "------";
String currTime;

// declarations for Arduino web client to talk to Pushover web server online, and MQTT server locally
WiFiClient wifi;
PubSubClient mqttClient(wifi);
// char array for messages send via MQTT
char copy[45];
//HttpClient client = HttpClient(wifi, serverAddress, port);

// Variables in regard to timing
const unsigned long sendInterval = 10000; // 1 minute minimum between notifications
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;

// define variables for determining if we are still vibrating
long vibMillis = 0;
long stillMillis = 0;
boolean vibFlag = false;    // assume we are NOT vibrating when the arduino initially boots up .. this should reduce nuisance notifications.
boolean vibFlagLast = false;  // create memory variable
boolean cycleStarted = false;   // assume at boot that laundry has never started; once it does, we'll start counting time since started/ended

// define acceleration variables..
float x, y, z;
// define *previous* accel variables
float xP = 0;
float yP = 0;
float zP = 0;

float xDA, yDA, zDA, xDAL, yDAL, zDAL, xDAF, yDAF, zDAF, xDAhF, yDAhF, zDAhF;
float xFilt, yFilt, zFilt;
float filtConst = 0.10;
float heavyFiltConst = 0.01;
float accComp, accCompHeavyFilt;
// define the 'still threshold' for the straight mean/average 'grouped' acceleration - determined empirically by shaking arduino a bit, and then making it still.
float stillThresh = 0.0021;   // 9/21/2020 - change by DB from 0.0012 to 0.0017 to try and reject nuisance vibration detections from furnace (in MarshApal).
                              // 9/21/2020 - change by DB from 0.0017 to 0.0021 to keeping trying to reject nuisance vibration detections from furnace (in MarshApal).
                              // 8/11/2021 - change by DB from 0.0021 to 0.0026 to try to improve ability to settle back into "non vibrating mode" after laundry ends - threshold may have been just a bit high and filtering not quite tight enough that little jolts were keeping the vibration flag true

// define interval and timer for vibration checks
long intervIMU = 25;
long millisIMU = 0;
long millisDelayStill = 60000;
long millisDelayVib = 30000;
long millisSinceVibStart = 0;
long millisSinceVibEnd = 0;

// define interval and timer for LED blinkers
long intervLED = 5000;
long millisLED = 0;
// constant used here to set a pin number:
const int ledPin =  LED_BUILTIN;// the number of the LED pin
// Variable will change:
int ledState = LOW;             // ledState used to set the LED

const long millisMQTTWatchdogTimer = 65000;
long millisLastMQTTWatchdogUpdate = 0;
boolean blnWDAlarm = false;
boolean blnWDAlarmLast = false;
int wdCounter = 1;
int recvdWD = 0;
int uptimeCounter = 0;
int laundryMinsSinceStarted = 0;
int laundryMinsSinceEnded = 0;


void setup() {
  
  // set LED pin on constant while setting up..
  // set LED pin as output
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  
  //Initialize serial and wait for port to open:
  Serial.begin(9600);

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }
  
  // MQTT setup
  mqttClient.setServer(mqttServer, 1883);
  mqttClient.setCallback(callback);

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  Serial.println("Initializing inertial measurement unit..");
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  
  // turn LED pin off to signify done setting up..
  digitalWrite(ledPin, LOW);

}

// MAIN LOOP
void loop() {
    
  // check timer to see if we should check IMU and determine vibration yet..  
  if (millis() - millisIMU > intervIMU) {
    // reset IMU timer
    millisIMU = millis();
    
    // if acceleration is available, then print out the variables..
    if (IMU.accelerationAvailable()) {
    
      // read acceleration
      IMU.readAcceleration(x, y, z);
      
      // calculate abs difference in acceleration since last cycle
      xDA = abs(x - xP);
      yDA = abs(y - yP);
      zDA = abs(z - zP);
      
      // calculate filtered values of delta accelerations    
      xDAF = (1-filtConst)*xDAF + (filtConst)*xDA;
      yDAF = (1-filtConst)*yDAF + (filtConst)*yDA; 
      zDAF = (1-filtConst)*zDAF + (filtConst)*zDA;

      // calculate heavily filtered values of delta accelerations
      xDAhF = (1-heavyFiltConst)*xDAhF + (heavyFiltConst)*xDA;
      yDAhF = (1-heavyFiltConst)*yDAhF + (heavyFiltConst)*yDA; 
      zDAhF = (1-heavyFiltConst)*zDAhF + (heavyFiltConst)*zDA;

      // calculate an attempt at a "composite" differential acceleration
      accComp = (xDAF + yDAF + zDAF) / 3;
      accCompHeavyFilt = (xDAhF + yDAhF + zDAhF)/3;

      // figure out if filtered composite acceleration is more than 'still threshold' - if so, reset the vibMillis variable 
      if (accCompHeavyFilt > stillThresh) {
        vibMillis = millis();
        if(!vibFlag) {intervLED = 1000;}  // if vibration flag hasn't latched on yet, but we are currently above the vibration threshold, then quicken the blink pace
        else {intervLED = 100;}
      } else {
        // if not, set the still millis tracker
        stillMillis = millis();
        if(vibFlag) {intervLED = 300;}  // if vibration flag is still latched and hasnt' fallen off yet, but we are currently BELOW the vibration threshold, then slow down the blink pace
        else {intervLED = 3000;}        
      }
      
      // based on delay timers, set the vibration flag true or false
      if (vibFlag && (millis() - vibMillis > millisDelayStill)) {    // set time delay for 'stillness' to 60 seconds .. hopefully will reduce nuisance notifications
        vibFlag = false;
        millisMQTTUpdate = 6000;
        millisSinceVibEnd = millis();   // restart timer for "time since laundry vibration ended"
      }
      if (!vibFlag && (millis() - stillMillis > millisDelayVib)) {   // set time delay for vibration to 30 seconds - this will try to account for people unloading the dryer or washer..
        vibFlag = true;
        cycleStarted = true;
        millisMQTTUpdate = 6000;
        millisSinceVibStart = millis();   // restart timer for "time since laundry vibration started"
      }
      
      // based on vibration flag, constantly reset one of the timers counting time since laundry started or finished
      if(vibFlag) {
        millisSinceVibEnd = millis();     // restart timer for "time since laundry vibration ended" -- since it hasnt' ended yet
      } else {
        millisSinceVibStart = millis();   // restart timer for "time since laundry vibration started"  -- since it hasn't started yet
        if(!cycleStarted) {
          millisSinceVibEnd = millis();     // restart timer for "time since laundry vibration ended" -- laundry hasn't started since arduino booted or last reset, so laundry hasnt' officially ended!
        }
      }
      
      /*  BLOCK COMMENT FOR DEBUG
      if(true) {
        Serial.print("Vib flag, accComp, accCompHvyFlt, millis, stillSecs, vibSecs: ");
        Serial.print(vibFlag);
        Serial.print(" | ");
        Serial.print(accComp, 5);
        Serial.print(" | ");
        Serial.print(accCompHeavyFilt, 5);
        Serial.print(" | ");
        Serial.print((int)(millis()/1000));
        Serial.print(" | ");
        Serial.print((int)(stillMillis/1000));
        Serial.print(" | ");
        Serial.println((int)(vibMillis/1000));
      }
      */
      
      // print out current and filtered acceleration
      //Serial.print(xDAF, 4);
      //Serial.print(',');
      //Serial.print(yDAF, 4);
      //Serial.print(',');
      //Serial.print(zDAF, 4);
      //Serial.print(',');        
      //Serial.print(accComp, 4);   
      //Serial.print(',');        
      //if (!vibFlag) { Serial.print(floor((millis()-stillMillis)/1000),0); }   // if not vibrating, show seconds since we were last still?
      //if (vibFlag) { Serial.print(floor((millis()-vibMillis)/1000),0); }   // if vibrating, show seconds since we last vibrated?
      //if (!vibFlag) { Serial.print(",still"); }   // if not vibrating, add a notation at the end of the Serial debug line..
      //Serial.println();        
      
      // set 'previous' values equal to current values
      xP = x;
      yP = y;
      zP = z;
      xDAL = xDA;
      yDAL = yDA;
      zDAL = zDA;
    } // end if IMU is available.
    
  } // end if IMU check time is due

  // check timer to see if we should be blinking LED yet.. 
  if (millis() - millisLED > intervLED) {
    // reset LED timer
    millisLED = millis();
    // if we have a comm watchdog fail, then turn the LED on full constantly.
    if(blnWDAlarm) {
      ledState = HIGH; 
    } else {    // end if comm failure // start if comms are normal
      // if the LED is off turn it on and vice-versa:
      if (ledState == LOW) {
        ledState = HIGH;
      } else {
        ledState = LOW;
      }  
    } // end if comms are normal
    
    // change LED pin state accordingly..
    digitalWrite(ledPin, ledState);
  } // end check/change of LED state
  
  // if we're not connected to Wifi and enough time has passed, then try to reconnect
  if((WiFi.status() != WL_CONNECTED && (millis() - millisSinceWifiReconnect > millisWifiReconnectTime)) || firstRun) {
    wifiReconnect();
    mqttReconnect();
    
    // reset delay timer
    millisSinceWifiReconnect = millis();
  }
  
   // is the MQTT watchdog timer fresh?
  if(millis() - millisLastMQTTWatchdogUpdate > millisMQTTWatchdogTimer) {
    blnWDAlarm = true;
    // if watchdog is failed and the node has been running for a bit, then restart - cnxn to MQTT broker seems to have failed and we need to restart?
    //if (millis() > 60000) { ESP.restart(); }
  } else {
    blnWDAlarm = false;
  }
  
  // if this is the cycle where we have a watchdog failure event, notify
  if(blnWDAlarm && !blnWDAlarmLast) {
    pushover("WDT Fail: OH/MQTT watchdog communication to Laundry Nano.");
  } else if (!blnWDAlarm && blnWDAlarmLast) {
    pushover("WDT OK: OH/MQTT watchdog communication to Laundry Nano.");
  }
  
  // update memory variable for watchdog fail flag *from MQTT*
  blnWDAlarmLast = blnWDAlarm;
  
  // is it time to report via MQTT?
  if(millis() - millisLastMQTTUpdate > millisMQTTUpdate) {

    // measure uptime
    uptimeCounter = (int)(millis()/1000/60);    // integer value in minutes
    // measure time (minutes) since laundry vibration started/ended
    laundryMinsSinceStarted = (int)((millis()-millisSinceVibStart)/1000/60);    // integer value in minutes
    laundryMinsSinceEnded = (int)((millis()-millisSinceVibEnd)/1000/60);    // integer value in minutes
    
    // construct char array for notification message
    String str = String((int)vibFlag);
    str.toCharArray(copy, 45);
    mqttClient.publish("Liberty/Laundry/Vibrating", copy);
    
    str = String(accCompHeavyFilt, 5);    // need lots of decimal places for precision..
    str.toCharArray(copy, 45);
    mqttClient.publish("Liberty/Laundry/VibrationVal", copy);
 
    str = String(laundryMinsSinceStarted);
    str.toCharArray(copy, 45);
    mqttClient.publish("Liberty/Laundry/MinsSinceLaundryStarted", copy);
    
    str = String(laundryMinsSinceEnded);
    str.toCharArray(copy, 45);
    mqttClient.publish("Liberty/Laundry/MinsSinceLaundryEnded", copy);    
    
    str = String(s+uptimeCounter);
    str.toCharArray(copy, 45);
    mqttClient.publish("Liberty/Laundry/LaundryNano-UptimeMins", copy);
    
    str = String(s+wdCounter);
    str.toCharArray(copy, 45);
    mqttClient.publish("Liberty/Laundry/LaundryNano-MQTT-WDT", copy);

    wdCounter++;
    if(wdCounter > 100) {
      wdCounter = 1;
    }
    
    // update MQTT update delay timer..
    millisLastMQTTUpdate = millis();
  }  
  
  // MQTT maintenance
  if (!mqttClient.connected()) { mqttReconnect(); }
  mqttClient.loop();  
  
  // set firstRun variable false
  firstRun = false;  
  
  // if this is a cycle where the vibration flag has turned off, then send a notification
  if (!vibFlag && vibFlagLast) {
    pushover("Laundry has finished.");
  }
  
  // update vibration flag memory variable
  vibFlagLast = vibFlag;

}   // end main loop..

/*
boolean isNumeric(String str) {
    unsigned int stringLength = str.length();
 
    if (stringLength == 0) {
        return false;
    }
 
    boolean seenDecimal = false;
 
    for(unsigned int i = 0; i < stringLength; ++i) {
        if (isDigit(str.charAt(i))) {
            continue;
        }
 
        if (str.charAt(i) == '.') {
            if (seenDecimal) {
                return false;
            }
            seenDecimal = true;
            continue;
        }
        return false;
    }
    return true;
}
*/

void mqttReconnect() {
  if(!mqttClient.connected()) 
  {
    // Connect to MQTT
    Serial.print(s+"Connecting to MQTT: "+mqttServer+" ... ");
    //M5.Lcd.print("Connecting to MQTT:");
    if (mqttClient.connect("LaundryNanoClient")) {
      Serial.println("connected");
      mqttClient.subscribe("Liberty/Laundry/MQTT-LaundryNano-WDT");
      mqttClient.subscribe("Liberty/Laundry/ResetCycle");
    } else {
      Serial.println(s+"failed, rc="+mqttClient.state());
      //M5.Lcd.print("0");
    }
  }
}

void wifiReconnect() {
    // Connect to WiFi
  Serial.print(s+"Connecting to WiFi: "+ssid+" ");
  
  // stop wifi ..
  WiFi.end(); 
  delay(500);     // wait a short while..
  
  WiFi.begin(ssid, pass);
  if (WiFi.status() != WL_CONNECTED) {
    intRetryWifi++;
    //delay(500);
    //Serial.print(".");
    //M5.Lcd.print(".");

    // if too many tries, then try to reconnect /// then reboot..
    if(intRetryWifi == 20 && !reconnectTried) {
      //WiFi.reconnect();
      reconnectTried = true;
      //M5.Lcd.println("Trying to connect again.");
    } 
    else if (intRetryWifi > 40 && reconnectTried) {
      Serial.println("WiFi died, should restart?");
      //ESP.restart();
    }
  } else {
    intRetryWifi = 0;
    Serial.println("wifi connected");

    /*  the WIFI status codes are:
     * That means
     * WL_NO_SHIELD = 255,
     * WL_IDLE_STATUS = 0,
     * WL_NO_SSID_AVAIL = 1
     * WL_SCAN_COMPLETED = 2
     * WL_CONNECTED = 3
     * WL_CONNECT_FAILED = 4
     * WL_CONNECTION_LOST = 5
     * WL_DISCONNECTED = 6
     */
  
    Serial.println("");
    Serial.print("WiFi connected @ IP address: ");
    Serial.println(WiFi.localIP());
  } // end checking if wifi connected
  
}

byte pushover(char *pushovermessage)    // function to send pushover notifications
{
  String message = pushovermessage;

  pushLength = 81 + message.length();

  if(wifi.connect(pushoversite,80))
  {
    wifi.println("POST /1/messages.json HTTP/1.1");
    wifi.println("Host: api.pushover.net");
    wifi.println("Connection: close\r\nContent-Type: application/x-www-form-urlencoded");
    wifi.print("Content-Length: ");
    wifi.print(pushLength);
    wifi.println("\r\n");;
    wifi.print("token=");
    wifi.print(apitoken);
    wifi.print("&user=");
    wifi.print(userkey);
    wifi.print("&message=");
    wifi.print(message);
    while(wifi.connected())  
    {
      while(wifi.available())
      {
        char ch = wifi.read();
        Serial.write(ch);
      }
    }
    wifi.stop();  // don't think we need this, but will leave it in for now..
  }  
}

void callback(char* topic, byte* payload, unsigned int length) 
{
  //Serial.print("Message arrived [");
  //Serial.print(topic);
  //Serial.print("] ");
  //for (int i = 0; i < length; i++) 
  //{
  //  Serial.print((char)payload[i]);
  //}
  //Serial.println();

  if(String(topic) == "Liberty/Laundry/MQTT-LaundryNano-WDT") {
    //Serial.print("Watchdog reset from OH/MQTT: ");
    //for (int i = 0; i < length; i++) 
    //{
    //  Serial.print((char)payload[i]);
    //}
    //Serial.println();
    millisLastMQTTWatchdogUpdate = millis();
  }
  
  if(String(topic) == "Liberty/Laundry/ResetCycle") {
    // check first character
    if((char)payload[0] == '1') {
      cycleStarted = false; 
      mqttClient.publish("Liberty/Laundry/ResetCycle", "0");  // reset the value
      Serial.print("Laundry cycle reset from OH/MQTT: ");
      for (int i = 0; i < length; i++) 
      {
        Serial.print((char)payload[i]);
      }
      Serial.println();
    } // end if first character was 1..
  }  

}