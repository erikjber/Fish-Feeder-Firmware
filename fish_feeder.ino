#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <Servo.h>
#include <NTPClient.h>
#include "RTClib.h"
// define SECRET_SSID and SECRET_PASS in wifi_secrets.h and place it in the same directory as this file.
#include "wifi_secrets.h"

#define MAX_WIFI_RETRIES 4 // How many times to attempt to reconnect to the network before giving up
#define MANUAL_BUTTON_FEED_TIME 300 // How long, in milliseconds, to run the servo when the button is pressed
#define CLIENT_TIMEOUT_MILLIS 100 // The maximum amount of time we allow a client to be connected
#define MULTICAST_PORT 5050 // The port clients will be listening for a multicast on

IPAddress multicastAddress(226, 1, 1, 1);
unsigned int localPort = 2390;  // local port to listen on

char ssid[] = SECRET_SSID; // your network SSID (name)
char pass[] = SECRET_PASS; // your network password (use for WPA, or use as key for WEP)
WiFiUDP Udp;
NTPClient timeClient(Udp);
WiFiServer server(localPort);
RTC_DS1307 rtc;
Servo servo;

int servoPin = 3; // The Arduino pin the servo is connected to
int buttonPin = 6; // The Arduino pin the momentary switch is connected to
int debounce_time = 30;
int buttonState;             // the current reading from the input pin
int lastButtonState = HIGH;   // the previous reading from the input pin
unsigned long lastDebounceTime;
unsigned long lastBeacon;
unsigned long lastRtcCheck;
unsigned long lastConnectionTime;
int lastSecond; // The seconds portion of the time the last time we checked the RTC for feeding times.
bool timeIsDirty = false; // If this gets set to true, time will be synced with NTP at the next opportunity
bool servoIsRunning = false;
unsigned long servoTimeout = 0;
unsigned long servoStartedAt = 0;


void setup() {
  Serial.begin(1000000);
  while (!Serial)
  {
    delay(100);
    // Only wait for small time, we don't hang if we have power but no serial
    if (millis() > 5000)
    {
      break;
    }
  }
  Serial.println("Starting FishFeeder 3000!");

  pinMode(buttonPin, INPUT_PULLUP);
  lastDebounceTime = millis();
  lastRtcCheck = millis();
  lastBeacon = millis();

  setupRTC();
  connectToWifi();
  printWifiStatus();
  timeClient.begin();

  // The first time you run this code do this:
  // 1. uncomment the following code block
  // 2. upload and run the code
  // 3. comment out the code block
  // 4. upload the code again
  //for(int x = 0;x<18;x++)
  //{
  //  eraseFeedingTime(x);
  //}

  // Start networking servers
  server.begin();
  Udp.begin(localPort);
}


/**
 * Program main loop.
 */
void loop() {
  checkButton();
  checkRTC();
  handleServer();
  checkServoTimeout();
}

void setupRTC()
{
  if (! rtc.begin()) {
    while (1)
    {
      Serial.println("Couldn't find RTC");
      delay(1000);
    }
  }
  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running!");
    timeIsDirty = true;
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // September 17, 2022 at 15:00 you would call:
    //rtc.adjust(DateTime(2022, 9, 17, 15, 0, 0));
  }
}

/**
   Erase one of the 18 feeding times.
*/
void eraseFeedingTime(int time_slot)
{
  if (time_slot >= 0 && time_slot < 18)
  {
    int addr = time_slot * 3;
    uint8_t data[3] = {0xFF, 0xFF, 0xFF};
    rtc.writenvram(addr, data, 3);
  }
}

/**
 * Store a new feeding time.
 * This function does not check if a feeding time already exists.
 */
void writeFeedingTime(int time_slot, uint8_t hour, uint8_t minute, uint8_t deciseconds)
{
  if (time_slot >= 0 && time_slot < 18
      && hour >= 0 && hour < 24
      && minute >= 0 && minute < 60
      && deciseconds > 0 && deciseconds < 256)
  {
    int addr = time_slot * 3;
    uint8_t data[3] = {hour, minute, deciseconds};
    rtc.writenvram(addr, data, 3);
  }
}

/**
 * Read the feeding time for a given slot.
 */
void readFeedingTime(int time_slot, uint8_t &hour, uint8_t &minute, uint8_t &deciseconds)
{
  if (time_slot >= 0 && time_slot < 18)
  {
    int addr = time_slot * 3;
    uint8_t data[3] = {0};
    rtc.readnvram(data, 3, addr);
    hour = data[0];
    minute = data[1];
    deciseconds = data[2];
  }
}

/**
 * Read all feeding times and send them to the client.
 */
void sendData(WiFiClient client)
{
  uint8_t hour,minute,deciseconds;
  for(int x=0;x<18;x++)
  {
    readFeedingTime(x,hour,minute,deciseconds);
    client.write(hour);
    client.write(minute);
    client.write(deciseconds);
  }
}


void connectToWifi()
{
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    // don't continue
    while (true)
    {
      Serial.println("Communication with WiFi module failed!");
      delay(1000);
    }
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to Wifi network:
  for (int x = 0; x < MAX_WIFI_RETRIES; x++)
  {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    WiFi.begin(ssid, pass);
    if (WiFi.status() == WL_CONNECTED)
    {
      Serial.println();
      Serial.print("Connected to ");
      Serial.println(ssid);
      lastConnectionTime = millis();
      break;
    }
    else
    {
      delay(3000);
    }
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Could not connect to WiFi");
  }
}

/**
 * Check if the servo has been running for so long that it should be stopped.
 */
void checkServoTimeout()
{
  if(servoIsRunning)
  {
    unsigned long now = millis();
    if (servoTimeout >= servoStartedAt)
    {
      // Normal case
      if(now >= servoTimeout || now < servoStartedAt)
      {
        stopServo();
      }
    }
    else
    {
      // millis() have rolled over/will roll over during the time the servo runs
      if(now >= servoTimeout && now < servoStartedAt)
      {
        stopServo();
      }
    }
  }
}

/**
 * Get the time from the RTC and check if it's time to start the servo.
 */
void checkRTC()
{
  long diff = millis() - lastRtcCheck;
  if (diff >= 10000 || diff < 0)
  {
    lastRtcCheck = millis();
    // Check if it's a new minute
    DateTime now = rtc.now();
    if (now.second() < lastSecond)
    {
      // Read the RTC values, check if they match the current hour and minute
      uint8_t hour = 0;
      uint8_t minute = 0;
      uint8_t deciseconds = 0;
      for (int x = 0; x < 18; x++)
      {
        readFeedingTime(x, hour, minute, deciseconds);
        if (hour == now.hour() && minute == now.minute())
        {
          long milliseconds = deciseconds * 100;
          startServo(milliseconds);
          break; // Each time slot only used once
        }
      }
      syncWithNtpTime();
    }
    lastSecond = now.second();
  }
  if(timeIsDirty)
  {
    syncWithNtpTime();
  }
}

/**
 * Synchronize the RTC with NTP, if necessary. 
 */
void syncWithNtpTime()
{
  DateTime now = rtc.now();
  if ( now.minute() == 0 || timeIsDirty)
  {
    // Download the latest time from an NTP server on the hour
    timeClient.update();
    if (timeClient.isTimeSet())
    {
      DateTime ntpTime = DateTime(timeClient.getEpochTime());
      serialPrintTime();
      Serial.print("NTP epoch time: ");
      Serial.println(timeClient.getEpochTime());
      Serial.print("NTP time: ");
      char buffer[40];
      sprintf(buffer, "YYYY-MM-DD hh:mm:ss");
      Serial.println(ntpTime.toString(buffer));
      if (timeClient.getEpochTime() != rtc.now().unixtime())
      {
        Serial.println("Times differ, updating RTC.");
        rtc.adjust(ntpTime);
        serialPrintTime();
      }
      else
      {
        Serial.println("NTP and RTC times agree, doing nothing.");
      }
      timeIsDirty = false;
    }
    else
    {
      Serial.println("NTP time not set.");
    }
  }
}

/**
 * Print the current RTC time to 
 */
void serialPrintTime()
{
  char buffer[40];

  DateTime now = rtc.now();

  Serial.print("RTC time: ");
  sprintf(buffer, "%04d", now.year());
  Serial.print(buffer);
  Serial.print('-');
  sprintf(buffer, "%02d", now.month());
  Serial.print(buffer);
  Serial.print('-');
  sprintf(buffer, "%02d", now.day());
  Serial.print(buffer);
  Serial.print(" ");
  sprintf(buffer, "%02d", now.hour());
  Serial.print(buffer);
  Serial.print(':');
  sprintf(buffer, "%02d", now.minute());
  Serial.print(buffer);
  Serial.print(':');
  sprintf(buffer, "%02d", now.second());
  Serial.print(buffer);
  Serial.println();
  Serial.print("RTC epoch time: ");
  Serial.print(now.unixtime());
  Serial.println();

}

/**
 * Check if any clients have connect and handle their requests.
 */
void handleServer()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    lastConnectionTime = millis();
    long diff = millis() - lastBeacon;
    if (diff > 1000 || diff < 0)
    {
      lastBeacon = millis();
      Udp.beginPacket(multicastAddress, MULTICAST_PORT);
      char data[40];
      sprintf(data, "%d", localPort);
      Udp.write(data);
      Udp.endPacket();
    }
    
    //  Handle connections from clients
    WiFiClient client = server.available();
    if (client)
    {
      // an http request ends with a blank line
      boolean currentLineIsBlank = true;
      int bytenumber = 0;
      uint8_t recv_data[5];
      uint8_t data;
      long client_start_connect = millis();
      while (client.connected())
      {
        if (client.available())
        {
          data = client.read();
          recv_data[bytenumber] = data;
          bytenumber++;
        }
        long diff = millis()-client_start_connect;
        if(diff > CLIENT_TIMEOUT_MILLIS || diff < 0)
        {
          // Time out 
          Serial.println("Client timeout.");
          break;
        }

        // Process the data
        if(recv_data[0] == 'u' && bytenumber == 1)
        {
          // Update
          sendData(client);
          break;
        }
        else if(recv_data[0] == 'c' && bytenumber == 5)
        {
          // Create
          writeFeedingTime(recv_data[1],recv_data[2],recv_data[3],recv_data[4]);
          sendData(client);
          break;
        }
        else if(recv_data[0] == 'd' && bytenumber == 2)
        {
          eraseFeedingTime(recv_data[1]);
          sendData(client);
          break;
        }
        else if(recv_data[0] == 'm' && bytenumber == 2)
        {
          long ms = recv_data[1]*100;
          Serial.print("Manual running for ");
          Serial.print(ms);
          Serial.println(" ms.");
          startServo(ms);
          break;
        }
        else if(bytenumber > 5)
        {
          // Client sent nonsense request, goodbye
          break;
        }
      }
      delay(1);
      client.stop();
    }
  }
  else
  {
    // Have we been down for more then a minute? Try to reconnect
    long diff = millis() - lastConnectionTime;
    if (diff > 60000 || diff < 0)
    {
      connectToWifi();
      lastConnectionTime = millis();
    }
  }
}

/**
 * Check if the button has been pressed.
 * Includes debouncing.
 */
void checkButton()
{
  int reading = digitalRead(buttonPin);

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounce_time) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == LOW) {
        Serial.println("Button pushed!");
        startServo(MANUAL_BUTTON_FEED_TIME);
      }
    }
  }

  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState = reading;
}

/**
 * Start servo, indicating for how long it should run.
 * This method returns immediately, it does not wait for the servo to finish running.
 */
void startServo(unsigned long timeToRun)
{
  if(!servoIsRunning)
  {
    Serial.println("Starting servo...");
    servoIsRunning = true;
    servo.attach(servoPin);
    unsigned long spentTime = 0;
    for(int x = 89;x>=85;x--)
    {
      servo.write(x);
      delay(20);
      spentTime+=20;
    }
    servo.write(0);
    servoStartedAt = millis();
    if(spentTime>=timeToRun)
    {
      timeToRun = 0;
    }
    else
    {
      timeToRun -= spentTime;
    }
    servoTimeout = servoStartedAt+timeToRun;
    Serial.println("Servo started.");
  }
}

void stopServo()
{
  if(servoIsRunning)
  {
    Serial.println("Stopping servo...");
    for(int x = 85;x<=90;x++)
    {
      servo.write(x);
      delay(20);
    }
    servo.detach();
    servoIsRunning = false;
    Serial.println("Servo stopped.");
  }
}

void printWifiStatus() 
{
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
