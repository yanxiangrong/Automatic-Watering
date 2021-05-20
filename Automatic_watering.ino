#include <DHT.h>
#include <math.h>
#include <DHT_U.h>
#include <rpcWiFi.h>
#include <TFT_eSPI.h>
#include <DateTime.h>
#include <RTC_SAMD51.h>
#include <WiFiClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SleepyDog.h>
#include <BlynkSimpleWioTerminal.h>
#include "Free_Fonts.h"


#define DHTPIN D2     // Digital pin connected to the DHT sensor 
#define DHTTYPE    DHT11     // DHT 11
#define BLYNK_PRINT Serial


char auth[] = "OKRox8fOgUijBjBaHPCXlLreyrxEnMZ1";
char server[] = "yandage.top";
int port = 8080;


// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "IOT_2518";
char pass[] = "wlwsys6688";

//char ssid[] = "HUAWEI_E5576_Yan";
//char pass[] = "88668866";

//char ssid[] = "vivo Z5x";
//char pass[] = "12348765";

RTC_SAMD51 rtc;
WiFiUDP udp;

const int NTP_PACKET_SIZE = 48;
char timeServer[] = "ntp.aliyun.com";
byte packetBuffer[NTP_PACKET_SIZE];
unsigned int localPort = 2390;

BlynkTimer timer;
TFT_eSPI tft = TFT_eSPI();
DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;
int setWet = 0;
int lastWater = 0xFFFF;
bool manualMode = false;
bool rgbOn = false;


BLYNK_WRITE(V8) { //setWet
  int value = param.asInt();
  setWet = value;
}

BLYNK_WRITE(V9) { //
  int value = param.asInt();
  if (value == 1)
  {
    manualMode = true;
    digitalWrite(D4, LOW);
  }
  else if (value == 0)
  {
    manualMode = false;
    digitalWrite(D4, HIGH);
  }
}

BLYNK_WRITE(V10) { //
  int value = param.asInt();
  if (value == 1)
  {
    rgbOn = true;
  }
  else if (value == 0)
  {
    rgbOn = false;
    analogWrite(D0, 0) ;
    analogWrite(D6, 0) ;
    analogWrite(D8, 0) ;
  }
}

void rgbChange() {
  if (!rgbOn) {
    return ;
  }
  static double i = 0;
  int r = sin(i) * 128 + 128;
  int g = sin(i + PI / 3) * 128 + 128;
  int b = sin(i + PI * 2 / 3) * 128 + 128;
  analogWrite(D0, r);
  analogWrite(D6, g);
  analogWrite(D8, b);
  i += 0.01;
}

void printV()
{
  int sensorValue = analogRead(A7);
  float voltage = sensorValue * (5.0 / 1023.0);
  //Serial.println(voltage);
  //tft.drawString( String(voltage) + "V", 60, 40);

  int wet = 100 - sensorValue * (100.0 / 1023.0);
  Blynk.virtualWrite(V7, wet); //将土壤湿度上传到服务器
  tft.drawString("Wetness: " + String(wet) + "%  ", 60, 70);
  //Serial.println("wetness =" + String(wet) + "  setWet= " + String(setWet));
  if (manualMode == false)
  {
    if ((wet < setWet) and ((millis() / 1000 - lastWater) > 600)) //浇水
    {
      digitalWrite(D4, LOW);
      lastWater = millis() / 1000;
      delay(1000);
      digitalWrite(D4, HIGH) ;
    }
//    if (wet >= setWet + 5) {
//      digitalWrite(D4, HIGH);
//    }
  }

  tft.drawString("Setting wet: " + String(setWet) + "%  ", 60, 90);
}

void light()
{
  int sensorValue = analogRead(A5);
  float light = sensorValue * (5.0 / 1023.0);
  //tft.drawString("light " + String(light) + "V", 60, 120);

  int x = 108.14 - light / (1.48 - 0.28 * light);
  Blynk.virtualWrite(V6, x); //将亮度上传到服务器
  tft.drawString("light: " + String(x) + "lux  ", 60, 150);
}


BLYNK_WRITE(V3) {
  int value = param.asInt();
  rgbOn = false;
  analogWrite(D0, value);
}

BLYNK_WRITE(V4) {
  int value = param.asInt();
  rgbOn = false;
  analogWrite(D6, value);
}

BLYNK_WRITE(V5) {
  int value = param.asInt();
  rgbOn = false;
  analogWrite(D8, value);
}


void sendTemprature() {

  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else {
    tft.drawString("Temperature: " + String(event.temperature, 1) + "C  ", 60, 110); //prints string at (30,80)
    Blynk.virtualWrite(V1, event.temperature); //将温度上传到服务器
  }

  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    tft.drawString("Humidity: " + String(event.relative_humidity, 0) + "%  ", 60, 130); //prints string at (30,100)
    Blynk.virtualWrite(V2, event.relative_humidity); //将湿度上传到服务器
  }


}

void water(void)
{

  if (digitalRead(D4) == LOW)
  {
    manualMode = false;
    digitalWrite(D4, HIGH);
  }
  else if (digitalRead(D4) == HIGH)
  {
    manualMode = true;
    digitalWrite(D4, LOW);
  }
}

void minsetWet(void)
{
  setWet--;
  if (setWet < 0)
  {
    setWet = 0;
  }
  tft.drawString("Setting wet: " + String(setWet) + "%  ", 60, 90);
}

void addsetWet(void)
{

  setWet++;
  if (setWet > 100)
  {
    setWet = 100;
  }
  tft.drawString("Setting wet: " + String(setWet) + "%  ", 60, 90);
}

void showtime() {
  static DateTime start_time = rtc.now();
  static unsigned long start_millis = millis();
  DateTime now = start_time + (millis() - start_millis) / 1000;

  char buf1[] = "hh:mm:ss";
  char buf2[] = "YY-MM-DD";
  tft.drawString(now.toString(buf2),228, 5);
  tft.drawString(now.toString(buf1),5, 5);
}

// send an NTP request to the time server at the given address
void sendNTPpacket(const char *address)
{
    // set all bytes in the buffer to 0
    for (int i = 0; i < NTP_PACKET_SIZE; ++i)
    {
        packetBuffer[i] = 0;
    }
    // Initialize values needed to form NTP request
    // (see URL above for details on the packets)
    packetBuffer[0] = 0b11100011; // LI, Version, Mode
    packetBuffer[1] = 0;          // Stratum, or type of clock
    packetBuffer[2] = 6;          // Polling Interval
    packetBuffer[3] = 0xEC;       // Peer Clock Precision
    // 8 bytes of zero for Root Delay & Root Dispersion
    packetBuffer[12] = 49;
    packetBuffer[13] = 0x4E;
    packetBuffer[14] = 49;
    packetBuffer[15] = 52;

    // all NTP fields have been given values, now
    // you can send a packet requesting a timestamp:
    udp.beginPacket(address, 123); //NTP requests are to port 123
    udp.write(packetBuffer, NTP_PACKET_SIZE);
    udp.endPacket();
}

unsigned long getNTPtime()
{

    // module returns a unsigned long time valus as secs since Jan 1, 1970
    // unix time or 0 if a problem encounted

    //only send data when connected
    if (WiFi.status() == WL_CONNECTED)
    {
        //initializes the UDP state
        //This initializes the transfer buffer
        udp.begin(WiFi.localIP(), localPort);

        sendNTPpacket(timeServer); // send an NTP packet to a time server
        // wait to see if a reply is available
        delay(1000);
        if (udp.parsePacket())
        {
            Serial.println("udp packet received");
            Serial.println("");
            // We've received a packet, read the data from it
            udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

            //the timestamp starts at byte 40 of the received packet and is four bytes,
            // or two words, long. First, extract the two words:

            unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
            unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
            // combine the four bytes (two words) into a long integer
            // this is NTP time (seconds since Jan 1 1900):
            unsigned long secsSince1900 = highWord << 16 | lowWord;
            // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
            const unsigned long seventyYears = 2208988800UL;
            // subtract seventy years:
            unsigned long epoch = secsSince1900 - seventyYears;

            // adjust time for timezone offset in secs +/- from UTC
            // WA time offset from UTC is +8 hours (28,800 secs)
            // + East of GMT
            // - West of GMT
            long tzOffset = 28800UL;

            // WA local time
            unsigned long adjustedTime;
            return adjustedTime = epoch + tzOffset;
        }
        else
        {
            // were not able to parse the udp packet successfully
            // clear down the udp connection
            udp.stop();
            return 0; // zero indicates a failure
        }
        // not calling ntp time frequently, stop releases resources
        udp.stop();
    }
    else
    {
        // network not connected
        return 0;
    }
}

int adjust_time()
{
    unsigned long devicetime = getNTPtime();
    if (devicetime == 0)
    {
        Serial.println("Failed to get time from network time server.");
        return -1;
    }
    else
    {
        if (DateTime(devicetime) != rtc.now())
        {
            rtc.adjust(DateTime(devicetime));
            Serial.println("RTC (boot) time updated.");
        }
        else
        {
            Serial.println("RTC (boot) time if right.");
        }
        return 0;
    }
}

void feedDog() {
  Watchdog.reset();
}

void setup() {
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK); //Black background
  tft.setFreeFont(FM9);
  tft.setCursor(0, 15);
//  tft.setTextColor(TFT_WHITE);
  tft.print("[" + String(millis()) + "]Starting Serial... ");

  Serial.begin(9600);
//  tft.setTextColor(TFT_GREEN);
  
  tft.println("OK");

//  tft.setTextColor(TFT_WHITE);
  tft.print("[" + String(millis()) + "]Starting GPIO... ");
  
  pinMode(A5, INPUT);  // 光照
  pinMode(A7, INPUT);  // 土壤湿度
  pinMode(D4, OUTPUT); // 水泵
  digitalWrite(D4, HIGH);
  pinMode(DHTPIN, INPUT); //
  pinMode(D0, OUTPUT);  // R
  digitalWrite(D0, LOW);
  pinMode(D6, OUTPUT);  // G
  digitalWrite(D6, LOW);
  pinMode(D8, OUTPUT);  // B
  digitalWrite(D8, LOW);
  pinMode(WIO_KEY_A, INPUT_PULLUP);
  pinMode(WIO_KEY_B, INPUT_PULLUP);
  pinMode(WIO_KEY_C, INPUT_PULLUP);

//  tft.setTextColor(TFT_GREEN);
  tft.println("OK");
//  tft.setTextColor(TFT_WHITE);
  tft.print("[" + String(millis()) + "]Setting Watchdog... ");

  Watchdog.enable(16000);

//  tft.setTextColor(TFT_GREEN);
  tft.println("OK");
//  tft.setTextColor(TFT_WHITE);
  tft.print("[" + String(millis()) + "]Starting DHT... ");

  dht.begin();

//  tft.setTextColor(TFT_GREEN);
  tft.println("OK");
//  tft.setTextColor(TFT_WHITE);
  tft.print("[" + String(millis()) + "]Connect to server... ");

  Blynk.begin(auth, ssid, pass, server, port);
  
//  tft.setTextColor(TFT_GREEN);
  tft.println("OK");
//  tft.setTextColor(TFT_WHITE);
  tft.print("[" + String(millis()) + "]Starting RTC... ");

  rtc.begin();

//  tft.setTextColor(TFT_GREEN);
  tft.println("OK");
//  tft.setTextColor(TFT_WHITE);
  tft.print("[" + String(millis()) + "]Getting time... ");
  
  adjust_time();

//  tft.setTextColor(TFT_GREEN);
  tft.println("OK");
//  tft.setTextColor(TFT_WHITE);
  tft.print("[" + String(millis()) + "]Syncing data... ");

  Blynk.syncAll();

//  tft.setTextColor(TFT_GREEN);
  tft.println("OK");
//  tft.setTextColor(TFT_WHITE);
  tft.print("[" + String(millis()) + "]Setting timer... ");


  timer.setInterval(5003L, sendTemprature);
  timer.setInterval(5007L, printV);
  timer.setInterval(5013L, light);
  timer.setInterval(1000L, showtime);
  timer.setInterval(1023L, feedDog);
  timer.setInterval(30L, rgbChange);

//  tft.setTextColor(TFT_GREEN);
  tft.println("OK");
//  tft.setTextColor(TFT_WHITE);
  tft.print("[" + String(millis()) + "]Setting Interrupt...");

  attachInterrupt(WIO_KEY_C, water, FALLING);      //外部中断
  attachInterrupt(WIO_KEY_B, addsetWet, FALLING);  //外部中断
  attachInterrupt(WIO_KEY_A, minsetWet, FALLING);  //外部中断

//  tft.setTextColor(TFT_GREEN);
  tft.println("OK");
//  tft.setTextColor(TFT_WHITE);
  tft.print("[" + String(millis()) + "]Done ");
  tft.setCursor(0, 0);
  feedDog();
  delay(2000);

  tft.fillScreen(tft.color565(0,192,192));
  tft.fillRect(0, 0, 320, 22, TFT_BLACK);
  tft.fillRect(0, 220, 320, 22, TFT_BLACK);
  tft.fillRoundRect(30, 45, 260, 150, 5, TFT_BLACK);

  sendTemprature();
  printV();
  light();
  tft.drawString(WiFi.SSID(), 315 - tft.textWidth(WiFi.SSID()), 223);
  tft.setCursor(5, 235);
  tft.print(WiFi.localIP());
  tft.setCursor(0, 0);
}


void loop()
{
  Blynk.run();
  timer.run();
}
