// Use ESP8266 Core V2.74 OR esp32 2.0.11 by Espressif
// Use FastLED V3.30, WiFiManager 2.0.15-rc.1
// Also installed are Arduino Uno WiFi Dev Ed Library 0.0.3, Adafruit TSL2561 1.1.0
// RED WIRE to 3.3v
// BLUE WIRE to GND
// GREEN (or WHTIE) WIRE to D2 

// Boards supported are:
// ESP8266 -> NodeMCU 1.0(ESP-12E Module)
// and
// Board ESP32-WROOM-DA
// Have used pin D5 on each for the LEDs

// No TESTING OF LIGHT SENSORS HAS BEEN DONE.

// In my experience, a level shifter is needed for the data signal for the LEDs.

// To get a look at the XML output directly from a browser, edit the end of next line for an airport or airport list.
// It's not identical to what the code receives, but can be helpful.
// https://aviationweather.gov/api/data/metar?format=xml&hoursBeforeNow=3&mostRecentForEachStation=true&ids=KTME,KSGR

/*
January 24, 2024
----------------
Added support for ESP32-WROOM-DA.  Can now compile for either that board or the ESP8266.
1.  Added defines at top of code specific to each board.
2.  Added define for each board around call to set mode G on wifi interface.
No other code changes needed to support the ESP32-WROOM_DA.

Made misc edits.

January 20, 2024
----------------
1. Serial output now shows airport code for LEDs with storms/lightning, high winds, or very high winds vs just the LED
   number.  Also shows loop total, e.g. "Loop 1 of 300".
2. Changed default LOOP_INTERVAL to 1000 vs 5000.

January 19, 2024
----------------
Edited to show METAR string in serial output.  Prior code looked like it should have worked, but did not.

Added a second threshold for very high winds or gusts.
 1.  Added constant FADE_FOR_HIGH_WINDS to either blink/fade to 50% of flight category color for high winds, or
      blink black/clear.  Default is true.
  2.  LEDs will blink orange for very high winds or gusts.

Edited to show count of LEDs with thunderstorms/lightning, high winds, very high winds in serial output.

 January 17, 2024
 ----------------
 Added line: WiFi.setPhyMode(WIFI_PHY_MODE_11G); after WiFi.mode(WIFI_STA);
 This seems to fix a problem with connection failures to SmartWiFi routers that have the same SSID for both 2.4 and
 5 ghz bands.
 
 October 19, 2023
 ----------------
 Updated SERVER and BASE_URI to reflect aviationweather.gov changes.
  
 May 5, 2023
 -----------
 Fixed wifi connection logic.  Previously was doing an autoconnect every time a loop
 occurred, which is whenever there is lighting or thunderstorms or high winds found.
 
 April 27, 2023
 CHANGE LIST from Kyle Harmon's Code at https://github.com/WKHarmon/led-sectional
 --------------------------------------------------------------------------------
 1. Added WiFiManger. See this reference: https://randomnerdtutorials.com/wifimanager-with-esp8266-autoconnect-custom-parameter-and-manage-your-ssid-and-password/
 2. High wind (orange) color will now blink vs being a solid orange, and be shown for all flight categories, not just VFR
 3. Lightning (white blink) will occur not just for TS (thunderstorms), but also for LTG and LTNG reported in <raw_text> in response XML

 As a result of 2 and 3, any particular LED could have 3 colors...one for flight category, and blinking either or both of orange and white.

 See https://led-sectional.kyleharmon.com/ for the author's write up on building his map.

Issues
------
If an airport isn't reporting flight category, but has high winds, lightning or thunderstorms, you get the blink colors but no flight category (blank).
Better behavior might be to skip that LED?
*/


// December 1 2024 - modifed to include wvfr (windy vfr) several weeks ago
// toady I'm setting the wind setting down to my personal minimums.  (12kts) (student pilot)

// December 11th 2024 - modified again to remove redundant elseif vfr statement at the end
// that was preventing the LEDs from turning yellow for my windy personal minimums.  it worked after that.
// also - in the future - make sure that the 'libraries' folder is set up at the same level as the
// ino folder so you don't get a comple error.

// April 4th 2025 - uploaded this to github so that I can remember where the hell the dang thing is when I need it.

// May 16th 2025 - removed most all serial prints, removed 3 extra null entries in metar get list, removed extra restart 
// lines, also hooked LEDs up to 5V line - all in order to track down what is causing the freeze up every
// 2 days or so.  Uploaded to github again.

// July 27th - verified wifi reconnect fix is working and set wvfr to 15.




#include <uptime.h>
#if defined(ESP32)
#include <esp_wifi.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#endif

#include "src/WiFiManager/WiFiManager.h"      // https://github.com/tzapu/WiFiManager
#include <FastLED.h>
#include <vector>

using namespace std;
#define RebootMinutes 1440
#define NUM_AIRPORTS 22         // This is the number of airports in list, including nulls, NOT # LEDs in string.
#define WIND_THRESHOLD 15        // Winds or gusting winds above this but less than HIGH_WIND_THRESHOLD cause LED to either fade or blink between black/clear and the flight category color
#define HIGH_WIND_THRESHOLD 19   // Winds or gusting winds above this cause LED to blink orange
#define LOOP_INTERVAL 1000       // Interval in ms between brightness updates, and lightning/storm, high wind blinks
#define DO_LIGHTNING true        // Causes LED to blink white for thunderstorms or lightning
#define DO_WINDS true            // Causes LED to 1) fade flight category color or blink black/clear for winds > WIND_THRESHOLD, to blink orange for winds > HIGH_WIND_THRESHOLD
boolean HIGH_WINDS = false;      // Initialize global var
#define FADE_FOR_HIGH_WINDS true // Blink to fade flight category color by 50% vs blinking black/clear if set to false
boolean VERY_HIGH_WINDS = false; // Initialize global var
#define REQUEST_INTERVAL 900000  // Interval in ms for METAR updates. In practice LOOP_INTERVAL is added. 300000 is 5 minutes.

#define USE_LIGHT_SENSOR true      // Set true if you're using any light sensor.
#define LIGHT_SENSOR_TSL2561 true  // Set true if you're using a TSL2561 digital light sensor.  False assumes an analog sensor.

// WiFi Management for ESP8266
WiFiManager wm;
#define WIFI_TIMEOUT 75        // Connection timeout in seconds for call to setConfigPortalTimeout
boolean isWiFiConnected = false;

// Define the array of leds
CRGB leds[NUM_AIRPORTS];
#define LED_BUILTIN 2  // ON Board LED GPIO 2
#define DATA_PIN    5 // Kits shipped after March 1, 2019 should use 14. Earlier kits us 5.
                      // I'm using pin D5 (which is GPIO14) on my ESP8266 12-E NodeMCU in April, 2023.  Setting this to 5 works fine.
                      // SW - NOTE 4/4/25 - I'm using GPIO5 on my eps32 units. 

//WS2812 and GRB for LED string from AliExpress
#define LED_TYPE WS2811
#define COLOR_ORDER RGB
#define BRIGHTNESS 100   // 20-30 suggested for LED strip, 100 for 2811 bulbs
                        // If using a light sensor, this is the initial brightness on boot.

/* This section only applies if you have an ambient light sensor connected */
#if USE_LIGHT_SENSOR
/* The sketch will automatically scale the light between MIN_BRIGHTNESS and
MAX_BRIGHTNESS on the ambient light values between MIN_LIGHT and MAX_LIGHT
Set MIN_BRIGHTNESS and MAX_BRIGHTNESS to the same value to achieve a simple on/off effect. */
#define MIN_BRIGHTNESS 20 // Recommend values above 4 as colors don't show well below that
#define MAX_BRIGHTNESS 100 // Recommend values between 20 and 30

// Light values are a raw reading for analog and lux for digital
#define MIN_LIGHT -1 // Recommended default is 16 for analog and 2 for lux
#define MAX_LIGHT 500 // Recommended default is 30 to 40 for analog and 20 for lux

#if LIGHT_SENSOR_TSL2561
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <Wire.h>
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
#else
#define LIGHTSENSORPIN A0 // A0 is the only valid pin for an analog light sensor
#endif

#endif  // USE_LIGHT_SENSOR
/* ----------------------------------------------------------------------- */

std::vector<unsigned short int> lightningLeds;
std::vector<unsigned short int> windLeds;
std::vector<unsigned short int> highwindLeds;

std::vector<String> airports({
    "VFR", // 1 order of LEDs, starting with 1 should be KKIC; use VFR, WVFR, MVFR, IFR, LIFR for key; NULL for no airport
    "LIFR", // 2 Limited IFR
    "IFR", // 3 IFR Instrument Flight
    "MVFR", // 4 Marginal VFR
    "WVFR", // 5 WINDY VFR
    "VFR", // 6 VFR Visual Flight
    "KOTG", // 7 Worthington 1
    "KRST", // 8 Rochester 2
    "KSTP", // 9 St. Paul 3
    "KLXL", // 10 Little Falls 4
    "KDLH", // 11 Duluth 5
    "KCKC", // 12  Grand Marais 6
    "KINL", // 13 International Falls 7
    "KHCO", // 14 HALLOCK 8
    "KGFK", // 15 Grand Forks 9
    "KFAR", // 16 Fargo 10
    "KMOX", // 17 Morris 11
    "KAXN", // 18 Alexandria 12
    "KADC", // 19 Wadena 13
    "KPKD", // 20 Park Rapids 14
    "KBJI", // 21 Bemidji 15
    "KSTC" // 22 st cloud 16 KSTC
});

#define DEBUG false

#define READ_TIMEOUT 15     // Cancel query if no data received (seconds)
#define RETRY_TIMEOUT 15000 // in ms

#define SERVER "aviationweather.gov"
#define BASE_URI "/api/data/metar?format=xml&hoursBeforeNow=3&mostRecentForEachStation=true&ids="

boolean ledStatus = true;   // used so leds only indicate connection status on first boot, or after failure
int loops = -1;             // "loops" used only to set blink colors if any high/very high winds, thunderstorms and/or lightning are foundf

int status = WL_IDLE_STATUS;


void setup() {
  Serial.begin(115200);     // initialize serial port

  pinMode(LED_BUILTIN, OUTPUT); // give us control of the onboard LED
  digitalWrite(LED_BUILTIN, LOW);

  #if USE_LIGHT_SENSOR
  #if LIGHT_SENSOR_TSL2561
  Wire.begin(21, 22);
  if(!tsl.begin()) {
    /* There was a problem detecting the TSL2561 ... check your connections */
    Serial.println("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
  } else {
    tsl.enableAutoRange(true);
    tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);
  }
  #else
  pinMode(LIGHTSENSORPIN, INPUT);
  #endif //LIGHT_SENSOR_TSL2561
  #endif //USE_LIGHT_SENSOR

  // Initialize LEDs
  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_AIRPORTS);
  FastLED.setBrightness(BRIGHTNESS);

  WiFi.mode(WIFI_STA);
#if defined(ESP32)
  esp_wifi_set_protocol (WIFI_IF_STA, WIFI_PROTOCOL_11G);  // esp32 only
#elif defined(ESP8266)
  WiFi.setPhyMode(WIFI_PHY_MODE_11G); // esp8266 only. added 1/17/24 to fix issue connecting to smart routers with a single SSID for 2.4 and 5 ghz bands
#endif

  wm.setDebugOutput(true);
  wm.setConfigPortalTimeout(WIFI_TIMEOUT);
  wm.setConnectTimeout(WIFI_TIMEOUT);

}  // END SETUP

#if USE_LIGHT_SENSOR
void adjustBrightness() {
  unsigned char brightness;
  float reading;

  #if LIGHT_SENSOR_TSL2561
  sensors_event_t event;
  tsl.getEvent(&event);
  reading = event.light;
  #else
  reading = analogRead(LIGHTSENSORPIN);
  #endif // LIGHT_SENSOR_TSL2561

//  Serial.print("Light reading: ");
//  Serial.print(reading);
//  Serial.print(" raw, ");
//  Serial.println(uptime::getMinutes());

  if ((uptime::getMinutes()) >= RebootMinutes) {
  ESP.restart(); // Reboot the ESP32
}

  if (reading <= MIN_LIGHT) brightness = 0;
  else if (reading >= MAX_LIGHT) brightness = MAX_BRIGHTNESS;
  else {
    // Percentage in lux range * brightness range + min brightness
    float brightness_percent = (reading - MIN_LIGHT) / (MAX_LIGHT - MIN_LIGHT);
    brightness = brightness_percent * (MAX_BRIGHTNESS - MIN_BRIGHTNESS) + MIN_BRIGHTNESS;
  }

//  Serial.print(brightness);
//  Serial.println(" brightness");
  FastLED.setBrightness(brightness);
  FastLED.show();
}
#endif // USE_LIGHT_SENSOR

void loop() {
  digitalWrite(LED_BUILTIN, LOW); // on if we're awake

  uptime::calculateUptime();

  #if USE_LIGHT_SENSOR
  adjustBrightness();
  #endif

  int c;
  loops++;
//  Serial.print("Loop ");
//  Serial.print(loops);
//  Serial.print(" of ");
//  Serial.println(REQUEST_INTERVAL/LOOP_INTERVAL);
  unsigned int loopThreshold = 1;
  if (DO_LIGHTNING || DO_WINDS || USE_LIGHT_SENSOR)
    loopThreshold = REQUEST_INTERVAL / LOOP_INTERVAL;

  // Connect to WiFi
  if (!isWiFiConnected) {
    if (ledStatus) fill_solid(leds, NUM_AIRPORTS, CRGB::Orange); // indicate status with LEDs, but only on first run or error
    FastLED.show();
    isWiFiConnected = wm.autoConnect();
    if (isWiFiConnected) {
      Serial.println("Connected to local network");
      if (ledStatus) fill_solid(leds, NUM_AIRPORTS, CRGB::Purple);  //set to purple while retrieving data
      FastLED.show();
      ledStatus = false;
    }
    else {
      Serial.println("Failed to connect to local network or hit timeout");
      fill_solid(leds, NUM_AIRPORTS, CRGB::Orange);
      FastLED.show();
      ledStatus = true;
      wm.autoConnect("AutoConnectAP");  // should popup signin else goto 192.168.4.1 after connecting to AutoConnectAP or ESPxxxx
      return;
    }
  }

  // Blink white if thunderstorms (TS) found in <wx_string> or if lightning (LTG or LTNG) found in <raw_text>
  if (DO_LIGHTNING && lightningLeds.size() > 0) {
    std::vector<CRGB> lightning(lightningLeds.size());
      for (unsigned short int i = 0; i < lightningLeds.size(); ++i) {
      unsigned short int currentLed = lightningLeds[i];
      lightning[i] = leds[currentLed]; // temporarily store original color
      leds[currentLed] = CRGB::White;  // set to white briefly

//      Serial.print("Lightning on LED: ");
//      Serial.print(currentLed);
//      Serial.print(", Airport Code: ");
//      Serial.println(airports[currentLed]);
    }
    delay(25); // extra delay seems necessary with light sensor
    FastLED.show();
    delay(1000);
    for (unsigned short int i = 0; i < lightningLeds.size(); ++i) {
      unsigned short int currentLed = lightningLeds[i];
      leds[currentLed] = lightning[i]; // restore original color
    }
    FastLED.show();
  }

  // // Blink orange if winds or gusts exceed HIGH_WIND_THRESHOLD
  // if (DO_WINDS && highwindLeds.size() > 0) {
  //   std::vector<CRGB> veryhighwind(highwindLeds.size());
  //   for (unsigned short int i = 0; i < highwindLeds.size(); ++i) {
  //     unsigned short int currentLed = highwindLeds[i];
  //     veryhighwind[i] = leds[currentLed];  // temporarily store original color
  //     leds[currentLed] = CRGB::Yellow; // set to yellow briefly
  //     Serial.print("Very high wind or gusts on LED: ");
  //     Serial.print(currentLed);
  //     Serial.print(", Airport Code: ");
  //     Serial.println(airports[currentLed]);
  //   }
  //   delay(25); // extra delay seems necessary with light sensor
  //   FastLED.show();
  //   delay(1000);
  //   for (unsigned short int i = 0; i < highwindLeds.size(); ++i) {
  //     unsigned short int currentLed = highwindLeds[i];
  //     leds[currentLed] = veryhighwind[i]; // restore original color
  //   }
  //   FastLED.show();
  // }

  // // Blink clear/black, or fade flight category color, if winds or gusts exceed WIND_THRESHOLD
  // if (DO_WINDS && windLeds.size() > 0) {
  //   std::vector<CRGB> highwind(windLeds.size());
  //   for (unsigned short int i = 0; i < windLeds.size(); ++i) {
  //     unsigned short int currentLed = windLeds[i];
  //     highwind[i] = leds[currentLed];   // temporarily store original color
  //     if (FADE_FOR_HIGH_WINDS)
  //       leds[currentLed] %= 128;        //fade by 50% (128/256), never fading to black
  //     else
  //       leds[currentLed] = CRGB::Black; // set to clear briefly
  //     Serial.print("High wind or gusts on LED: ");
  //     Serial.print(currentLed);
  //     Serial.print(", Airport Code: ");
  //     Serial.println(airports[currentLed]);
  //   }
  //   delay(25); // extra delay seems necessary with light sensor
  //   FastLED.show();
  //   delay(1000);
  //   for (unsigned short int i = 0; i < windLeds.size(); ++i) {
  //     unsigned short int currentLed = windLeds[i];
  //     leds[currentLed] = highwind[i]; // restore original color
  //   }
  //   FastLED.show();
  // }

  if (loops >= loopThreshold || loops == 0) {
    loops = 0;
    if (DEBUG) {
      fill_gradient_RGB(leds, NUM_AIRPORTS, CRGB::Red, CRGB::Blue); // Just let us know we're running
      FastLED.show();
    }


//     Serial.println("Getting METARs ...");
//     if (getMetars()) {
//       Serial.println("Refreshing LEDs.");
//       FastLED.show();
//       if ((DO_LIGHTNING && lightningLeds.size() > 0) || (DO_WINDS && windLeds.size() > 0) || USE_LIGHT_SENSOR || (DO_WINDS && highwindLeds.size() > 0)) {
//         Serial.println("There is lightning, thunderstorms or high wind, or we're using a light sensor, so no long sleep.");
//         Serial.print("# LEDs with high winds or gusts: ");
//         Serial.println(windLeds.size());
//         Serial.print("# LEDs with very high winds or gusts: ");
//         Serial.println(highwindLeds.size());
//         Serial.print("# LEDs with thunderstorms or lightning: ");
//         Serial.println(lightningLeds.size());
//         digitalWrite(LED_BUILTIN, HIGH);
//         delay(LOOP_INTERVAL); // pause during the interval
//       }
//       else {
//         Serial.print("No thunderstorms or lightning or strong winds. Going into sleep for: ");
//         Serial.println(REQUEST_INTERVAL);
//         digitalWrite(LED_BUILTIN, HIGH);
//         delay(REQUEST_INTERVAL);
//       }
//     } else {
//       digitalWrite(LED_BUILTIN, HIGH);
//       delay(RETRY_TIMEOUT); // try again if unsuccessful
//     }
//   } else {
//     digitalWrite(LED_BUILTIN, HIGH);
//     delay(LOOP_INTERVAL);   // pause during the interval
//   }
// }

if (WiFi.status() != WL_CONNECTED) {
  Serial.println("WiFi disconnected. Attempting to reconnect...");
  WiFi.disconnect();  // Reset connection
  delay(1000);
  isWiFiConnected = wm.autoConnect();
  if (!isWiFiConnected) {
    Serial.println("Reconnection failed. Skipping METAR fetch.");
    digitalWrite(LED_BUILTIN, HIGH);
    delay(RETRY_TIMEOUT);
    return;  // skip this loop
  } else {
    Serial.println("Reconnected to WiFi.");
    ledStatus = false;
  }
}


    Serial.println("Getting METARs ...");
    if (getMetars()) {
      Serial.println("Refreshing LEDs.");
      FastLED.show();
      if ((DO_LIGHTNING && lightningLeds.size() > 0) || USE_LIGHT_SENSOR) {
        Serial.println("There is lightning or we're using a light sensor, so no long sleep.");
        digitalWrite(LED_BUILTIN, HIGH);
        delay(LOOP_INTERVAL); // pause during the interval
      } else {
        Serial.print("No lightning; Going into sleep for: ");
        Serial.println(REQUEST_INTERVAL);
        digitalWrite(LED_BUILTIN, HIGH);
        delay(REQUEST_INTERVAL);
      }
    } else {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(RETRY_TIMEOUT); // try again if unsuccessful
    }
  } else {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(LOOP_INTERVAL); // pause during the interval
  }
  }


bool getMetars(){
  lightningLeds.clear(); // clear out existing lightning LEDs since they're global
  windLeds.clear();
  highwindLeds.clear();
  fill_solid(leds, NUM_AIRPORTS, CRGB::Black); // Set everything to black just in case there is no report
  uint32_t t;
  char c;
  boolean readingRawText = false;
  boolean readingAirport = false;
  boolean readingCondition = false;
  boolean readingWind = false;
  boolean readingGusts = false;
  boolean readingWxstring = false;
  
  std::vector<unsigned short int> led;
  String currentRawText = "";
  String currentAirport = "";
  String currentCondition = "";
  String currentLine = "";
  String currentWind = "";
  String currentGusts = "";
  String currentWxstring = "";
  String airportString = "";
  bool firstAirport = true;
  
  // Build comma-separated list of airport IDs from airport string vector (list) to send to www.aviationweather.gov
  for (int i = 0; i < NUM_AIRPORTS; i++) {
    if (airports[i] != "NULL" && airports[i] != "VFR" && airports[i] != "MVFR" && airports[i] != "WVFR" && airports[i] != "IFR" && airports[i] != "LIFR") {
      if (firstAirport) {
        firstAirport = false;
        airportString = airports[i];
      } else airportString = airportString + "," + airports[i];
    }
  }

  WiFiClientSecure client;
  client.setInsecure();
  Serial.println("\nStarting connection to server...");
  if (!client.connect(SERVER, 443)) {
    Serial.println("Connection failed!");
    client.stop();
    return false;
  } else {
    Serial.println("Connected ...");
    Serial.print("GET ");
    Serial.print(BASE_URI);
    Serial.print(airportString);
    Serial.println(" HTTP/1.1");
    Serial.print("Host: ");
    Serial.println(SERVER);
    Serial.println("User-Agent: LED Map Client");
    Serial.println("Connection: close");
    Serial.println();
    // Make the GET request, and print it to console
    client.print("GET ");
    client.print(BASE_URI);
    client.print(airportString);
    client.println(" HTTP/1.1");
    client.print("Host: ");
    client.println(SERVER);
    client.println("User-Agent: LED Sectional Client");
    client.println("Connection: close");
    client.println();
    client.flush();
    t = millis(); // start time
    FastLED.clear();

    Serial.print("Getting data");

    while (!client.connected()) {
      if ((millis() - t) >= (READ_TIMEOUT * 1000)) {
        Serial.println("---Timeout---");
        client.stop();
        return false;
      }
      Serial.print(".");
      delay(1000);
    }

    Serial.println();

    while (client.connected()) {
      if ((c = client.read()) >= 0) {
        yield(); // Otherwise the WiFi stack can crash
        currentLine += c;
        if (c == '\n') currentLine = "";
        if (currentLine.endsWith("<raw_text>")) { // start paying attention
             if (!led.empty()) {                  // we assume we are recording results at each change in airport
               for (vector<unsigned short int>::iterator it = led.begin(); it != led.end(); ++it)  {
                 doColor(currentAirport, *it, currentWind.toInt(), currentGusts.toInt(), currentCondition, currentWxstring, currentRawText);
               }
             led.clear();
             }
          currentRawText = "";
          currentAirport = "";                    // Reset everything when the airport changes
          readingRawText = true;
          currentCondition = "";
          currentWind = "";
          currentGusts = "";
          currentWxstring = "";
        } else if (readingRawText)  {
            if (!currentLine.endsWith("<"))
              currentRawText += c;
            else
              readingRawText = false;
         } else if (currentLine.endsWith("<station_id>")) { 
          readingAirport = true;
        } else if (readingAirport) {
            if (!currentLine.endsWith("<"))
              currentAirport += c;
            else {
              readingAirport = false;
              for (unsigned short int i = 0; i < NUM_AIRPORTS; i++) {
                if (airports[i] == currentAirport)
                  led.push_back(i);
              }
            }
        } else if (currentLine.endsWith("<wind_speed_kt>")) {
          readingWind = true;
        } else if (readingWind) {
          if (!currentLine.endsWith("<")) {
            currentWind += c;
          } else {
            readingWind = false;
          }
        } else if (currentLine.endsWith("<wind_gust_kt>")) {
          readingGusts = true;
        } else if (readingGusts) {
          if (!currentLine.endsWith("<")) {
            currentGusts += c;
          } else {
            readingGusts = false;
          }
        } else if (currentLine.endsWith("<flight_category>")) {
          readingCondition = true;
        } else if (readingCondition) {
          if (!currentLine.endsWith("<")) {
            currentCondition += c;
          } else {
            readingCondition = false;
          }
        } else if (currentLine.endsWith("<wx_string>")) {
          readingWxstring = true;
        } else if (readingWxstring) {
          if (!currentLine.endsWith("<"))
            currentWxstring += c;
          else
            readingWxstring = false;
        }
        t = millis(); // Reset timeout clock
      } else if ((millis() - t) >= (READ_TIMEOUT * 1000)) {
        Serial.println("---Timeout---");
        fill_solid(leds, NUM_AIRPORTS, CRGB::Cyan); // indicate status with LEDs
        FastLED.show();
        ledStatus = true;
        client.stop();
        return false;
      }
    }
  }
  // need to doColor this for the last airport
  for (vector<unsigned short int>::iterator it = led.begin(); it != led.end(); ++it) {
    doColor(currentAirport, *it, currentWind.toInt(), currentGusts.toInt(), currentCondition, currentWxstring, currentRawText);
  }
  led.clear();

  // Do the key LEDs now if they exist
  for (int i = 0; i < (NUM_AIRPORTS); i++) {
    // Use this opportunity to set colors for LEDs in our key
    if (airports[i] == "VFR") leds[i] = CRGB::Green;
    else if (airports[i] == "WVFR") leds[i] = CRGB::Yellow;
    else if (airports[i] == "MVFR") leds[i] = CRGB::Blue;
    else if (airports[i] == "IFR") leds[i] = CRGB::Red;
    else if (airports[i] == "LIFR") leds[i] = CRGB::Magenta;
  }

  client.stop();
  return true;
}

void doColor(String identifier, unsigned short int led, int wind, int gusts, String condition, String wxstring, String currentRawText) {
  CRGB color;
  Serial.print(identifier);
  Serial.print(": ");
  Serial.print(condition);
  Serial.print(" ");
  Serial.print(wind);
  Serial.print("G");
  Serial.print(gusts);
  Serial.print("kts LED ");
  Serial.print(led);
  Serial.print(" WX: ");
  Serial.println(currentRawText);
  
  // LTG or LTNG for lightning is in raw_text of METAR, not in any other XML field.
  // We'll blink white for either or both of lightning and thunderstorms.
  if ((wxstring.indexOf("TS") != -1) || (currentRawText.indexOf("LTG") != -1) || (currentRawText.indexOf("LTNG") != -1)) {
    Serial.println("... found thunderstorms or lightning!");
    lightningLeds.push_back(led);
  }

  // if ((wind > HIGH_WIND_THRESHOLD) || (gusts > HIGH_WIND_THRESHOLD)) {
  //   Serial.println("... found very high winds or gusts!");
  //   VERY_HIGH_WINDS = true;
  //   highwindLeds.push_back(led);
  // } else if ((wind > WIND_THRESHOLD) || (gusts > WIND_THRESHOLD)) {
  //      Serial.println("... found high winds or gusts!");
  //     HIGH_WINDS = true;
  //     windLeds.push_back(led);
  // }

  if (condition == "LIFR" || identifier == "LIFR") color = CRGB::Magenta;
  else if (condition == "IFR") color = CRGB::Red;
  else if (condition == "MVFR") color = CRGB::Blue;
 // else if (condition == "VFR")color = CRGB::Green;
    else if (condition == "VFR") {
    if ((wind > WIND_THRESHOLD || gusts > WIND_THRESHOLD) && DO_WINDS) {
      color = CRGB::Yellow;
    } else {
      color = CRGB::Green;
    }
  } else color = CRGB::Black;

  leds[led] = color;

}
