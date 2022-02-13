#include <Keyboard.h>

#include <Adafruit_NeoPixel.h>

#include <Adafruit_MQTT_FONA.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include "arduino_secrets.h"

//WIFI Configuration
#define SPIWIFI       SPI  // The SPI port
#define SPIWIFI_SS     6   // Chip select pin
#define ESP32_RESETN   5   // Reset pin
#define SPIWIFI_ACK    7   // a.k.a BUSY or READY pin
#define ESP32_GPIO0   -1
int status = WL_IDLE_STATUS;     // the WiFi radio's status


// Neopixel for status
#define PIN       11
#define NUMPIXELS 1
#define DELAYVAL 2
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

typedef void (*Callback)(int status );

struct Debounce {
  int state;
  int lastButtonState;
  int input_pin;
  unsigned long lastDebounceTime;
  unsigned long debounceDelay;
  Callback callback;
};


struct Debounce new_debounce(int input_pin, Callback callback) {
  struct Debounce result;
  result.input_pin = input_pin;
  result.debounceDelay = 50;
  result.lastButtonState = LOW;
  result.callback = callback;
  return result;
}

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, SECRET_MQTT_SERVER, MQTT_PORT, SECRET_MQTT_USER, SECRET_MQTT_PASS);
Adafruit_MQTT_Publish test = Adafruit_MQTT_Publish(&mqtt, "/office/meeting_mode/set");
struct Debounce meeting_switch = new_debounce(3, set_meeting_mode);
struct Debounce video_switch = new_debounce(2, toggle_camera);
struct Debounce mic_switch = new_debounce(1, toggle_mic);
struct Debounce end_meeting_switch = new_debounce(0, quit_meeting);


void setup() {
  setup_button(meeting_switch);
  setup_button(video_switch);
  setup_button(mic_switch);
  setup_button(end_meeting_switch);

  pixels.begin();
  statusPixel(255, 255, 0);
  unsigned long serialStart = millis();
  // Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial && millis() < serialStart + 1500) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  WiFi.setPins(SPIWIFI_SS, SPIWIFI_ACK, ESP32_RESETN, ESP32_GPIO0, &SPIWIFI);
  // check for the WiFi module:
  while (WiFi.status() == WL_NO_MODULE)
  {
    statusPixel(255, 0, 0);
    Serial.println("Communication with WiFi module failed!");
    delay(1000);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < "1.0.0") {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to WiFi network:
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(SECRET_SSID);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(SECRET_SSID, SECRET_PASS);

    // wait 5 seconds for connection:
    delay(5000);
  }

  // you're connected now, so print out the data:
  Serial.println("You're connected to the network");
  printWiFiStatus();
  MQTT_connect();
  statusPixel(0, 255, 0);
  Keyboard.begin();
}

void setup_button(Debounce button) {
  pinMode(button.input_pin, INPUT);
}

void loop() {
  debounce(&meeting_switch);
  debounce(&video_switch);
  debounce(&mic_switch);
  debounce(&end_meeting_switch);

}

void MQTT_connect()
{
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected())
  {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0)
  { // connect will return 0 for connected
    statusPixel(255, 0, 0);
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000); // wait 5 seconds
    retries--;
    if (retries == 0)
    {
      Serial.println("Failed to connect to MQTT, waiting for reset");
      // basically die and wait for WDT to reset me
      while (1);
    }
  }

  Serial.println("MQTT Connected!");
}

void MQTT_reconnect() {
  mqtt.disconnect();
  MQTT_connect();
}

void printWiFiStatus()
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

  
  int pingResult = WiFi.ping("www.google.com");

  if (pingResult >= 0) {
    Serial.print("SUCCESS! RTT = ");
    Serial.print(pingResult);
    Serial.println(" ms");
  } else {
    Serial.print("FAILED! Error code: ");
    Serial.println(pingResult);
  }
}

void toggle_mic(int ignore) {
  Serial.println("Toggling mic");
//  Keyboard.write('m'); -- Bluejeans
  Keyboard.press(KEY_LEFT_CTRL);
  Keyboard.press(KEY_LEFT_ALT);
  Keyboard.press(KEY_LEFT_SHIFT);
  Keyboard.write('a');
  Keyboard.releaseAll();
}

void toggle_camera(int ignore) {
  Serial.println("Toggling camera");
  Keyboard.press(KEY_LEFT_CTRL);
  Keyboard.press(KEY_LEFT_ALT);
  Keyboard.press(KEY_LEFT_SHIFT);
  Keyboard.write('v'); // Comment other keys for bluejeans
  Keyboard.releaseAll();
}

void quit_meeting(int state) {
  Serial.println("Leaving meeting");

  if (state == HIGH) {
    Keyboard.press(KEY_LEFT_GUI);
    Keyboard.write('w');
    Keyboard.releaseAll();
    delay(10);
    Keyboard.press(KEY_RETURN);
    Keyboard.releaseAll();
  }

}

void set_meeting_mode(int state) {
  if (state) {
    Serial.println("Turning meeting mode on");
    publish("on");
  } else {
    Serial.println("Turning meeting mode off");
    publish("off");
  }

}

void publish(char payload[]) {
  int retries = 3;
  while (retries > 0) {
    if (!test.publish(payload)) {
      Serial.println(F("MQTT call failed!"));

      statusPixel(255, 127, 0);
      retries--;
    }
    else {
      statusPixel(0, 255, 0);
      return;
    }
  }
  MQTT_reconnect();
  publish(payload);
}

void statusPixel(int red, int green, int blue) {
  pixels.setPixelColor(0, pixels.Color(red, green, blue));
  pixels.show();
}

void debounce(Debounce *button) {
  // read the state of the switch into a local variable:
  int reading = digitalRead(button->input_pin);

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading != button->lastButtonState) {
    // reset the debouncing timer
    button->lastDebounceTime = millis();
  }

  // save the reading. Next time through the loop, it'll be the lastButtonState:
  button->lastButtonState = reading;

  if ((millis() - button->lastDebounceTime) > button->debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:
    // if the button state has changed:
    if (reading != button->state) {
      button->state = reading;
      button->callback(reading);
    }
  }

}
