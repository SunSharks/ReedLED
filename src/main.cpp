/* ESP32 Deep sleep: https://randomnerdtutorials.com/esp32-external-wake-up-deep-sleep/
*/

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define LED_PIN     21
#define NUM_LEDS    29
#define BRIGHTNESS  5
#define LED_TYPE    SK6812
#define COLOR_ORDER GRB
#define UPDATES_PER_SECOND 100

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRBW + NEO_KHZ800);

// NEOPIXEL BEST PRACTICES for most reliable operation:
// - Add 1000 uF CAPACITOR between NeoPixel strip's + and - connections.
// - MINIMIZE WIRING LENGTH between microcontroller board and first pixel.
// - NeoPixel strip's DATA-IN should pass through a 300-500 OHM RESISTOR.
// - AVOID connecting NeoPixels on a LIVE CIRCUIT. If you must, ALWAYS
//   connect GROUND (-) first, then +, then data.
// - When using a 3.3V microcontroller with a 5V-powered NeoPixel strip,
//   a LOGIC-LEVEL CONVERTER on the data line is STRONGLY RECOMMENDED.

RTC_DATA_ATTR int bootCount = 0;

int reedpin = 22; // KY-025 digital interface
int reedpin2 = 23;
// int analogPin = A0; // KY-025 analog interface
int reedval; // digital readings
int analogVal; //analog readings
int reed2val;


int curr_color[4] = {0,0,0,0};

void write_strip(int d=0){
  for(int i=0; i<NUM_LEDS; i++){
    strip.setPixelColor(i, strip.Color(curr_color[0], curr_color[1], curr_color[2], curr_color[3]));
    delay(d);
    strip.show();
  }
}

void setup() {
  ++bootCount;
  strip.begin();
  strip.setBrightness(50);
  strip.show(); // Initialize all pixels to 'off'
  pinMode(reedpin, INPUT);
  pinMode(reedpin2, INPUT);
  //pinMode(analogPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // Read the digital interface
  reedval = digitalRead(reedpin);
  reed2val = digitalRead(reedpin2);
  if(reedval == LOW) // if magnetic field is detected
  {
    curr_color[3] = 0;
  }
  else
  {
    curr_color[3] = 100;
  }
  if(reed2val == LOW) // if magnetic field is detected
  {
    curr_color[1] = 0;
  }
  else
  {
    curr_color[1] = 100;
  }
  write_strip();
  while(digitalRead(reedpin) && digitalRead(reedpin2)){
    delay(10);
  }
  // Read the analog interface
  // analogVal = analogRead(analogPin);
  // Serial.println(analogVal); // print analog value to serial
  delay(100);

}
