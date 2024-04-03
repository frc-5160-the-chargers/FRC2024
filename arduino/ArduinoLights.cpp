/*
 * Part of this code is created by by ArduinoGetStarted.com
 * Author: Daniel Chen
 */

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#define SNAKE_LENGTH 10
#define PIN_NEO_PIXEL  4   // Arduino pin that connects to NeoPixel
#define NUM_PIXELS     26  // The number of LEDs (pixels) on NeoPixel
#define DELAY_MS      30

#define PRINT_RIO_OUTPUT false
// tbd
#define PORT_A_PIN 6
#define PORT_B_PIN 8
#define PORT_C_PIN 10

Adafruit_NeoPixel NeoPixel(NUM_PIXELS, PIN_NEO_PIXEL, NEO_GRB + NEO_KHZ800);

const uint8_t gamma8[] = {
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
    1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
    2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
    5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
   10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
   17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
   25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
   37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
   51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
   69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
   90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,
  115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,
  144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
  177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
  215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255 };


int r1 = 50;
int g1 = 65;
int b1 = 158;

int r2 = 253;
int g2 = 200;
int b2 = 14;

uint32_t BLUE = NeoPixel.Color(gamma8[50], gamma8[65], gamma8[158]);
uint32_t YELLOW = NeoPixel.Color(gamma8[253], gamma8[200], gamma8[14]);

uint32_t RED = NeoPixel.Color(255,0,0);
uint32_t ORANGE = NeoPixel.Color(255,165,0);
uint32_t GREEN = NeoPixel.Color(0, 255, 0);
uint32_t PURPLE = NeoPixel.Color(160,32,40);
uint32_t NOTHING = NeoPixel.Color(0,0,0);

float portARaw = 0.0;
float portBRaw = 0.0;
float portCRaw = 0.0;

// signals here
bool portASignal = false;
bool portBSignal = false;
bool portCSignal = false;

/**
* Updates all signals.
*/
void updateSignals(){
  portARaw = analogRead(PORT_A_PIN);
  portBRaw = analogRead(PORT_B_PIN);
  portCRaw = analogRead(PORT_C_PIN);

  if (PRINT_RIO_OUTPUT){
    Serial.print("Port A Raw Signal: ");
    Serial.println(portARaw);
    Serial.print("Port B Raw Signal: ");
    Serial.println(portBRaw);
    Serial.print("Port B Raw Signal: ");
    Serial.println(portCRaw);
  }

  portASignal = portARaw > 60.0;
  portBSignal = portBRaw > 60.0;
  portCSignal =  portCRaw > 60.0;
}

void setAllPixels(uint32_t color){
  for (int pixel = 0; pixel < NUM_PIXELS; pixel++){
      NeoPixel.setPixelColor(pixel, color);
  }
}

bool solidOrangeRequested(){return portASignal && portBSignal && portCSignal;}
bool blinkingOrangeRequested(){return portASignal && portBSignal && !portCSignal;}
bool redRequested(){ return !portASignal && !portBSignal && portCSignal; }
bool yellowRequested(){ return !portASignal && portBSignal && !portCSignal; }
bool greenRequested(){ return portASignal && !portBSignal && portCSignal; }


void setup() {
  Serial.begin(9600);

  NeoPixel.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  for (int pixel = 0; pixel < NUM_PIXELS; pixel++) {
    NeoPixel.setPixelColor(pixel, BLUE);
  }

  NeoPixel.show();
  portASignal = false;
  portBSignal = false;
}

void loop() {
  updateSignals();

  if (solidOrangeRequested()){
    setAllPixels(ORANGE);
  }else if (blinkingOrangeRequested()){
    setAllPixels(ORANGE);
    delay(DELAY_MS);
    setAllPixels(NOTHING);
    delay(DELAY_MS);
  }else if (redRequested()){
    setAllPixels(RED);
  }else if (yellowRequested()){
    setAllPixels(YELLOW);
  }else if (greenRequested()){
    setAllPixels(GREEN);
  }else{
    // Default Lights
    // turn pixels from blue to yellow with delay between each pixel then sets them back to BLUE
    for (int pixel = 0; pixel < NUM_PIXELS+SNAKE_LENGTH+1; pixel++) { // for each pixel
      NeoPixel.setPixelColor(pixel, YELLOW);
      NeoPixel.show();   // send the updated pixel colors to the NeoPixel hardware.

      if(pixel > SNAKE_LENGTH) {
        NeoPixel.setPixelColor(pixel-SNAKE_LENGTH-1, BLUE);
        NeoPixel.show();   // send the updated pixel colors to the NeoPixel hardware.x
      }

      delay(DELAY_MS); // pause between each pixel
    }
  }
}