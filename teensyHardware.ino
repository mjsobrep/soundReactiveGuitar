///////////////////
// Teensy Based Frequency responsive light show
// written by: Michael Sobrepera COPYRIGHT 2015
// CONTACT: mjsobrep@live.com   (770)324-6196
// V:0.1
// 13-May-15
// Author (Michael Sobrepera retains ownership. Use is as is, there is no warranty of anykind implied or expreesed for fitness for 
// any purpose. Modify and redistribute freely, retain author name and license in any redistribution and modification. May be used
// for commercial purposes, so long as such usage does not infringe on the usage of anyother. 
///////////////////

#include <Audio.h> //The library to do FFT
#include <Wire.h> //Library to use GPIO
#include <SPI.h> //Required for Wire?
#include <SD.h> //Required for Wire?
#include <Adafruit_NeoPixel.h> //Libraryn for NeoPixels
#include <Snooze.h>

#define DEFAULTS 1 //default saturation for colors
#define DEFAULTV 1 //defaults value for colors
#define DEFAULTHFH 120 //the highest frequency will be blue
#define DEFAULTLFH 359 //the lowest frequency will be red
#define DEFAULTB 200

#define NEOPIN 3 //The pin for the Neopixel data line
#define NEOLEN 38 //The length of NeoPixels connected
#define MIDPIX 19 //The pixel where the scrolling startys
#define MODEBUTTON 2 //pin to change modes
#define CHARGINGPIN 0 //pin which goes high when the battery is charging
#define CHARGEDPIN 1 //pin which goes high when the battery is charged
#define DEBOUNCE 100 //a value to help debounce (ms)
#define MINBUTTON 50

//modes:
#define NUMSTATES 4 // the number of playable states
#define SLEEP 20
#define SLEEPWAITING 21
#define PLAYINGFREQ 1
#define CHARGING 10
#define CHARGED 11
#define SCROLLING 2
#define RANDOM 3
#define RANDOMSCROLLING 4
#define DEFAULTSTATE SCROLLING

#define UPDATEDELAY 10 //this is how frequently it will run the FFT
#define SLEEPPRESS 3000 //number of milliseconds to press button to put it all to sleep
#define MEMORY 30000 //how long to remember high and low notes

//DEBUGGING COMMANDS TO PRINT OUT USEFUL INFO TO SERIAL:
#define FREQINFO
#define COLORINFO
#define STATEINFO

//STUFF TO HANDLE RESTART AFTER A WAKE FROM SLEEP:
#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);

//STURUCTURE TO HOLD COLOR
struct ColorReturn {
  bool valid;
  uint32_t color;
} colorToReturn;

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NEOLEN, NEOPIN, NEO_GRB + NEO_KHZ800); //set up the NeoPixels object
AudioInputAnalog         adc1(A0);           //xy=200.94000244140625,192.70999145507812 -> reads from an analog input pin
AudioAnalyzeFFT1024      fft;       //xy=372.54998779296875,200.44998168945312 -> sets up for FFT
AudioConnection          patchCord1(adc1, fft); //Connects the audio input to the fft

int state = DEFAULTSTATE;
unsigned long lastRead = millis();
unsigned long buttonOn;
unsigned long buttonOff;
bool buttonPressed = false;
bool quitCharging = false;
bool goingUp = false;
int brightness = DEFAULTB;
int maxRunningBin[] = {0, 0, 0, 0};
int minRunningBin[] = {600, 600, 600, 600};
unsigned long lastBinMove = millis();

SnoozeBlock config; //sets up for sleep

void setup() {
  Serial.begin(9600); //sets up the USB connection
  Serial.println("connected"); //prints a connected message to USB connection
  AudioMemory(12);  //allocate memory for the FFT...
  Serial.println("memory allocated");
  //Do something:
  for (int bob = 0; bob < 100; bob++) {
    Serial.print(".");
    delay(10);
  }
  Serial.println("");//newline

  //SET UP IO PINS
  pinMode(MODEBUTTON, INPUT_PULLUP);
  pinMode(CHARGINGPIN, INPUT);
  pinMode(CHARGEDPIN, INPUT);
  *portConfigRegister(CHARGEDPIN) |= PORT_PCR_PE; //pull enable
  *portConfigRegister(CHARGEDPIN) &= ~PORT_PCR_PS; //pull down
  *portConfigRegister(CHARGINGPIN) |= PORT_PCR_PE; //pull enable
  *portConfigRegister(CHARGINGPIN) &= ~PORT_PCR_PS; //pull down


  config.pinMode(MODEBUTTON, INPUT_PULLUP, RISING); //configures the button to wake from sleep

  pixels.begin(); //setup pixels
  pixels.show(); //push out some data for pixels
}
void loop() {
  modeChange();
  if (state == SLEEP) {
    pixels.setBrightness(0);
    pixels.show();
    #ifdef STATEINFO
      Serial.println("going to sleep");
    #endif
    delay(200);
    Snooze.deepSleep( config ); //low power mode
    delay(200);
    CPU_RESTART
    state = DEFAULTSTATE; //if we make it here, it means we woke back up
    for (int i = 0; i < 4; i++) {
      maxRunningBin[i] = 0;
      minRunningBin[i] = 0;
    }
    delay(200);
    #ifdef STATEINFO
      Serial.println("woke back up");
    #endif
  } else if (state == PLAYINGFREQ) {
    if ((millis() >= (UPDATEDELAY + lastRead)) | (lastRead > millis())) { //if it has been a while since we have read the FFT
      randFFTLights();
      brightness = DEFAULTB;
      pixels.setBrightness(brightness);
      pixels.show();
    }
  } else if (state == SCROLLING) {
    if ((millis() >= (UPDATEDELAY*3 + lastRead)) | (lastRead > millis())) { //if it has been a while since we have read the FFT
      ScrollingFTTLights();
      brightness = DEFAULTB;
      pixels.setBrightness(brightness);
      pixels.show();
    }
  } else if (state == CHARGING) {

    if ((millis() >= (UPDATEDELAY + lastRead)) | (lastRead > millis())) { //if it has been a while since we have done something
      lastRead = millis();
      allToColor(pixels.Color(255, 89, 0));
      cycleBrightness();
      pixels.show();
    }
  } else if (state == CHARGED) {

    if ((millis() >= (UPDATEDELAY + lastRead)) | (lastRead > millis())) { //if it has been a while since we have done something
      lastRead = millis();
      allToColor(pixels.Color(0, 255, 0));
      cycleBrightness();
      pixels.show();
    }
  } else if (state == RANDOM) {
    if ((millis() >= (UPDATEDELAY + lastRead)) | (lastRead > millis())) { //if it has been a while since we have done something
      lastRead = millis();
      pixels.setPixelColor(random(NEOLEN), HSVtoColor(random(255), 1, 1));
      brightness = DEFAULTB;
      pixels.setBrightness(brightness);
      pixels.show();
    }
  } else if (state == RANDOMSCROLLING) {
    if ((millis() >= (UPDATEDELAY * 15 + lastRead)) | (lastRead > millis())) { //if it has been a while since we have done something
      lastRead = millis();
      ScrollingRandomLights();
      brightness = DEFAULTB;
      pixels.setBrightness(brightness);
      pixels.show();
    }
  }
}
void cycleBrightness() {
  if (goingUp) {
    brightness++;
    if (brightness > 254) {
      goingUp = false;
    }
  }
  else {
    brightness--;
    if (brightness < 10) {
      goingUp = true;
    }
  }
  #ifdef COLORINFO
    Serial.print("brightness changed to: ");
    Serial.println(brightness);
  #endif
  pixels.setBrightness(brightness);
}

//SETS ALL NEOPIXELS TO ONE COLOR
void allToColor(uint32_t color) {
  for (int i = 0; i < NEOLEN; i++) {
    pixels.setPixelColor(i, color);
  }
}

//RANDOMLY ASSIGNS COLORS TO NEOPIXELS
void randFFTLights() {
  FFTbasedLights();
  if (colorToReturn.valid) {
    pixels.setPixelColor(random(NEOLEN), colorToReturn.color);
  }
}

//SCROLLS LIGHTS ALONG STRAND, STARTS AT MIDPOINT, COLOR IS FREQUENCY DEFINED
void ScrollingFTTLights() {
  FFTbasedLights();
  if (colorToReturn.valid) {
    for (int i = 0; i < NEOLEN / 2; i++) {
      pixels.setPixelColor(i, pixels.getPixelColor(i + 1));
      pixels.setPixelColor(NEOLEN - i-1, pixels.getPixelColor(NEOLEN - i - 2));
    }
    pixels.setPixelColor(NEOLEN / 2, colorToReturn.color);
  }
}

//SCROLLS LIGHTS ALONG STRAND, STARTS AT MIDPOINT, COLOR IS RANDOMLY DEFINED
void ScrollingRandomLights() {
  for (int i = 0; i < NEOLEN / 2; i++) {
    pixels.setPixelColor(i, pixels.getPixelColor(i + 1));
    pixels.setPixelColor(NEOLEN - i-1, pixels.getPixelColor(NEOLEN - i - 2));
  }
  pixels.setPixelColor(NEOLEN / 2, HSVtoColor(random(255), 1, 1));
}

//SETS UP NEW COLOR BASED ON FFT
void FFTbasedLights() {
  colorToReturn.valid = false; 
  if (millis() > (lastBinMove + (MEMORY / 4))) {
    lastBinMove = millis();
    maxRunningBin[3] = maxRunningBin[2];
    maxRunningBin[2] = maxRunningBin[1];
    maxRunningBin[1] = maxRunningBin[0];
    maxRunningBin[0] = 0;
    minRunningBin[3] = minRunningBin[2];
    minRunningBin[2] = minRunningBin[1];
    minRunningBin[1] = minRunningBin[0];
    minRunningBin[0] = 600;
  }
  int maxMaxRunningBin = 0;
  int minMinRunningBin = 600;
  for (int i = 0; i < 4; i++) {
    if (maxRunningBin[i] > maxMaxRunningBin) {
      maxMaxRunningBin = maxRunningBin[i];
    }
    if (minRunningBin[i] < minMinRunningBin) {
      minMinRunningBin = minRunningBin[i];
    }
  }
  int i;
  float n;
  float maxn;
  int maxbin;
  if (fft.available()) {
    maxbin = 0;
    maxn = 0;
    for (i = 3; i < 511; i++) {
      n = fft.read(i);
      if (n > maxn) {
        maxn = n;
        maxbin = i;
      }
    }
    if (maxn >= .01) {
      lastRead = millis();

      if (maxbin > maxMaxRunningBin) {
        maxRunningBin[0] = maxbin;
        maxMaxRunningBin = maxbin;
      }
      if (maxbin < minMinRunningBin) {
        minRunningBin[0] = maxbin;
        minMinRunningBin = maxbin;
      }
      #ifdef FREQINFO
        Serial.print("\nFFT: ");
        Serial.print("maxn: ");
        Serial.print(maxn, 10);
        Serial.print("\tmaxbin: ");
        Serial.print(maxbin);
        Serial.print("\tBinRange: (");
        Serial.print(minMinRunningBin);
        Serial.print(", ");
        Serial.print(maxMaxRunningBin);
        Serial.println(")");
      #endif
      float h = (((float)maxbin - (float)minMinRunningBin) / ((float)maxMaxRunningBin - (float)minMinRunningBin)) * ((float)DEFAULTHFH - (float)DEFAULTLFH) + DEFAULTLFH;
      #ifdef COLORINFO
        Serial.print("H value for color: ");
        Serial.print(h);
        Serial.print('\t');
      #endif
      colorToReturn.valid = true;
      colorToReturn.color = HSVtoColor(h, DEFAULTS, DEFAULTV);
    }
  }
}

//takes in HSV values and outputs a neopixel color
uint32_t HSVtoColor(float h, float s, float v) {
  float c = v * s;
  float a = h / 60;
  float x = c * (1 - abs((a - (2 * floor(a / 2))) - 1));
  float m = v - c;
  float r = 0;
  float g = 0;
  float b = 0;
  if ((0 <= h) & (h < 60)) {
    r = c;
    g = x;
    b = 0;
  } else if ((60 <= h) & (h < 120)) {
    r = x;
    g = c;
    b = 0;
  } else if ((120 <= h) & (h < 180)) {
    r = 0;
    g = c;
    b = x;
  } else if ((180 <= h) & (h < 240)) {
    r = 0;
    g = x;
    b = c;
  } else if ((240 <= h) & (h < 300)) {
    r = x;
    g = 0;
    b = c;
  } else if ((300 <= h) & (h < 360)) {
    r = c;
    g = 0;
    b = x;
  }
  #ifdef COLORINFO
    Serial.print("R,G,B: ");
    Serial.print(r * 255 + m);
    Serial.print(", ");
    Serial.print(g * 255 + m);
    Serial.print(", ");
    Serial.println(b * 255 + m);
  #endif
  return pixels.Color(r * 255 + m, g * 255 + m, b * 255 + m);
}

//Changing Mode using button
void modeChange() {
  if (!buttonPressed & (digitalRead(MODEBUTTON) == LOW)) { //when the button is first pressed
    buttonPressed = true;
    buttonOn = millis();
    quitCharging = true;
    #ifdef STATEINFO
      Serial.println("rising edge");
    #endif
  } else if (buttonPressed & (digitalRead(MODEBUTTON) == HIGH) & (millis() < buttonOn + MINBUTTON))  {
    buttonPressed = false;
    #ifdef STATEINFO
      Serial.println("falsePress");
    #endif
  } else if (buttonPressed & (millis() > (buttonOn + SLEEPPRESS)) & (digitalRead(MODEBUTTON) == HIGH)) { //the button is pressed long enough to go to sleep
    state = SLEEP;
    buttonPressed = false;
    #ifdef STATEINFO
      Serial.println("going to sleep");
    #endif
  } else if (buttonPressed & (digitalRead(MODEBUTTON) == HIGH) & (millis() > (DEBOUNCE + buttonOn))) {
    buttonPressed = false;
    state++;
    if (state > NUMSTATES) {
      state = 1;
    }
    #ifdef STATEINFO
      Serial.print("changed state to: ");
      Serial.println(state);
    #endif
  } else if (quitCharging & (digitalRead(CHARGINGPIN) == LOW) & (digitalRead(CHARGEDPIN) == LOW)) {
    quitCharging = false;
  } else if (((state == CHARGING) | (state == CHARGED)) & (digitalRead(CHARGINGPIN) == LOW) & (digitalRead(CHARGEDPIN) == LOW)) {
    state = SLEEP;
  } else if (!quitCharging & (digitalRead(CHARGINGPIN) == HIGH)) {
    state = CHARGING;
  } else if (!quitCharging & (digitalRead(CHARGEDPIN) == HIGH)) {
    state = CHARGED;
  }
}
