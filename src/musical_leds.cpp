#include <Arduino.h>

#include <Adafruit_NeoPixel.h>
#include <arduinoFFT.h>

arduinoFFT FFT;

#define DEBUG 0
#define SAMPLES 512

#define PIN D0
#define NUMPIXELS 5 * 30

uint32_t hsv_slide[NUMPIXELS];

unsigned long last_loop_time = 0;
unsigned long current_time = 0;

Adafruit_NeoPixel pixels(NUMPIXELS, PIN);

#include <PDM.h>

// default number of output channels
static const char channels = 1;

// default PCM output frequency
static const int frequency = 16000;

// Buffer to read samples into, each sample is 16-bits
short sampleBuffer[512];

// Number of audio samples read
volatile int samplesRead;

double vReal[SAMPLES];
double vImag[SAMPLES];

void onPDMdata() {
  // Query the number of available bytes
  int bytesAvailable = PDM.available();

  // Read into the sample buffer
  PDM.read(sampleBuffer, bytesAvailable);

  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;
}

void setup() {

  Serial.begin(115200);

  for (int i = 0; i < NUMPIXELS; i++) {
    hsv_slide[i] = 0;
  }

  PDM.onReceive(onPDMdata);

  // Optionally set the gain
  // Defaults to 20 on the BLE Sense and 24 on the Portenta Vision Shield
  // PDM.setGain(30);

  // Initialize PDM with:
  // - one channel (mono mode)
  // - a 16 kHz sample rate for the Arduino Nano 33 BLE Sense
  // - a 32 kHz or 64 kHz sample rate for the Arduino Portenta Vision Shield
  if (!PDM.begin(channels, frequency)) {
    Serial.println("Failed to start PDM!");
    while (1)
      ;
  }
}

void shift() {
  for (int i = NUMPIXELS - 1; i > 0; i--) {
    hsv_slide[i] = hsv_slide[i - 1];
    // hsv_slide[i-1] = hsv_slide[i] / 8; //FIXME
  }
  hsv_slide[0] = 0;
}

void light_up_the_bar(double volume, double frequency) {
  double cutoff, scale, volume_scale;
  uint16_t hue;

  cutoff = 100;
  scale = UINT16_MAX /
          (log2(3400 - cutoff)); // We assume we want to map voice between 50Hz
                                 // and 7kHz using logarithmic scale

  hue = (uint16_t)round((log2(frequency - cutoff)) * scale) % UINT16_MAX;
  if (volume > 150) {
    volume = 150;
  }
  volume_scale = UINT8_MAX / (log2(150 - 20));
  uint8_t value = (uint8_t)round((log2(volume - 20) * volume_scale));
  if (volume > 30) {
    uint8_t saturation = 255;
#ifdef DEBUG
    Serial.println(frequency);
    Serial.print("Hue: ");
    Serial.print(hue);
    Serial.print(", Saturation: ");
    Serial.print(saturation);
    Serial.print(", Value: ");
    Serial.println(value);
#endif
    // We want some separation between the bursts. Max burst lenght: 4, then 8
    // pixels of space
    if (hsv_slide[4] == 0 && hsv_slide[5] == 0 && hsv_slide[6] == 0 &&
        hsv_slide[7] == 0 && hsv_slide[8] == 0 && hsv_slide[9] == 0 &&
        hsv_slide[10] == 0 && hsv_slide[11] == 0) {
      hsv_slide[0] = pixels.ColorHSV(hue, value, value);
    }
  }
}

void loop() {
  current_time = micros();
  if (samplesRead) {
    double sum_squares = 0.0;
    for (int i = 0; i < SAMPLES; i++) {
      vReal[i] = sampleBuffer[i] / 32767.0;
      vImag[i] = 0;
      sum_squares += pow(sampleBuffer[i], 2);
    }
    double rms = sqrt(sum_squares / SAMPLES);
    FFT = arduinoFFT(vReal, vImag, SAMPLES, frequency);
    FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(FFT_FORWARD);
    FFT.ComplexToMagnitude();
    double peak = FFT.MajorPeak();
#ifdef DEBUG
    Serial.print("Frequency:");
    Serial.print(peak);
    Serial.print(" (log: ");
    Serial.print(log(peak));
    Serial.print(") volume: ");
    Serial.println(rms);
#endif

    light_up_the_bar(rms, peak);
#ifdef DEBUG
    // Print samples to the serial monitor or plotter
    for (int i = 0; i < samplesRead; i++) {
      if (channels == 2) {
        Serial.print("L:");
        Serial.print(sampleBuffer[i]);
        Serial.print(" R:");
        i++;
      }
      Serial.println(sampleBuffer[i]);
    }
#endif

    // Clear the read count
    samplesRead = 0;
  }

  if (current_time - last_loop_time > 1UL) {
    last_loop_time = current_time;
    shift();
  }

  for (int i = 0; i < NUMPIXELS; ++i) {
    pixels.setPixelColor(i, pixels.gamma32(hsv_slide[i]));
  }
  pixels.show();
}
