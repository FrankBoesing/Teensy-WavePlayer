// Simple WAV file player example
//
// This example code is in the public domain.

#include <play_wav.h>


#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// GUItool: begin automatically generated code
AudioPlayWav             playWav;     //xy=323,171
AudioMixer4              mixer1;         //xy=647,123
AudioMixer4              mixer3;         //xy=648,212
AudioOutputPT8211        output;       //xy=828,169
AudioConnection          patchCord1(playWav, 0, mixer1, 0);
AudioConnection          patchCord2(playWav, 1, mixer3, 0);
AudioConnection          patchCord3(playWav, 2, mixer1, 1);
AudioConnection          patchCord4(playWav, 3, mixer3, 1);
AudioConnection          patchCord5(playWav, 4, mixer1, 2);
AudioConnection          patchCord6(playWav, 5, mixer3, 2);
AudioConnection          patchCord7(playWav, 6, mixer1, 3);
AudioConnection          patchCord8(playWav, 7, mixer3, 3);
AudioConnection          patchCord9(mixer1, 0, output, 0);
AudioConnection          patchCord10(mixer3, 0, output, 1);
// GUItool: end automatically generated code

#define SDCARD_CS_PIN    BUILTIN_SDCARD
#define SDCARD_MOSI_PIN  11  // not actually used
#define SDCARD_SCK_PIN   13  // not actually used

void setup() {
  Serial.begin(9600);
  if (CrashReport) {
    Serial.println(CrashReport);
    CrashReport.clear();
  }

  AudioMemory(20);

  SPI.setMOSI(SDCARD_MOSI_PIN);
  SPI.setSCK(SDCARD_SCK_PIN);
  if (!(SD.begin(SDCARD_CS_PIN))) {
    // stop here, but print a message repetitively
    while (1) {
      Serial.println("Unable to access the SD card");
      delay(500);
    }
  }
}

void playFile(const char *filename)
{
  Serial.print("Playing file: ");
  Serial.println(filename);
  playWav.play(filename);
  while (playWav.isPlaying()) {}
}


void loop() {
  //playFile("Nums_7dot1_16_44100.wav");
  playFile("Nums_7dot1_8_44100.wav");
  delay(500);
  playFile("SDTEST1.WAV");  // filenames are always uppercase 8.3 format
  delay(500);
  playFile("SDTEST2.WAV");
  delay(500);
  playFile("SDTEST3.WAV");
  delay(500);
  playFile("SDTEST4.WAV");
  delay(1500);
}