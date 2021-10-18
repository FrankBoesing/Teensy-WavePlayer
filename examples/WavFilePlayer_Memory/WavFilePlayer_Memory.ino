// Simple WAV file player example
//
// This example code is in the public domain.
//
// Plays a file from PSRAM
//
//
#include <play_wav.h>

#include <MemFile.h> // https://github.com/FrankBoesing/MemFile/

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// GUItool: begin automatically generated code
AudioPlayWav             playWav;     //xy=323,171
AudioOutputPT8211        output;       //xy=828,169
AudioConnection          patchCord1(playWav, 0, output, 0);
AudioConnection          patchCord2(playWav, 1, output, 1);
// GUItool: end automatically generated code


const char *filename = "sine440.wav";

extern uint8_t external_psram_size;

MemFS  myfs;
File sdFile;
File memFile;

void setup() {
  Serial.begin(9600);
  if (CrashReport) {
    Serial.println(CrashReport);
    CrashReport.clear();
  }

  AudioMemory(20);

  //1. Copy the file from SD to PSRAM:

  if (!(SD.begin(BUILTIN_SDCARD))) {
    Serial.println("Unable to access the SD card");
    abort();
  }

  Serial.println("Copy file...");


  //File open
  sdFile = SD.open(filename);
  size_t len = sdFile.size();

  if (len == 0) {
    Serial.print(filename);
    Serial.println(" not readable");
    abort();
  }

  if (len > external_psram_size * 1024 * 1024) {
    Serial.print(filename);
    Serial.println(" is too large");
    abort();
  }

  //2. File copy
  char *p = (char *)0x70000000;
  sdFile.read(p, len);
  sdFile.close();

  //3. Play
  Serial.println("Playing...");
  memFile = myfs.open(p, len, FILE_READ);
  playWav.play(memFile);
}

void loop()
{
}
