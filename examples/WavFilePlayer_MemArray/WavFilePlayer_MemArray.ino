// Simple WAV file player example
//
// This example code is in the public domain.
//
// Plays a file from program flash
//
//
#include <play_wav.h>

#include <MemFile.h> // https://github.com/FrankBoesing/MemFile/

#include "gong.h"

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

MemFS  myfs;
File memFile;

void setup() {
  Serial.begin(9600);
  if (CrashReport) {
    Serial.println(CrashReport);
    CrashReport.clear();
  }

  AudioMemory(20);

  Serial.println("Playing...");
  memFile = myfs.open((char*)&_ac86773__juskiddink__gong, sizeof(_ac86773__juskiddink__gong), FILE_READ);
  playWav.play(memFile);
}

void loop()
{
}
