
//Play 14 files simultanously
//
//Download the file "Mellotron Violin samples (zip)" from here: https://robertsonics.com/tsunami-downloads/

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

#include <play_wav.h>

// GUItool: begin automatically generated code
AudioPlayWav             playWav8;     //xy=206,433
AudioPlayWav             playWav9;     //xy=206,468
AudioPlayWav             playWav6;     //xy=207,359
AudioPlayWav             playWav10;    //xy=207,506
AudioPlayWav             playWav5;     //xy=208,321
AudioPlayWav             playWav13;    //xy=207,613
AudioPlayWav             playWav7;     //xy=208,396
AudioPlayWav             playWav14;    //xy=207,649
AudioPlayWav             playWav3;     //xy=209,241
AudioPlayWav             playWav4;     //xy=209,282
AudioPlayWav             playWav11;    //xy=208,543
AudioPlayWav             playWav1;     //xy=210,161
AudioPlayWav             playWav12;    //xy=209,579
AudioPlayWav             playWav2;     //xy=211,201
AudioMixer4              mixer1;         //xy=512,151
AudioMixer4              mixer2;         //xy=515,226
AudioMixer4              mixer3;         //xy=516,304
AudioMixer4              mixer4;         //xy=516,377
AudioMixer4              mixer5;         //xy=517,448
AudioMixer4              mixer6;         //xy=518,521
AudioMixer4              mixer7;         //xy=519,601
AudioMixer4              mixer8;         //xy=521,682
AudioMixer4              mixer9;         //xy=745,277
AudioMixer4              mixer10;        //xy=756,528
AudioOutputPT8211        pt8211_1;       //xy=934,393
AudioConnection          patchCord1(playWav8, 0, mixer2, 3);
AudioConnection          patchCord2(playWav8, 1, mixer6, 3);
AudioConnection          patchCord3(playWav9, 0, mixer3, 0);
AudioConnection          patchCord4(playWav9, 1, mixer7, 0);
AudioConnection          patchCord5(playWav6, 0, mixer2, 1);
AudioConnection          patchCord6(playWav6, 1, mixer6, 1);
AudioConnection          patchCord7(playWav10, 0, mixer3, 1);
AudioConnection          patchCord8(playWav10, 1, mixer7, 1);
AudioConnection          patchCord9(playWav5, 0, mixer2, 0);
AudioConnection          patchCord10(playWav5, 1, mixer6, 0);
AudioConnection          patchCord11(playWav13, 0, mixer4, 0);
AudioConnection          patchCord12(playWav13, 1, mixer8, 0);
AudioConnection          patchCord13(playWav7, 0, mixer2, 2);
AudioConnection          patchCord14(playWav7, 1, mixer6, 2);
AudioConnection          patchCord15(playWav14, 0, mixer4, 1);
AudioConnection          patchCord16(playWav14, 1, mixer8, 1);
AudioConnection          patchCord17(playWav3, 0, mixer1, 2);
AudioConnection          patchCord18(playWav3, 1, mixer5, 2);
AudioConnection          patchCord19(playWav4, 0, mixer1, 3);
AudioConnection          patchCord20(playWav4, 1, mixer5, 3);
AudioConnection          patchCord21(playWav11, 0, mixer3, 2);
AudioConnection          patchCord22(playWav11, 1, mixer7, 2);
AudioConnection          patchCord23(playWav1, 0, mixer1, 0);
AudioConnection          patchCord24(playWav1, 1, mixer5, 0);
AudioConnection          patchCord25(playWav12, 0, mixer3, 3);
AudioConnection          patchCord26(playWav12, 1, mixer7, 3);
AudioConnection          patchCord27(playWav2, 0, mixer1, 1);
AudioConnection          patchCord28(playWav2, 1, mixer5, 1);
AudioConnection          patchCord29(mixer1, 0, mixer9, 0);
AudioConnection          patchCord30(mixer2, 0, mixer9, 1);
AudioConnection          patchCord31(mixer3, 0, mixer9, 2);
AudioConnection          patchCord32(mixer4, 0, mixer9, 3);
AudioConnection          patchCord33(mixer5, 0, mixer10, 0);
AudioConnection          patchCord34(mixer6, 0, mixer10, 1);
AudioConnection          patchCord35(mixer7, 0, mixer10, 2);
AudioConnection          patchCord36(mixer8, 0, mixer10, 3);
AudioConnection          patchCord37(mixer9, 0, pt8211_1, 0);
AudioConnection          patchCord38(mixer10, 0, pt8211_1, 1);
// GUItool: end automatically generated code

const int numFiles = 14;

AudioPlayWav *player[numFiles] = {
  &playWav1,  &playWav2,  &playWav3, &playWav4,
  &playWav5,  &playWav6,  &playWav7, &playWav8,
  &playWav9,  &playWav10, &playWav11, &playWav12,
  &playWav13, &playWav14
};

const char *filename[numFiles] = {
  "045_A2.wav",
  "047_B2.wav",
  "048_C3.wav",
  "050_D3.wav",
  "052_E3.wav",
  "053_F3.wav",
  "055_G3.wav",
  "057_A3.wav",
  "059_B3.wav",
  "060_C4.wav",
  "062_D4.wav",
  "064_E4.wav",
  "065_F4.wav",
  "067_G4.wav",
};


int cnt = 0;

void setup() {
  Serial.begin(9600);
  AudioMemory(50);
  delay(500);
  if (CrashReport) {
    pinMode(13, OUTPUT);
    digitalWriteFast(13, 1);
    Serial.println(CrashReport);
    CrashReport.clear();
    delay(30000);
  }

  if (!(SD.begin(BUILTIN_SDCARD))) {
    // stop here, but print a message repetitively
    while (1) {
      Serial.println("Unable to access the SD card");
      delay(500);
    }
  }

  // set up mixers
  float gain = 1.0 / numFiles;
  for (int i = 0; i < 4; i++) {
    mixer1.gain(i, gain);
    mixer2.gain(i, gain);
    mixer3.gain(i, gain);
    mixer4.gain(i, gain);
    mixer5.gain(i, gain);
    mixer6.gain(i, gain);
    mixer7.gain(i, gain);
    mixer8.gain(i, gain);
  }

  gain = 1.0 / 4;
  for (int i = 0; i < 4; i++) {
    mixer9.gain(i, gain);
    mixer10.gain(i, gain);
  }

}

void loop() {

  for (int i = 0; i < numFiles; i++)
  {
    Serial.printf("Start %02d : %s\n", i + 1, filename[i]);
    player[i]->play(filename[i]);
    delay(500);
  }

  int playing;
  do {

    playing = 0;
    for (int i = 0; i < numFiles; i++)
    {
      if (player[i]->isPlaying())
        playing++;
    }

  } while (playing > 0);

  Serial.println("All stopped.");

}
