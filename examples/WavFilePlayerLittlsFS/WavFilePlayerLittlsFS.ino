// WAV file player example LittleFS
// This example code is in the public domain.

#include <LittleFS.h>

LittleFS_QPINAND myfs;
uint64_t fTot, totSize1;

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#include <play_wav.h>

// GUItool: begin automatically generated code
AudioPlayWav             playWav1;
AudioPlayWav             playWav2;
AudioPlayWav             playWav3;
AudioPlayWav             playWav4;
AudioPlayWav             playWav5;
AudioPlayWav             playWav6;
AudioMixer4              mixer1;
AudioMixer4              mixer2;
AudioOutputPT8211        audioOutput;
//AudioOutputI2S           audioOutput;
//AudioOutputMQS           audioOutput;
AudioConnection          patchCord1(playWav1, 0, mixer1, 0);
AudioConnection          patchCord2(playWav2, 0, mixer1, 1);
AudioConnection          patchCord3(playWav3, 0, mixer1, 2);
AudioConnection          patchCord4(playWav4, 0, mixer1, 3);
AudioConnection          patchCord5(playWav5, 0, mixer2, 0);
AudioConnection          patchCord6(playWav6, 0, mixer2, 1);
AudioConnection          patchCord7(mixer1, 0, audioOutput, 0);
AudioConnection          patchCord8(mixer2, 0, audioOutput, 1);

const int fileCnt = 6;
const char *filename[fileCnt] = {
  "102790__mhc__acoustic-open-hihat2.wav",
  "171104__dwsd__kick-gettinglaid.wav",
  "201159__kiddpark__cash-register.wav",
  "82583__kevoy__snare-drum-4.wav",
  "86334__zgump__tom-0104.wav",
  "86773__juskiddink__gong.wav"
};

AudioPlayWav *player[fileCnt] = {
  &playWav1, &playWav2, &playWav3, &playWav4,
  &playWav5, &playWav6
};

File file[fileCnt];

void setup() {
  Serial.begin(9600);
  if (CrashReport) {
    Serial.println(CrashReport);
    CrashReport.clear();
  }

  AudioMemory(20);
  Serial.println("LittleFS Test");
  if (!myfs.begin()) {
    Serial.println("Error starting qspidisk");
    while (1) ;
  }
  Serial.printf("TotalSize (Bytes): %d\n", myfs.totalSize());

  printDirectory();

  const float gain = 1.0f / 4;
  for (int i = 0; i < 4; i++) {
    mixer1.gain(i, gain);
    mixer2.gain(i, gain);
  }

  //Open all files in paused mode, autorewind
  for (int i = 0; i < fileCnt; i++)
  {
    file[i] = myfs.open(filename[i]);
    if (!file[i])
    {
      Serial.printf("Could not open \"%s\"\n", filename[i]);
      while (1);
    }
    if (!player[i]->play(file[i], true, true))
    {
      Serial.printf("Could not start \"%s\"\n", filename[i]);
      while (1);
    }
  }

  AudioProcessorUsageMaxReset();
  AudioMemoryUsageMaxReset();
}


void loop() {
  static int c = 0;
  static unsigned long t = millis();
  unsigned long m;
  if ( (m = millis()) - t > 200 )
  {
    t = m;

    if (player[c]->isPaused())
    {
      player[c]->pause(false);
    }

    Serial.printf("Proc = %0.2f (%0.2f),  Mem = %d (%d)\n",
                  AudioProcessorUsage(), AudioProcessorUsageMax(),
                  AudioMemoryUsage(), AudioMemoryUsageMax());

    AudioProcessorUsageMaxReset();
    AudioMemoryUsageMaxReset();

    if (++c >= fileCnt) c = 0;
  }

}

void printDirectory() {
  Serial.println("printDirectory\n--------------");
  printDirectory(myfs.open("/"), 0);
  Serial.println();
}


void printDirectory(File dir, int numTabs) {
  //dir.whoami();
  uint64_t fSize = 0;
  uint32_t dCnt = 0, fCnt = 0;
  if ( 0 == dir ) {
    Serial.printf( "\t>>>\t>>>>> No Dir\n" );
    return;
  }
  while (true) {
    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      //Serial.printf("\n %u dirs with %u files of Size %u Bytes\n", dCnt, fCnt, fSize);
      fTot += fCnt;
      totSize1 += fSize;
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');
    }

    if (entry.isDirectory()) {
      Serial.print("DIR\t");
      dCnt++;
    } else {
      Serial.print("FILE\t");
      fCnt++;
      fSize += entry.size();
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println(" / ");
      printDirectory(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
    //Serial.flush();
  }
}