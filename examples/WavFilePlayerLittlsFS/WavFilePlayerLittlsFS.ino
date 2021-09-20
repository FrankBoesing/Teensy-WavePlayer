// Simple WAV file player example
//
// Three types of output may be used, by configuring the code below.
//
//   1: Digital I2S - Normally used with the audio shield:
//         http://www.pjrc.com/store/teensy3_audio.html
//
//   2: Digital S/PDIF - Connect pin 22 to a S/PDIF transmitter
//         https://www.oshpark.com/shared_projects/KcDBKHta
//
//   3: Analog DAC - Connect the DAC pin to an amplified speaker
//         http://www.pjrc.com/teensy/gui/?info=AudioOutputAnalog
//
// To configure the output type, first uncomment one of the three
// output objects.  If not using the audio shield, comment out
// the sgtl5000_1 lines in setup(), so it does not wait forever
// trying to configure the SGTL5000 codec chip.
//
// The SD card may connect to different pins, depending on the
// hardware you are using.  Uncomment or configure the SD card
// pins to match your hardware.
//
// Data files to put on your SD card can be downloaded here:
//   http://www.pjrc.com/teensy/td_libs_AudioDataFiles.html
//
// This example code is in the public domain.
#include <LittleFS.h>
LittleFS_QSPIFlash myfs;
uint64_t fTot, totSize1;

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

// GUItool: begin automatically generated code
AudioPlaySdWav           playSdWav1;     //xy=323,171
AudioMixer4              mixer1;         //xy=647,123
AudioMixer4              mixer3;         //xy=648,212
//AudioOutputPT8211        pt8211_1;       //xy=828,169
AudioOutputI2S           audioOutput;
AudioConnection          patchCord1(playSdWav1, 0, mixer1, 0);
AudioConnection          patchCord2(playSdWav1, 1, mixer3, 0);
AudioConnection          patchCord3(playSdWav1, 2, mixer1, 1);
AudioConnection          patchCord4(playSdWav1, 3, mixer3, 1);
AudioConnection          patchCord5(playSdWav1, 4, mixer1, 2);
AudioConnection          patchCord6(playSdWav1, 5, mixer3, 2);
AudioConnection          patchCord7(playSdWav1, 6, mixer1, 3);
AudioConnection          patchCord8(playSdWav1, 7, mixer3, 3);
AudioConnection          patchCord9(mixer1, 0, audioOutput, 0);
AudioConnection          patchCord10(mixer3, 0, audioOutput, 1);

AudioControlSGTL5000     sgtl5000_1;

// GUItool: end automatically generated code

#define SDCARD_CS_PIN    BUILTIN_SDCARD
#define SDCARD_MOSI_PIN  11  // not actually used
#define SDCARD_SCK_PIN   13  // not actually used


extern uint32_t _AudioPlaySdWavInstances;

void setup() {
  Serial.begin(9600);
  if (CrashReport) {
    Serial.println(CrashReport);
    CrashReport.clear();
  }
  Serial.printf("Instances: %d\n", (int)_AudioPlaySdWavInstances);
  Serial.println(_AudioPlaySdWavInstances);

  AudioMemory(50);
  // Comment these out if not using the audio adaptor board.
  // This may wait forever if the SDA & SCL pins lack
  // pullup resistors
  sgtl5000_1.enable();
  sgtl5000_1.volume(0.5);

  Serial.println("LittleFS Test"); delay(5);
  if (!myfs.begin()) {
  //if (!myfs.begin(chipSelect)) {
    Serial.println("Error starting qspidisk");
    while (1) ;
  }


  Serial.printf("TotalSize (Bytes): %d\n", myfs.totalSize());
  //myfs.deviceErase();

  delay(1000);
  Serial.println("started");
  printDirectory();


  //SPI.setMOSI(SDCARD_MOSI_PIN);
  //SPI.setSCK(SDCARD_SCK_PIN);
  //if (!(SD.begin(SDCARD_CS_PIN))) {
    // stop here, but print a message repetitively
  //  while (1) {
  //    Serial.println("Unable to access the SD card");
  //    delay(500);
  //  }
  //}
}

void playFile(const char *filename)
{
  Serial.print("Playing file: ");
  Serial.println(filename);

  // Start playing the file.  This sketch continues to
  // run while the file plays.
  File ccc = myfs.open(filename);  ///Change to play off QSPI flash
  playSdWav1.play(ccc);

  // Simply wait for the file to finish playing.
  while (playSdWav1.isPlaying()) {
  }
}


void loop() {

  playFile("Nums_7dot1_16_44100.wav");
  delay(500);
  //playFile("SDTEST1.WAV");  // filenames are always uppercase 8.3 format
  //delay(500);
  //playFile("SDTEST2.WAV");
  //delay(500);
  //playFile("SDTEST3.WAV");
  //delay(500);
  //playFile("SDTEST4.WAV");
  //delay(1500);
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
      Serial.printf("\n %u dirs with %u files of Size %u Bytes\n", dCnt, fCnt, fSize);
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
