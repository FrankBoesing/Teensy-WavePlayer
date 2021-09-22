#include <Audio.h>
#include <play_wav.h>

AudioPlayWav             play;           //xy=176.88333129882812,230.88333129882812
AudioInputUSB            usb1;           //xy=182.88333129882812,132.88333129882812
AudioMixer4              mixer2;         //xy=389.8833312988281,251.88333129882812
AudioMixer4              mixer1;         //xy=392.8833312988281,133.88333129882812
AudioOutputPT8211        pt8211_1;       //xy=597.88330078125,163.88333129882812
AudioRecordWav           record;         //xy=598.88330078125,258.8833312988281
AudioConnection          patchCord1(usb1, 0, mixer1, 0);
AudioConnection          patchCord2(usb1, 1, mixer2, 0);
AudioConnection          patchCord3(play, 0, mixer1, 1);
AudioConnection          patchCord4(play, 1, mixer2, 1);
AudioConnection          patchCord5(usb1, 0, record, 0);
AudioConnection          patchCord6(usb1, 1, record, 1);
AudioConnection          patchCord7(mixer1, 0, pt8211_1, 0);
AudioConnection          patchCord8(mixer2, 0, pt8211_1, 1);


#if !defined(USB_AUDIO)
#error Please select USB type "USB-AUDIO"
#endif
#if AUDIO_BLOCK_SAMPLES < 256
#error Please set AUDIO_BLOCK_SAMPLES >= 256  (in AudioStream.h)
#endif


const char filename[] = "usbAudio.wav";
int tRec = 30; // Seconds to record.


void setup() {
  AudioMemory(20);
  Serial.begin(9600);
  delay(600);

  if (CrashReport) {
    Serial.println(CrashReport);
    CrashReport.clear();
  }

  mixer1.gain(0, 1);
  mixer1.gain(1, 1);
  mixer2.gain(0, 0);
  mixer2.gain(1, 0);

  Serial.println("Audio recording example.\n");

  Serial.printf("Audio Block: %d Bytes.\n", AUDIO_BLOCK_SAMPLES);
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD: initialization failed!");
    abort();
  }

  size_t t, tlast = 0;

#if 1
  Serial.println("If you don't hear audio, check your PC's settings and play an audio file.");
  Serial.printf("Please wait %d seconds.\n", tRec);

  SD.remove(filename); //Delete a probably existing file
  if (!record.record(filename, APW_16BIT_SIGNED, 2)) {
    Serial.printf("Could not start recording. Error: %d\n", record.lastErr());
    abort();
  }

  Serial.println("Recording started.");

  while ((t = record.lengthMillis()) < tRec * 1000) {

    t = t / 1000;
    if (t != tlast) {
      Serial.printf("%d. ", t);
      tlast = t;
    }
    if ((float)AudioProcessorUsageMax() > 10.0) {
      Serial.printf("\nDetected longer than usual card wait time. Max processor usage was: %0.2f%%\n", AudioProcessorUsageMax());
      AudioProcessorUsageMaxReset();
    }
    delay(3);
  }

  record.stop();
#endif

  Serial.println("Recording stopped.");
  Serial.println();
  Serial.println("Play:");

  mixer1.gain(0, 0);
  mixer2.gain(0, 0);
  mixer1.gain(1, 1);
  mixer2.gain(1, 1);

  play.play(filename);
  
  while (play.isPlaying()) {
    t = play.positionMillis() / 1000;
    if (t != tlast) {
      Serial.printf("%d. ", t);
      tlast = t;
    }
    if ((float)AudioProcessorUsageMax() > 10.0) {
      Serial.printf("\nDetected longer than usual card wait time. Max processor usage was: %0.2f%%\n", AudioProcessorUsageMax());
      AudioProcessorUsageMaxReset();
    }
    delay(3);
  }
  Serial.println("Done.");


  Serial.println("Output is now USB Audio...");
  delay(1000);
  mixer1.gain(0, 0.5);
  mixer2.gain(0, 0.5);
  mixer1.gain(1, 0);
  mixer2.gain(1, 0);
}

void loop() {


}
