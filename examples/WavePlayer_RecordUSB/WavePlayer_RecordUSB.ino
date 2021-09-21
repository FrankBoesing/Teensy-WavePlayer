#include <Audio.h>
#include <play_wav.h>

AudioPlayWav             play;           //xy=176.88333129882812,230.88333129882812
AudioInputUSB            usb1;           //xy=182.88333129882812,132.88333129882812
AudioMixer4              mixer2;         //xy=389.8833312988281,251.88333129882812
AudioMixer4              mixer1;         //xy=392.8833312988281,133.88333129882812
AudioOutputPT8211        pt8211_1;       //xy=597.88330078125,163.88333129882812
AudioRecordWav           record;           //xy=598.88330078125,258.8833312988281
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

void setup() {
 AudioMemory(20);
 Serial.begin(9600);
 
 if (CrashReport) {
  Serial.println(CrashReport);
  CrashReport.clear();
 }
 
 mixer1.gain(0, 0.5);
 mixer1.gain(1, 0.5);
 mixer2.gain(0, 0.5);
 mixer2.gain(1, 0.5);
 
 if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD: initialization failed!");
    return;
 }
 SD.remove(filename);

 int tRec = 120;
 Serial.printf("Start recording... %d seconds\n", tRec);
 
 unsigned long t = millis();
 record.record(filename, APW_16BIT_SIGNED, 2);
 
 int i = 1;
 while (millis() - t < tRec * 1000) { delay(1000); Serial.printf("%d ", i++); }
 record.stop();
 Serial.println();
 
 mixer1.gain(0,0);
 mixer2.gain(0,0);
 Serial.printf("\nPlay\n");
 play.play(filename);
 i = 1;
 while (play.isPlaying()) { delay(1000); Serial.printf("%d ", i++); }
 Serial.println("Done.");

 Serial.println("Output is now USB Audio...");
 mixer1.gain(0,0.5);
 mixer2.gain(0,0.5);
}

void loop() {
 

}
