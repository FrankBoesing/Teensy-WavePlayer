
#include <Audio.h>
#include <play_wav.h>

#define xHAVE_PT8211

#if defined(HAVE_PT8211)
AudioSynthToneSweep myEffect;
AudioOutputPT8211   audioOutput;
AudioRecordWav      record;
AudioPlayWav        play;

AudioConnection c1(play, 0, audioOutput, 0);
AudioConnection c2(play, 0, audioOutput, 1);

AudioConnection c3(myEffect, 0, record, 0);
AudioConnection c4(myEffect, 0, record, 1);

#else

// GUItool: begin automatically generated code
AudioPlayWav             play;       //xy=288,291
AudioSynthToneSweep      myEffect;     //xy=288,382
AudioMixer4              mixer2;         //xy=460,316
AudioOutputI2S           i2s1;           //xy=473,252
AudioRecordWav record;   //xy=491,383
AudioConnection          patchCord1(play, 0, i2s1, 0);
AudioConnection          patchCord2(play, 1, mixer2, 0);
AudioConnection          patchCord3(play, 1, i2s1, 1);
AudioConnection          patchCord4(myEffect, 0, mixer2, 3);
AudioConnection          patchCord5(myEffect, 0, record, 0);
AudioConnection          patchCord6(myEffect, 0, record, 1);
AudioControlSGTL5000     sgtl5000_1;     //xy=466,205
// GUItool: end automatically generated code
#endif // defined(HAVE_PT8211)



const char filename[] = "test5.wav";

float t_ampx = 0.8;
int t_lox = 500;
int t_hix = 5000;
float t_timex = 0.5;// Length of time for the sweep in seconds

File file;

void setup(void)
{
  AudioMemory(20);
  
#if !defined(HAVE_PT8211)
  sgtl5000_1.enable();
  sgtl5000_1.volume(0.1);
#endif // defined(HAVE_PT8211)

  Serial.begin(9600);

  delay(1000);
  if (CrashReport) {
    Serial.println(CrashReport);
    CrashReport.clear();
  }

  while (!SD.begin(BUILTIN_SDCARD)) 
  {
    Serial.println("SD: initialization failed!");
    delay(500);
  }


#if 1
  SD.remove(filename);
  file = SD.open(filename, FILE_WRITE_BEGIN);

  Serial.println("Record effect (silent):");
  AudioNoInterrupts();
  record.record(file, APW_16BIT_SIGNED, 2);
  if (!myEffect.play(t_ampx, t_lox, t_hix, t_timex)) {
    Serial.println("AudioSynthToneSweep - begin failed"); Serial.flush();
    while (1)
      ;
  }
  AudioInterrupts();
  // wait for the sweep to end
  while (myEffect.isPlaying())
  {
    Serial.print('.');
    delay(250);
  }
    

  // and now reverse the sweep
  if (!myEffect.play(t_ampx, t_hix, t_lox, t_timex)) {
    Serial.println("AudioSynthToneSweep - begin failed"); Serial.flush();
    while (1);
  }
  // wait for the sweep to end
  while (myEffect.isPlaying());
  //Stop recording
  record.stop();
  file.close();

  Serial.println("Done");
#endif

  Serial.println("Play:");
  play.play(filename);
}

void loop(void)
{
}
