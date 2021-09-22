
#include <Audio.h>
#include <play_wav.h>

// this is intended to record an incoming I2S stream and simultaneously make it hearable in the I2S output
// tested with a T4 and PCM1808a
 

// GUItool: begin automatically generated code
AudioPlayWav             play;       //xy=288,291
AudioInputI2S            ADC1;     //xy=288,382
AudioMixer4              MixerL;
AudioMixer4              MixerR;

AudioOutputI2S           i2s1;           //xy=473,252
AudioRecordWav           record;   //xy=491,383

AudioConnection          patchCord1 (play, 0, MixerL, 0);
AudioConnection          patchCord11(play, 1, MixerR, 0);

AudioConnection          patchCord2 (ADC1, 0, MixerL, 1);
AudioConnection          patchCord22(ADC1, 1, MixerR, 1);

AudioConnection          patchCord3 (MixerL, 0, i2s1, 0);
AudioConnection          patchCord33(MixerR, 0, i2s1, 1);

AudioConnection          patchCord5(ADC1, 0, record, 0);
AudioConnection          patchCord6(ADC1, 1, record, 1);
AudioControlSGTL5000     sgtl5000_1;     //xy=466,205



#if AUDIO_BLOCK_SAMPLES < 256
#error Please set AUDIO_BLOCK_SAMPLES >= 256  (in AudioStream.h)
#endif


const char filename[] = "RecADC3.wav";

File file;


void playFile(const char *filename)
{
  Serial.print("Playing file: ");
  Serial.println(filename);
  play.play(filename);
  while (play.isPlaying()) 
  {
    delay(10); 
  }
}

#define RECORD_LENGTH 120 // no of seconds to record

void setup(void)
{
  AudioMemory(20);
  
#if !defined(HAVE_PT8211)
  sgtl5000_1.enable();
  sgtl5000_1.volume(0.8);
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

  Serial.println("Recording (what you hear is what is being recorded):");
  record.record(file, APW_16BIT_SIGNED, 2);

  // record
  for(unsigned idx=0;idx<=RECORD_LENGTH;idx++)
  {
    Serial.print("Recording: "); Serial.print(idx); Serial.println(" seconds");
    delay(1000);    
  }

  //Stop recording
  record.stop();
  file.close();

  Serial.println("Done, Recording finsihed");
#endif
  delay(1000);
  Serial.println("Playing the recorded file");
  playFile(filename);
}

void loop(void)
{
  delay(2000);
  Serial.println("Playing it again:");
  playFile(filename);
  Serial.println("Stopped playing");
}
