/*
 * AudioWavPlaySD_MidiPiano example
 * (c) Frank B. jan/2022
 * 
 * Sample loaded from https://theremin.music.uiowa.edu/MISpiano.html
 * and processed with ffmpeg to make them louder and to remove silence from the beginnings
 * 
 * Howto:
 * - Connect a MIDI keyboard to the Teensy 4.1 USB Host
 * - Use SDFORMAT to format a FAST SD Card - recommended is a Kingston Canvas Go! Plus, >=64GB
 * - Create a Folder "Piano" and copy all *.aiff files to the folder
 * 
 */
#include <Audio.h>
#include <SD.h>
#include <USBHost_t36.h>

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

USBHost myusb;
USBHub hub1(myusb);
USBHub hub2(myusb);
MIDIDevice midi1(myusb);

const int numConcurrendFiles = 14;

typedef struct {
  AudioPlayWav* player;
  unsigned long millis;
  int note;
} tPlayer;

tPlayer plnotes[numConcurrendFiles] = {
  { &playWav1, 0, -1}, { &playWav2, 0, -1}, { &playWav3, 0, -1}, { &playWav4, 0, -1},
  { &playWav5, 0, -1}, { &playWav6, 0, -1}, { &playWav7, 0, -1}, { &playWav8, 0, -1},
  { &playWav9, 0, -1}, {&playWav10, 0, -1}, {&playWav11, 0, -1}, {&playWav12, 0, -1},
  {&playWav13, 0, -1}, {&playWav14, 0, -1}
};

const int maxlen_filename = 32;
const char* notes[12] = {"C", "Db", "D", "Eb", "E", "F", "Gb", "G", "Ab", "A", "Bb", "B"};
const int octave_min = 0;
const int octave_max = 7; //0..7 = 8
const int midiChannel = 1;
const int velocity_max = 2; //0..2 = 3

char filename[velocity_max + 1][octave_max + 1][12][maxlen_filename + 1];

FLASHMEM
void createFilenames(const char* velocityStr, int velocity)
{
  for (uint octave = 0; octave < octave_max + 1; octave++)
    for (uint note = 0; note < 12; note++) {
      snprintf(filename[velocity][octave][note], maxlen_filename, "Piano/Piano.%s.%s%d.aiff", velocityStr, (char*)notes[note], octave);
      if (!SD.exists(filename[velocity][octave][note]))
        filename[velocity][octave][note][0] = 0;
      // else Serial.println(filename[velocity][octave][note]);
    }
}

FLASHMEM
void setup()
{
  AudioMemory(40);

  Serial.begin(9600);
  while (millis() < 3000 && !Serial); // wait for Arduino Serial Monitor

  Serial.println("Midi Piano Example");
  Serial.print("Initializing SD card...");

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("initialization failed!");
    abort();
  }
  Serial.println("SD initialization done.");

  createFilenames("pp", 0);
  createFilenames("mf", 1);
  createFilenames("ff", 2);

  //You may want to setup the mixers here

  myusb.begin();
  midi1.setHandleNoteOn(OnNoteOn);
  midi1.setHandleNoteOff(OnNoteOff);
  midi1.setHandleControlChange(OnControlChange);
  
  Serial.println("Ready to play ;)");
}


int getNextPlayer(int note)
{
  unsigned long oldestTime = 0x7fffffff;
  int next = 0;

  for (uint i = 0; i < numConcurrendFiles; i++) {
    /*
        - On a real piano you use the same string, of course, to play if the same note is played again.
        Restarting the same player however results in a audible click, sometimes -
        this could be fixed with an additional effort, but would go beyond the scope of this simple example.        
    */

#if 0
    if ( note == plnotes[i].note ) //if same note, restart same player
      return i;
#endif

    if ( plnotes[i].player->isStopped() )
      return i; //found a unused player

    if ( plnotes[i].millis < oldestTime ) {
      oldestTime = plnotes[i].millis;
      next = i;
    }
  }

  return next;
}


void playNote(byte note, byte velocity) {
  int octave = note / 12;
  if (octave > octave_max) return;
  int n = note % 12;
  int v = roundf(((float)(velocity_max) / 127) * velocity);

  if (filename[v][octave][n][0]) {
    int pl = getNextPlayer(note);
    plnotes[pl].player->play(filename[v][octave][n]);
    plnotes[pl].millis = millis();
    plnotes[pl].note = note;
    Serial.printf("Player: %i, Note: %s, Octave: %i, Velocity: %i, File: %s\n",
                  pl + 1, notes[n], octave, v, filename[v][octave][n]);
  } else {
    Serial.printf("Note: %s, Octave: %i, File: %s NOT FOUND\n",
                  notes[n], octave, filename[v][octave][n]);

  }
}



void OnNoteOn(byte channel, byte note, byte velocity)
{
  if (midiChannel == channel)
    playNote(note, velocity);
  else 
    Serial.printf("Unknown channel: %u\n", channel);
  //Serial.printf("Note On, ch=%u, note=%u, velocity=%u\n", channel, note, velocity);
}

void OnNoteOff(byte channel, byte note, byte velocity)
{
  //TODO... for you :) - beyond the scope here

#if 0
  Serial.printf("Note Off, ch=%u, note=%u, velocity=%u %s%u\n",
                channel, note, velocity, notes[note % 12], note / 12);
#endif
}

void OnControlChange(byte channel, byte control, byte value)
{
  //Todo... pedal, etc
#if 0
  Serial.print("Control Change, ch=");
  Serial.print(channel);
  Serial.print(", control=");
  Serial.print(control);
  Serial.print(", value=");
  Serial.print(value);
  Serial.println();
#endif
}


void loop()
{
  myusb.Task();
  midi1.read();
}
