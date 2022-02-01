# Teensy-WavePlayer

This is an extended file player audio object, combining the functions of AudioPlaySdWav and AudioPlaySdRaw
with additional stability and functionality.

## Features

- Sample rate agnostic (does not check it)
- (up to) 8 Channels / 8- or 16-bit
- delay() after start no longer needed
- all audio block sizes
- interleaved reads / writes: only one file access on each audio-cycle
- lastErr() returns last error
- synchronized start


## Play: Formats
- n channel 8 bit unsigned *.wav
- n channel u-law *.wav
- n channel 16 bit signed *.wav
- n channel 24 bit signed *.wav
- n channel 8 bit signed *.aiff
- n channel 8 bit unsigned *.aifc
- n channel u-law *.aifc (Apple, non ccitt)
- n channel 16 bit signed *.aiff
- n channel 8 bit signed, unsigned or u-law RAW
- n channel 16 bit signed, unsigned or 16 bit big-endian signed RAW

## Record: Formats
- n channel 8 bit unsigned *.wav
- n channel 16 bit signed *.wav

---
## Updated GUI

An updated `index.html` file is supplied which documents the key API calls provided by AudioPlayWav. The object placed on the design area shows 8 outputs, of which 1, 2, 4, 6 or 8 will output audio data, depending on the number of channels provided in the file [can a RAW file have 3, 5 or 7 channels?]

---
## Main functions
#### play(File | filename [,paused] [,autorewind])
Plays the `File` object or named file; setting the optional `paused` parameter to `true` allocates and pre-loads the buffer, but does not start playing. Buffer memory is allocated when this function is called, with the amount dependent on the number of audio channels provided in the file.
If `autorewind` is `true`, it does not stop and close the file - instead it sets the state to `paused` and sets the first sample as new position.
#### bool pause(bool)
Starts a paused object if the parameter is `true`, or pauses it if `false`. Returns new state.
#### stop()
Stops playing, whether or not it is paused. The file is closed and buffer memory returned to the heap.
#### isPlaying()
Return true (non-zero) if playing, or false (zero) when stopped or paused.  
#### isStopped()
Return true (non-zero) if stopped, or false (zero) when playing or paused.  
#### isPaused()
Return true (non-zero) if paused, or false (zero) when playing or stopped.  
#### positionMillis()
While playing, return the current time offset, in milliseconds.  When not playing, the return from this function is undefined.
#### lengthMillis()
Return the total length of the current sound clip,in milliseconds. When not playing, the return from this function is undefined.
#### setPosition(sample)
Works in paused mode only. Sets the position to `sample`.
#### loop(bool)
If `true`, it repeats the whole file, endless. If set to `false` later, the looping stops and the file plays to the end.
#### loop(firstSample, lastSample, count)
When `lastSample` is reached, it rewinds back to `firstSample`. `Count` is the number of repetitions.
The loops can be stopped by using `count = 0` or calling `loop(false)`
#### unsigned int loopCount(void)
Returns the number of remaining loops.
#### lasterr()
If an internal error occurs, for example during a call to `play()` or within the audio update loop interrupt, this function will return the most recent error code. Possible values are:
- 0: no error
- 1: file format unsupported
- 2: file unredable (may not exist?)
- 3: insufficient memory to allocate buffer etc.
- 4: insufficient available audio blocks
---
## Detailed description
For the purposes of this documentation, it is assumed that the audio engine is compiled without changing the default audio block size (128 samples) or sample rate (44.1kHz), resulting in an update cycle of approximately 2.9ms. However, the AudioPlayWav object is intended to work with other block sizes and samples rates - please raise an issue if you find a combination that doesn't work, and you think that it should...
### Interleaved reads
By careful choice of buffer sizes and pre-load amount, the AudioPlayWav objects attempt to ensure that only one SD card read is attempted on each audio engine update. The buffer size required in order to do this is proportional to the total number of AudioPlayWav objects, the number of channels the WAV file contains, the sample size, and the audio block size. So for 3x 4-channel 16-bit files, 3 buffers of 4 x 2 x 128 bytes are needed, for a total allocation of 3kBytes of RAM. The SD card reads will occur every 2.9ms, and read in 1kByte of data.

### Synchronising playback start
In order to play multiple files in exact synchronisation, it is preferable to proceed as follows:
- call `play(<file>,true)` for all files, to pre-load their buffers without outputting any sound
- call `AudioNoInterrupts()` to prevent the audio interrupt firing
- call `pause(false)` for all the objects
- call `AudioInterrupts()` to allow the audio interrupt to fire: playback will start at the next interrupt


---
Example wave files taken from here:
https://www.jensign.com/bdp95/7dot1voiced/index.html
Converted to 16 bit using SoX (http://sox.sourceforge.net/):
`sox  Nums_5dot1_24_48000.wav -b 16 -r 44100 Nums_5dot1_16_44100.wav`

Sinewave files from here https://forum.pjrc.com/threads/67754-WaveplayerEx?p=286914&viewfull=1#post286914 (thanks to Jonathan )
