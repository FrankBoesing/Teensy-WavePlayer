/*
   Wavefile player
   Copyright (c) 2021, Frank Bösing, f.boesing @ gmx (dot) de

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

    Dieses Programm ist Freie Software: Sie können es unter den Bedingungen
    der GNU General Public License, wie von der Free Software Foundation,
    Version 3 der Lizenz oder (nach Ihrer Wahl) jeder neueren
    veröffentlichten Version, weiterverbreiten und/oder modifizieren.

    Dieses Programm wird in der Hoffnung, dass es nützlich sein wird, aber
    OHNE JEDE GEWÄHRLEISTUNG, bereitgestellt; sogar ohne die implizite
    Gewährleistung der MARKTFÄHIGKEIT oder EIGNUNG FÜR EINEN BESTIMMTEN ZWECK.
    Siehe die GNU General Public License für weitere Details.

    Sie sollten eine Kopie der GNU General Public License zusammen mit diesem
    Programm erhalten haben. Wenn nicht, siehe <http://www.gnu.org/licenses/>.

*/


#pragma once
#include <Arduino.h>

#if !defined(KINETISL)

#include <AudioStream.h>
#include <SD.h>

const int _AudioRecordWav_MaxChannels = 4;

enum APW_FORMAT { APW_8BIT_UNSIGNED = 0, APW_8BIT_SIGNED, APW_ULAW,
                  APW_16BIT_SIGNED, APW_16BIT_SIGNED_BIGENDIAN,
									APW_24BIT_SIGNED, APW_NONE
                };

enum APW_ERR	{ ERR_OK = 0,             // no Error
               ERR_FORMAT = 1,          // not supported Format
               ERR_FILE = 2,            // File not usable (does it exist?)
               ERR_OUT_OF_MEMORY = 3,   // Not enough dynamic memory available
               ERR_NO_AUDIOBLOCKS = 4		// insufficient # of available audio blocks
							};

#define APW_PREALLOCATE (60 * 60) // Recording: How much to preallocate (in Seconds)


/*********************************************************************************************************/

//#define DEBUG_PIN_PLAYWAV 0 //enable to view the timing on a scope

/*********************************************************************************************************/
// Helper class  - don't use
enum APW_FILETYPE : char {APW_FILE_SD, APW_FILE_OTHER, APW_FILE_NONE};

class apwFile
{
  public:
    apwFile(void) {reset();}
    void use(File *f);
    void open(const char *filename, int mode = FILE_READ);
    void close(void);
    size_t read(void *buf, size_t nbyte);
    size_t readInISR(void *buf, size_t nbyte);
    size_t write(void *buf, size_t nbyte);
    size_t writeInISR(void *buf, size_t nbyte);
    bool seek(size_t pos);
    size_t position(void);
    size_t size(void);
    void flush(void);
    bool preAllocate(uint64_t length);
    bool truncate(void);
    operator bool();
    void reset();
		bool isSD();
  private:
    File file;
    FsFile sdFile;
    APW_FILETYPE fileType = APW_FILE_NONE;
		size_t fpos;
};


/*********************************************************************************************************/
// Base clase - don't use

enum APW_STATE : char;

class AudioBaseWav
{
  public:
    virtual bool pause(const bool pause) = 0;
    bool togglePause(void);
    bool isPaused(void);
    bool isStopped(void);

    uint32_t numBits(void) {return bytes * 8;}
    uint32_t numChannels(void) {return channels;}
    uint32_t sampleRate(void) {return sample_rate;}
    uint8_t lastErr(void) {return (int)last_err;}

  private:
    friend class AudioPlayWav;
    friend class AudioRecordWav;

    AudioBaseWav(void);
    ~AudioBaseWav(void);

    static uint8_t _instances;
    static uint8_t _sz_mem_additional;

    void reset(void);
		size_t calcBufferIdx(void);
    bool createBuffer(void);
    void freeBuffer(void);
    void close(bool closeFile = true);

    bool isRunning(void);
    bool addMemory(size_t mult);

    apwFile wavfile;
    size_t sz_mem;              	// size of buffer
    int8_t* buffer;
    unsigned int sample_rate;
    APW_FORMAT dataFmt;
		APW_ERR last_err;
    APW_STATE state;     					// play status (stop, pause, running)
    uint8_t my_instance;          // instance id
    uint8_t bytes;            		// 1 or 2 bytes?
		uint8_t channels;        			// #of channels in the wave file
    int8_t updateStep = -1;

};

/*********************************************************************************************************/

typedef size_t (*_tEncoderDecoder)(int8_t [], audio_block_t *[], uint8_t);

class AudioPlayWav : public AudioBaseWav, public AudioStream
{
  public:
    AudioPlayWav(void) : AudioStream(0, NULL) {}
    void stop(void);

    bool play(File file, bool paused = false, bool autorewind = false);
    bool play(const char *filename, bool paused = false, bool autorewind = false); // play from SD
    bool playRaw(File file, APW_FORMAT fmt, uint32_t sampleRate, uint8_t number_of_channels, bool paused = false, bool autorewind = false);
    bool playRaw(const char *filename, APW_FORMAT fmt, uint32_t sampleRate, uint8_t number_of_channels, bool paused = false, bool autorewind = false); // from SD

    bool addMemoryForRead(size_t mult) {return addMemory(mult);} // add memory
    bool togglePlayPause(void) {return togglePause();}
    bool isPlaying(void) {return isRunning();}
		bool pause(bool pause);
    uint32_t positionMillis(void);
		size_t position();
		bool setPosition(size_t sample);
		void loop(bool enable);
		void loop(size_t firstSample, size_t lastSample, uint16_t count);
		int loopCount(void);
    uint32_t lengthMillis(void);
    uint32_t channelMask(void) {return channelmask;}
  private:
    bool _play(APW_FORMAT fmt, uint32_t sampleRate, uint8_t number_of_channels, bool paused, bool autorewind );
    bool readHeader(APW_FORMAT fmt, uint32_t sampleRate, uint8_t number_of_channels, APW_STATE newState );
		void stopFromUpdate(void);
		size_t dataReader(int8_t *buffer, int len);
    _tEncoderDecoder decoder;
    size_t total_length;      			// number of audio data bytes in file
		size_t firstSampleOffset;		    // file position of first sample
		size_t lastSample;							// file position of last sample
		size_t loopFirst, loopLast;
    uint32_t channelmask;           // dwChannelMask
		uint16_t _loopCount;
		bool autorewind;
		bool last;
		void start(void);
    virtual void update(void);
		size_t buffer_rd;
};
/*********************************************************************************************************/
#if 0
class AudioRecordWav : public AudioBaseWav, public AudioStream
{
  public:
    AudioRecordWav(void): AudioStream(_AudioRecordWav_MaxChannels, queue) {}
    void stop(void);

    bool record(File file, APW_FORMAT fmt, unsigned int channels, bool paused = false);
    bool record(const char *filename, APW_FORMAT fmt, unsigned int channels, bool paused = false);

    bool writeHeader(apwFile file);            						 	// updates header of a file
    bool writeHeader(const char *filename);         				// updates header of a file on SD
    bool writeHeader(void) {return writeHeader(wavfile);}  	// updates header of current file

    bool isRecording(void) {return isRunning();}
    uint32_t lengthMillis(void);
    bool addMemoryForWrite(size_t mult) {return addMemory(mult);} // add memory
    bool toggleRecordPause(void) {return togglePause();}
    bool pause(const bool pause);
  private:
    virtual void update(void);
    bool start( APW_FORMAT fmt, unsigned int channels, bool paused = false );
    _tEncoderDecoder encoder;
    audio_block_t *queue[_AudioRecordWav_MaxChannels] = {nullptr};
    size_t buffer_wr, buffer_wr_start;
		int data_length;              													// # of recorded frames
    int data_length_old;
};
#endif //enable


#endif // defined(KINETISL)
