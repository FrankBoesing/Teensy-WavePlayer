/*
   Wavefile player
   Copyright (c) 2021, Frank Bösing, f.boesing @ gmx (dot) de

   LICENSE: MIT
*/

#pragma once
#include <Arduino.h>

#if !defined(KINETISL)

#include <AudioStream.h>
#include <SD.h>

const int _AudioPlayWav_MaxChannels = 8;
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
    void use(File f) {file = f;fileType = APW_FILE_OTHER;}
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

    static uint8_t _instances;
    static uint8_t _sz_mem_additional;

  private:
    friend class AudioPlayWav;
    friend class AudioRecordWav;

    AudioBaseWav(void);
    ~AudioBaseWav(void);
    void reset(void);
    bool createBuffer(void);
    void freeBuffer(void);
    void close(bool closeFile = true);

    bool isRunning(void);
    bool addMemory(size_t mult);

    apwFile wavfile;
    size_t sz_mem;              	// size of buffer
    int8_t* buffer;
    unsigned int sample_rate;
    unsigned int channels;        // #of channels in the wave file
    int data_length;              // number of frames remaining in file /# of recorded frames
    APW_FORMAT dataFmt;
    uint8_t my_instance;          // instance id
    uint8_t bytes;            		// 1 or 2 bytes?
    APW_STATE state;            	// play status (stop, pause, running)
    APW_ERR last_err;
    uint8_t padding;              // value to pad buffer at EOF
    int8_t updateStep = -1;

};

/*********************************************************************************************************/

typedef size_t (*_tEncoderDecoder)(int8_t [], audio_block_t *[], unsigned int);

class AudioPlayWav : public AudioBaseWav, public AudioStream
{
  public:
    AudioPlayWav(void) : AudioStream(0, NULL) {}
    void stop(void);

    bool play(File file, bool paused = false);
    bool play(const char *filename, bool paused = false); // play from SD
    bool playRaw(File file, APW_FORMAT fmt, uint32_t sampleRate, uint8_t number_of_channels, bool paused = false);
    bool playRaw(const char *filename, APW_FORMAT fmt, uint32_t sampleRate, uint8_t number_of_channels, bool paused = false); // from SD

    bool addMemoryForRead(size_t mult) {return addMemory(mult);} // add memory
    bool togglePlayPause(void) {return togglePause();}
    bool isPlaying(void) {return isRunning();}
		bool pause(const bool pause);
    uint32_t positionMillis(void);
		size_t position();
		bool setPosition(size_t sample);
    uint32_t lengthMillis(void);
    uint32_t channelMask(void) {return channelmask;}
  protected:
    bool _play(APW_FORMAT fmt, uint32_t sampleRate, uint8_t number_of_channels, bool paused );
    bool readHeader(APW_FORMAT fmt, uint32_t sampleRate, uint8_t number_of_channels, APW_STATE newState );
		size_t dataReader(int8_t *buffer, size_t len);
    audio_block_t *queue[_AudioPlayWav_MaxChannels];
    _tEncoderDecoder decoder;
    size_t total_length;      			// number of audio data bytes in file
		size_t firstSampleOffset;		    // file position of first sample
		size_t lastSample;							// Number of last sample
    uint32_t channelmask;           // dwChannelMask
  //private:
		void start(void);
    virtual void update(void);
		size_t buffer_rd;
};
/*********************************************************************************************************/
#if 1
class AudioRecordWav : public AudioBaseWav, public AudioStream
{
  public:
    AudioRecordWav(void): AudioStream(_AudioRecordWav_MaxChannels, queue) {}
    void stop(bool closeFile = true);

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
  protected:
    virtual void update(void);
    bool start( APW_FORMAT fmt, unsigned int channels, bool paused = false );
    _tEncoderDecoder encoder;
    audio_block_t *queue[_AudioRecordWav_MaxChannels] = {nullptr};
    size_t buffer_wr, buffer_wr_start;
    int data_length_old;
};
#endif //enable


#endif // defined(KINETISL)
