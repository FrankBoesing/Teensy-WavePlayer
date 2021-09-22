/*
   Wavefile player
   Copyright (c) 2021, Frank Bösing, f.boesing @ gmx (dot) de

   This library is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this library.  If not, see <http://www.gnu.org/licenses/>.


   Diese Bibliothek ist freie Software: Sie können es unter den Bedingungen
   der GNU General Public License, wie von der Free Software Foundation,
   Version 3 der Lizenz oder (nach Ihrer Wahl) jeder neueren
   veröffentlichten Version, weiterverbreiten und/oder modifizieren.

   Diese Bibliothek wird in der Hoffnung, dass es nützlich sein wird, aber
   OHNE JEDE GEWÄHRLEISTUNG, bereitgestellt; sogar ohne die implizite
   Gewährleistung der MARKTFÄHIGKEIT oder EIGNUNG FÜR EINEN BESTIMMTEN ZWECK.
   Siehe die GNU General Public License für weitere Details.

   Sie sollten eine Kopie der GNU General Public License zusammen mit diesem
   Programm erhalten haben. Wenn nicht, siehe <http://www.gnu.org/licenses/>.

*/

#include "play_wav.h"

#if !defined(KINETISL)

#define MIN_BUFFER_SIZE 1024 //min bytes to read from file


#if defined(LOG_LEVEL)
//https://github.com/FrankBoesing/TeensyLogger
#define LOGTIMESTAMP
#define LOGPRINTLEVEL
#include <TeensyLogger.h>
#else
#define LOGE(...) {}
#define LOGD(...) {}
#define LOGW(...) {}
#define LOGI(...) {}
#define LOGV(...) {}
#endif

#ifdef PACKED
#undef PACKED
#endif
#ifdef INLINE
#undef INLINE
#endif
#ifdef likely
#undef likely
#endif
#ifdef unlikely
#undef unlikely
#endif

#define PACKED __attribute__((packed))
#define INLINE inline __attribute__((always_inline))
#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

#if defined(DEBUG_PIN_PLAYWAV)
#define DBGPIN_HIGH	{digitalWriteFast(DEBUG_PIN_PLAYWAV, HIGH);}
#define DBGPIN_LOW	{digitalWriteFast(DEBUG_PIN_PLAYWAV, LOW);}
#else
#define DBGPIN_HIGH	{}
#define DBGPIN_LOW	{}
#endif
//#define STOP_ALL_ISRS

enum APW_STATE : char {STATE_STOP, STATE_PAUSED, STATE_RUNNING};

static const float audioBlockMs = 1000 * AUDIO_BLOCK_SAMPLES / (float)(AUDIO_SAMPLE_RATE_EXACT); // block time ms (default: 2.9)
static const float msPerSample = 1000.0f / AUDIO_SAMPLE_RATE_EXACT;
static const audio_block_t zeroblock = {0}; // required to deal gracefully with NULL block
static const int numDecoders = (int)APW_NONE;
static const uint8_t bytesPerSample[numDecoders] = {1, 1, 1, 2, 2, 3};


//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------

INLINE static uint32_t __ldrexw(volatile uint32_t *addr)
{
  uint32_t result;
  asm volatile ("ldrex %0, [%1]" : "=r" (result) : "r" (addr) : "memory");
  return (result);
}

INLINE static uint32_t __strexw(uint32_t value, volatile uint32_t *addr)
{
  uint32_t result;
  asm volatile ("strex %0, %2, [%1]" : "=&r" (result) : "r" (addr), "r" (value) : "memory" );
  return (result);
}

INLINE bool stopInt()
{
#if defined(STOP_ALL_ISRS)
  uint32_t primask;
  asm volatile("mrs %0, primask\n" : "=r" (primask)::"memory");
  __disable_irq();
  return (primask == 0) ? true : false;
#else
  if ( likely(NVIC_IS_ENABLED(IRQ_SOFTWARE)) )
  {
    NVIC_DISABLE_IRQ(IRQ_SOFTWARE);
		asm("":::"memory");
    return true;
  }
  return false;
#endif
}

INLINE void startInt(bool enabled)
{
	asm("":::"memory");
#if defined(STOP_ALL_ISRS)
  if (likely(enabled)) __enable_irq();
#else
  if (likely(enabled))
    NVIC_ENABLE_IRQ(IRQ_SOFTWARE);
#endif

}

//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------

void apwFile::reset(void)
{
  close();
  sdFile = FsFile();
  file = File();
  fileType = APW_FILE_NONE;
}

void apwFile::open(const char *filename, int mode)
{
  bool irq = stopInt();
  sdFile = SD.sdfs.open(filename, mode);
  fileType = APW_FILE_SD;
  startInt(irq);
  LOGI("Open \"%s\"", filename);
  if (!sdFile) {
    LOGW("Could not open");
  }
}

void apwFile::close(void)
{
  bool irq = stopInt();
  if (sdFile) sdFile.close();
  if (file) file.close();
  startInt(irq);
}

apwFile::operator bool()
{
  return sdFile || file;
}

bool apwFile::isSD(void)
{
	return APW_FILE_SD == fileType;
}

bool apwFile::seek(size_t pos)
{
  bool b;
  bool irq = stopInt();
  switch (fileType)
  {
    case APW_FILE_SD : b = sdFile.seek(pos); break;
    case APW_FILE_OTHER : b = file.seek(pos); break;
    default : b = false; LOGW("Seek on unknown file type"); break;
  }
  startInt(irq);
  return b;
}

size_t apwFile::position(void)
{
  size_t p;
  bool irq = stopInt();
  switch (fileType)
  {
    case APW_FILE_SD : p = sdFile.position(); break;
    case APW_FILE_OTHER : p = file.position(); break;
    default : p = 0; LOGW("position() on unknown file type"); break;
  }
  startInt(irq);
  return p;
}

size_t apwFile::size(void)
{
  size_t s;
  bool irq = stopInt();
  switch (fileType)
  {
    case APW_FILE_SD : s = sdFile.fileSize(); break;
    case APW_FILE_OTHER : s = file.size(); break;
    default : s = 0; LOGW("size() on unknown file type"); break;
  }
  startInt(irq);
  return s;
}

void apwFile::flush(void)
{
  bool irq = stopInt();
  switch (fileType)
  {
    case APW_FILE_SD : sdFile.sync(); break;
    case APW_FILE_OTHER : file.flush(); break;
    default : break;
  }
  startInt(irq);
}

bool apwFile::preAllocate(uint64_t length)
{
  if (fileType != APW_FILE_SD) return true;
  bool irq = stopInt();
  bool b = sdFile.preAllocate(length);
  startInt(irq);
  LOGI("%d MBytes preallocated, result:%d", length / (1024 * 1024), b);
  return b;
}

bool apwFile::truncate(void)
{
  if (fileType != APW_FILE_SD)  return true;
  bool irq = stopInt();
  bool b = sdFile.truncate();
  startInt(irq);
  LOGI("File truncated, result: %d", b);
  return b;
}

INLINE size_t apwFile::read(void *buf, size_t nbyte)
{
  bool irq = stopInt();
  size_t r = readInISR(buf, nbyte);
  startInt(irq);
  return r;
}

INLINE size_t apwFile::readInISR(void *buf, size_t nbyte)
{
	size_t r;
	DBGPIN_HIGH;
  if (likely(fileType == APW_FILE_SD))
		r = sdFile.read(buf, nbyte);
  else
		r =  file.read(buf, nbyte);
	DBGPIN_LOW;
	return r;
}

INLINE size_t apwFile::write(void *buf, size_t nbyte)
{
  bool irq = stopInt();
  size_t r = writeInISR(buf, nbyte);
  startInt(irq);
  return r;
}

INLINE size_t apwFile::writeInISR(void *buf, size_t nbyte)
{
	size_t r;
	DBGPIN_HIGH;
  if (likely(fileType == APW_FILE_SD))
		r = sdFile.write(buf, nbyte);
	else
		r = file.write(buf, nbyte);
	DBGPIN_LOW;
	return r;
}

//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------

uint8_t AudioBaseWav::_instances = 0;
uint8_t AudioBaseWav::_sz_mem_additional = 1;

FLASHMEM
AudioBaseWav::AudioBaseWav(void)
{
  reset();
  my_instance = _instances;
  ++_instances;

#if defined (DEBUG_PIN_PLAYWAV)
  pinMode(DEBUG_PIN_PLAYWAV, OUTPUT);
#endif

}

FLASHMEM
AudioBaseWav::~AudioBaseWav(void)
{
  state = STATE_STOP;
  wavfile.close();
  freeBuffer();
}

void AudioBaseWav::reset(void)
{
  state = STATE_STOP;
  last_err = ERR_OK;
  dataFmt = APW_NONE;
  buffer = nullptr;
  sample_rate = channels = bytes = 0;
}

inline bool AudioBaseWav::isPaused(void) {
  return (state == STATE_PAUSED);
}
inline bool AudioBaseWav::isStopped(void) {
  return (state == STATE_STOP);
}

bool AudioBaseWav::isRunning(void)
{
  return state == STATE_RUNNING;
}

void AudioBaseWav::pause(const bool pause)
{
  switch (state)
  {
    case STATE_STOP: return;
    case STATE_PAUSED: state = STATE_RUNNING; break;
    case STATE_RUNNING: state = STATE_PAUSED; break;
  }
}

void AudioBaseWav::togglePause(void)
{
  pause(state == STATE_RUNNING);
}


bool AudioBaseWav::addMemory(size_t mult)
{

  if (mult < 1) mult = 1;
  _sz_mem_additional = mult;

  return true;
}

//----------------------------------------------------------------------------------------------------
bool AudioBaseWav::createBuffer() //!< allocate the buffer
{

  LOGV("malloc() 0x%x bytes", sz_mem);

  buffer = (int8_t*)malloc(sz_mem);
  if (!buffer) {
    sz_mem = 0;
    last_err = ERR_OUT_OF_MEMORY;
    LOGE("Out of Memory");
  }
  return buffer != nullptr;
}

void AudioBaseWav::freeBuffer(void)
{
  if (likely(buffer)) {
    free(buffer);
    LOGV("free() 0x%x bytes", sz_mem);
  }
  buffer = nullptr;
  sz_mem = 0;
}


//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
// 8 bit unsigned:
__attribute__((hot)) static
size_t decode_8bit(int8_t buffer[], audio_block_t *queue[], const unsigned int channels)
{
  int8_t *p = buffer;

  size_t i = 0;
  switch (channels) {
    case 1:
      //todo: 32 bit reads
      do {
        queue[0]->data[i    ] = ( p[i    ] - 128 ) << 8; //8 bit fmt is unsigned
        queue[0]->data[i + 1] = ( p[i + 1] - 128 ) << 8;
        i += 2;
      } while (i < AUDIO_BLOCK_SAMPLES);
      return AUDIO_BLOCK_SAMPLES * 1;

    case 2:
      //todo: 32 bit reads
      do {
        queue[0]->data[i] = ( *p++ - 128 ) << 8;
        queue[1]->data[i] = ( *p++ - 128 ) << 8;
      } while (++i < AUDIO_BLOCK_SAMPLES);
      return AUDIO_BLOCK_SAMPLES * 2;

    default:
      do {
        unsigned int chan = 0;
        do {
          queue[chan]->data[i] = ( *p++ - 128 ) << 8;
        } while (++chan < channels);
      } while (++i < AUDIO_BLOCK_SAMPLES);
      return AUDIO_BLOCK_SAMPLES * channels;
  }
}


// 8 bit signed:
__attribute__((hot)) static
size_t decode_8bit_signed(int8_t buffer[], audio_block_t *queue[], const unsigned int channels)
{
  int8_t *p = buffer;

  size_t i = 0;
  do {
    unsigned int chan = 0;
    do {
      queue[chan]->data[i] = (*p++) << 8;
    } while (++chan < channels);
  } while (++i < AUDIO_BLOCK_SAMPLES);
  return AUDIO_BLOCK_SAMPLES * channels;

}

//https://github.com/dpwe/dpwelib
const static short ulaw_decode[256] = {
  -32124, -31100, -30076, -29052, -28028, -27004, -25980, -24956,
  -23932, -22908, -21884, -20860, -19836, -18812, -17788, -16764,
  -15996, -15484, -14972, -14460, -13948, -13436, -12924, -12412,
  -11900, -11388, -10876, -10364,  -9852,  -9340,  -8828,  -8316,
  -7932,  -7676,  -7420,  -7164,  -6908,  -6652,  -6396,  -6140,
  -5884,  -5628,  -5372,  -5116,  -4860,  -4604,  -4348,  -4092,
  -3900,  -3772,  -3644,  -3516,  -3388,  -3260,  -3132,  -3004,
  -2876,  -2748,  -2620,  -2492,  -2364,  -2236,  -2108,  -1980,
  -1884,  -1820,  -1756,  -1692,  -1628,  -1564,  -1500,  -1436,
  -1372,  -1308,  -1244,  -1180,  -1116,  -1052,   -988,   -924,
  -876,   -844,   -812,   -780,   -748,   -716,   -684,   -652,
  -620,   -588,   -556,   -524,   -492,   -460,   -428,   -396,
  -372,   -356,   -340,   -324,   -308,   -292,   -276,   -260,
  -244,   -228,   -212,   -196,   -180,   -164,   -148,   -132,
  -120,   -112,   -104,    -96,    -88,    -80,    -72,    -64,
  -56,    -48,    -40,    -32,    -24,    -16,     -8,      0,
  32124,  31100,  30076,  29052,  28028,  27004,  25980,  24956,
  23932,  22908,  21884,  20860,  19836,  18812,  17788,  16764,
  15996,  15484,  14972,  14460,  13948,  13436,  12924,  12412,
  11900,  11388,  10876,  10364,   9852,   9340,   8828,   8316,
  7932,   7676,   7420,   7164,   6908,   6652,   6396,   6140,
  5884,   5628,   5372,   5116,   4860,   4604,   4348,   4092,
  3900,   3772,   3644,   3516,   3388,   3260,   3132,   3004,
  2876,   2748,   2620,   2492,   2364,   2236,   2108,   1980,
  1884,   1820,   1756,   1692,   1628,   1564,   1500,   1436,
  1372,   1308,   1244,   1180,   1116,   1052,    988,    924,
  876,    844,    812,    780,    748,    716,    684,    652,
  620,    588,    556,    524,    492,    460,    428,    396,
  372,    356,    340,    324,    308,    292,    276,    260,
  244,    228,    212,    196,    180,    164,    148,    132,
  120,    112,    104,     96,     88,     80,     72,     64,
  56,     48,     40,     32,     24,     16,      8,      0
};

// 8 bit ulaw:
__attribute__((hot)) static
size_t decode_8bit_ulaw(int8_t buffer[], audio_block_t *queue[], const unsigned int channels)
{
  uint8_t *p = (uint8_t*)buffer;

  size_t i = 0;
  switch (channels) {
    case 1:
      do {
        queue[0]->data[i    ] = ulaw_decode[p[i    ]];
        queue[0]->data[i + 1] = ulaw_decode[p[i + 1]];
        i += 2;
      } while (i < AUDIO_BLOCK_SAMPLES);
      return AUDIO_BLOCK_SAMPLES * 1;
    case 2:
      do {
        queue[0]->data[i] = ulaw_decode[*p++];
        queue[1]->data[i] = ulaw_decode[*p++];
      } while (++i < AUDIO_BLOCK_SAMPLES);
      return AUDIO_BLOCK_SAMPLES * 2;

    default:
      do {
        unsigned int chan = 0;
        do {
          queue[chan]->data[i] = ulaw_decode[*p++];
        } while (++chan < channels);
      } while (++i < AUDIO_BLOCK_SAMPLES);
      return AUDIO_BLOCK_SAMPLES * channels;
  }
}

// 16 bit:
__attribute__((hot)) static
size_t decode_16bit(int8_t buffer[],  audio_block_t *queue[], const unsigned int channels)
{
  switch (channels)
  {
    case 1:
      memcpy(&queue[0]->data[0], buffer, AUDIO_BLOCK_SAMPLES * 2); //benchmak this
      return AUDIO_BLOCK_SAMPLES * 2 * 1;
    case 2: {
        uint32_t sample;
        size_t i = 0;
        int32_t *p32 = (int32_t*) buffer;
        do {
          sample = *p32++;
          queue[0]->data[i] = (uint16_t)sample;
          queue[1]->data[i] = (uint16_t)(sample >> 16);
        } while (++i < AUDIO_BLOCK_SAMPLES);
        return AUDIO_BLOCK_SAMPLES * 2 * 2;
      }
    default: {
        size_t i = 0;
        int16_t *p = (int16_t*) buffer;
        do {
          unsigned int chan = 0;
          do {
            queue[chan]->data[i] = *p++;
          } while (++chan < channels);
        } while (++i < AUDIO_BLOCK_SAMPLES);
        return AUDIO_BLOCK_SAMPLES * 2 * channels;
      }
  }
}

// 16 bit big endian:

__attribute__((hot)) static
size_t decode_16bit_bigendian(int8_t buffer[], audio_block_t *queue[], const unsigned int channels)
{

  int16_t *p = (int16_t*) buffer;
  size_t i = 0;
  do {
    unsigned int chan = 0;
    do {
      queue[chan]->data[i] = __builtin_bswap16(*p++);
    } while (++chan < channels);
  } while (++i < AUDIO_BLOCK_SAMPLES);
  return AUDIO_BLOCK_SAMPLES * 2 * channels;
}


// 24 bit:
__attribute__((hot)) static
size_t decode_24bit(int8_t buffer[], audio_block_t *queue[], const unsigned int channels)
{
  //TODO: Optimize
	size_t i = 0;
	uint8_t *p = (uint8_t*) buffer;
	p++;
	do {
		unsigned int chan = 0;
		uint16_t sample;
		do {
			sample = *(uint16_t*)p; //note, this involves unaligned reads.
			p+=3;
			queue[chan]->data[i] = sample;
		} while (++chan < channels);
	} while (++i < AUDIO_BLOCK_SAMPLES);
	return AUDIO_BLOCK_SAMPLES * 3 * channels;
}

static const _tEncoderDecoder decoders[numDecoders] = {
  decode_8bit, decode_8bit_signed, decode_8bit_ulaw,
  decode_16bit, decode_16bit_bigendian, decode_24bit
};

// Todo:
//- upsampling (CMSIS?)
//- downsampling (CMSIS?) (is that really needed? pretty inefficient to load way more data than necessary and downsample then..
//- ...but may be useful for sample rate conversion or playback speed variation
//- play float formats?


//----------------------------------------------------------------------------------------------------

bool AudioPlayWav::play(File file, const bool paused)
{
  stop();
  wavfile.reset();
  wavfile.use(file);
  return _play(APW_NONE, 0, 0, paused);
}

bool AudioPlayWav::play(const char *filename, const bool paused)
{
  stop();
  wavfile.reset();
  wavfile.open(filename);
  return _play(APW_NONE, 0, 0, paused);
}

bool AudioPlayWav::playRaw(File file, APW_FORMAT fmt, uint32_t sampleRate, uint8_t number_of_channels, bool paused)
{
  stop();
  wavfile.reset();
  wavfile.use(file);
  return _play(fmt, sampleRate, number_of_channels, paused);
}

bool AudioPlayWav::playRaw(const char *filename, APW_FORMAT fmt, uint32_t sampleRate, uint8_t number_of_channels, bool paused)
{
  stop();
  wavfile.reset();
  File file = SD.open(filename);
  return _play(fmt, sampleRate, number_of_channels, paused);
}

bool AudioPlayWav::_play(APW_FORMAT fmt, uint32_t sampleRate, uint8_t number_of_channels, bool paused )
{
  if (!readHeader(fmt, sampleRate, number_of_channels, paused ? STATE_PAUSED : STATE_RUNNING ))
  {
    stop();
    return false;
  }
  return true;
}

void AudioPlayWav::stop(void)
{
  APW_STATE oldstate;
  oldstate = state;
  state = STATE_STOP;
  wavfile.close();
  freeBuffer();
  if (oldstate != STATE_STOP) LOGD("Play: stop");
}

//WAV:
/*
  00000000  52494646 66EA6903 57415645 666D7420  RIFFf.i.WAVEfmt
  00000010  10000000 01000200 44AC0000 10B10200  ........D.......
  00000020  04001000 4C495354 3A000000 494E464F  ....LIST:...INFO
  00000030  494E414D 14000000 49205761 6E742054  INAM....I Want T
  00000040  6F20436F 6D65204F 76657200 49415254  o Come Over.IART
  00000050  12000000 4D656C69 73736120 45746865  ....Melissa Ethe
  00000060  72696467 65006461 746100EA 69030100  ridge. a..i...
  00000070  FEFF0300 FCFF0400 FDFF0200 0000FEFF  ................
  00000080  0300FDFF 0200FFFF 00000100 FEFF0300  ................
  00000090  FDFF0300 FDFF0200 FFFF0100 0000FFFF  ................
*/

static const uint32_t cRIFF = 0x46464952; //'RIFF'
static const uint32_t cWAVE = 0x45564157; //'WAVE'
static const uint32_t cFMT  = 0x20746D66; //'fmt '
static const uint32_t cFACT = 0x74636166; //'fact'
static const uint32_t cDATA = 0x61746164; //'data'
static const char cGUID[14] = {'\x00', '\x00', '\x00', '\x00', '\x10', '\x00',
                               '\x80', '\x00', '\x00', '\xAA', '\x00', '\x38', '\x9B', '\x71'
                              };


typedef struct {
  unsigned long id;
  unsigned long len;
  unsigned long riffType;
} PACKED tFileHeader;

// https://docs.microsoft.com/de-de/windows/win32/api/mmreg/ns-mmreg-waveformat
typedef struct
{
  //unsigned long  chunkID;
  //unsigned long  chunkSize;
  unsigned short  wFormatTag;
  unsigned short  nChannels;
  unsigned long   nSamplesPerSec;
  unsigned long   nAvgBytesPerSec;
  unsigned short  nBlockAlign;
} PACKED tFmtHeader;

// https://docs.microsoft.com/en-us/previous-versions/dd757713(v=vs.85)
typedef struct
{
  //unsigned long  chunkID;
  //unsigned long  chunkSize;
  unsigned short wFormatTag;
  unsigned short wChannels;
  unsigned long  dwSamplesPerSec;
  unsigned long  dwAvgBytesPerSec;
  unsigned short wBlockAlign;
  unsigned short wBitsPerSample;
  //unsigned short cbSize;
} PACKED tFmtHeaderEx;

// https://docs.microsoft.com/de-de/windows/win32/api/mmreg/ns-mmreg-waveformatextensible
typedef struct
{
  //unsigned long  chunkID;
  //unsigned long  chunkSize;
  //tFmtHeaderex fmtHeader;
  union {
    unsigned short wValidBitsPerSample;
    unsigned short wSamplesPerBlock;
    unsigned short wReserved;
  } PACKED;
  unsigned long dwChannelMask;
  struct {
    unsigned short  format;
    char guid[14];
  } PACKED;
} PACKED tFmtHeaderExtensible;

typedef struct
{
  unsigned long chunkID;
  unsigned long chunkSize;
} PACKED tDataHeader;


//AIFF, AIFC:
// http://www-mmsp.ece.mcgill.ca/Documents/AudioFormats/AIFF/Docs/AIFF-1.3.pdf
// http://www-mmsp.ece.mcgill.ca/Documents/AudioFormats/AIFF/Docs/AIFF-C.9.26.91.pdf

/*
  00000000  46 4F 52 4D 00 01 6F 94 41 49 46 46 43 4F 4D 4D  FORM..o”AIFFCOMM
  00000010  00 00 00 12 00 02 00 00 5B C5 00 10 40 0B FA 00  ........[Å..@.ú.
  00000020  00 00 00 00 00 00 41 4E 4E 4F 00 00 00 49 41 46  ......ANNO...IAF
  00000030  73 70 64 61 74 65 3A 20 32 30 30 33 2D 30 31 2D  spdate: 2003-01-
  00000040  33 30 20 30 33 3A 32 38 3A 33 36 20 55 54 43 00  30 03:28:36 UTC.
  00000050  75 73 65 72 3A 20 6B 61 62 61 6C 40 43 41 50 45  user: kabal@CAPE
  00000060  4C 4C 41 00 70 72 6F 67 72 61 6D 3A 20 43 6F 70  LLA.program: Cop
  00000070  79 41 75 64 69 6F 00 00 53 53 4E 44 00 01 6F 1C  yAudio..SSND..o.
*/

static const uint32_t cFORM = 0x4D524F46; //'FORM'
static const uint32_t cAIFF = 0x46464941; //'AIFF'
static const uint32_t cAIFC = 0x43464941; //'AIFC'
static const uint32_t cCOMM = 0x4D4D4F43; //'COMM'
static const uint32_t cSSND = 0x444e5353; //'SSND'
//static const uint32_t cULAW = 0x57414C55; //'ULAW' // not supported
static const uint32_t culaw = 0x77616c75; //'ulaw'
static const uint32_t craw =  0x20776172; //'raw '

typedef struct {
  short numChannels;
  unsigned long numSampleFrames;
  short sampleSize;
  uint8_t sampleRate[10]; //80  bit  IEEE  Standard  754  floating  point... ignore that!
} PACKED taiffCommonChunk;

typedef struct {
  short numChannels;
  unsigned long numSampleFrames;
  short sampleSize;
  uint8_t sampleRate[10]; //80  bit  IEEE  Standard  754  floating  point... ignore that!
  unsigned long compressionType;
} PACKED taifcCommonChunk;

bool AudioPlayWav::readHeader(APW_FORMAT fmt, uint32_t sampleRate, uint8_t number_of_channels, APW_STATE newState)
{
  size_t sz_frame, rd;
  tFileHeader fileHeader;
  tDataHeader dataHeader;

  reset();
  buffer_rd = total_length = channelmask = 0;

  sample_rate = sampleRate;
  channels = number_of_channels;
  dataFmt = fmt;

  if (!wavfile) {
    last_err = ERR_FILE;
    LOGI("File not open.");
    return false;
  }

  last_err = ERR_FORMAT;
  wavfile.seek(0);

  if ( dataFmt == APW_NONE) {
    wavfile.read(&fileHeader, sizeof(fileHeader));
  }

  if ( dataFmt != APW_NONE) {
    // ---------- RAW ----------------------------
    LOGV("Format: RAW");
    total_length = wavfile.size();
    if (total_length == 0) return false;
  }

  else if (fileHeader.id == cFORM &&
           (fileHeader.riffType == cAIFF || fileHeader.riffType == cAIFC))
  {
    // ---------- AIFF ----------------------------
    //unlike wav, the samples chunk (here "SSND") can be everywhere in the file!
    //unfortunately, it is big endian :-(
    size_t position = sizeof(fileHeader);

    bool isAIFC = fileHeader.riffType == cAIFC;
    LOGV("Format: %s", isAIFC ? "AIFC" : "AIFF");
    bool COMMread = false;
    bool SSNDread = false;
    uint8_t bytes;

    do {
      if (!wavfile.seek(position)) return false;
      rd = wavfile.read(&dataHeader, sizeof(dataHeader));
      dataHeader.chunkSize = __builtin_bswap32(dataHeader.chunkSize);
      if (rd < sizeof(dataHeader)) return false;
      //Serial.printf("Chunk:%c%c%c%c", (char)dataHeader.chunkID & 0xff, (char)(dataHeader.chunkID >> 8 & 0xff), (char)(dataHeader.chunkID  >> 16 & 0xff), (char)(dataHeader.chunkID >> 24 &0xff));
      //Serial.printf(" 0x%x size: %d\n",dataHeader.chunkID, dataHeader.chunkSize);
      if (!COMMread && dataHeader.chunkID == cCOMM) {
        //Serial.print(":COMM ");
        taifcCommonChunk commonChunk;
        rd = wavfile.read(&commonChunk, sizeof(commonChunk));
        if (rd < sizeof(commonChunk)) return false;

        channels = __builtin_bswap16(commonChunk.numChannels);
        commonChunk.sampleSize = __builtin_bswap16(commonChunk.sampleSize);
        bytes = commonChunk.sampleSize / 8;
        total_length = __builtin_bswap32(commonChunk.numSampleFrames) * channels * bytes;

        //Serial.printf("Channels:%d Length:%d Bytes:%d\n", (int)channels, (int)total_length, (int)bytes);
        if (total_length == 0) return false;
        if (commonChunk.sampleSize != 8 && commonChunk.sampleSize != 16) return false;

        //if (isAIFC) Serial.printf("Compression:%c%c%c%c 0x%x\n", (char)commonChunk.compressionType & 0xff, (char)(commonChunk.compressionType >> 8 & 0xff), (char)(commonChunk.compressionType  >> 16 & 0xff), (char)(commonChunk.compressionType >> 24 &0xff), commonChunk.compressionType);

        if (bytes == 2) {
          if (isAIFC) return false;
          dataFmt = APW_16BIT_SIGNED_BIGENDIAN;
        } else if (bytes == 1) {
          if (isAIFC) {
            switch (commonChunk.compressionType)
            {
              case culaw: dataFmt = APW_ULAW;
                break;
              case craw:  dataFmt = APW_8BIT_UNSIGNED;
                break;
              default:    return false;
            }
          } else
            dataFmt = APW_8BIT_SIGNED;
        } else return false;

        COMMread = true;
        if (SSNDread) break;

      } else if (dataHeader.chunkID == cSSND) {
        //todo: offset etc...

        //Serial.println(":SSND");
        SSNDread = true;
        if (COMMread) break;
      } ;

      position += sizeof(dataHeader) + dataHeader.chunkSize ;
      if (position & 1) position++; //make position even
    } while (true);

    if (!SSNDread || !COMMread) return false;

  }  // AIFF
  else if ( likely(fileHeader.id == cRIFF &&
                   fileHeader.riffType == cWAVE) )
  {
    // ---------- WAV ----------------------------
    LOGV("Format: WAV");
    size_t position = sizeof(fileHeader);
    bool fmtok = false;

    do {

      wavfile.seek(position);
      rd = wavfile.read(&dataHeader, sizeof(dataHeader));

      if (rd < sizeof(dataHeader)) return false;

      if (dataHeader.chunkID == cFMT) {
				uint8_t bytes;
        tFmtHeaderEx fmtHeader;
        memset((void*)&fmtHeader, 0, sizeof(tFmtHeaderEx));
        //Serial.println(dataHeader.chunkSize);
        if (dataHeader.chunkSize < 16) {
          wavfile.read(&fmtHeader, sizeof(tFmtHeader));
          bytes = 1;
        } else if (dataHeader.chunkSize == 16) {
          wavfile.read(&fmtHeader, sizeof(tFmtHeaderEx));
          bytes = fmtHeader.wBitsPerSample / 8;
        } else {
          tFmtHeaderExtensible fmtHeaderExtensible;
          wavfile.read(&fmtHeader, sizeof(tFmtHeaderEx));
          bytes = fmtHeader.wBitsPerSample / 8;
          memset((void*)&fmtHeaderExtensible, 0, sizeof(fmtHeaderExtensible));
          wavfile.read(&fmtHeaderExtensible, sizeof(fmtHeaderExtensible));
          channelmask = fmtHeaderExtensible.dwChannelMask;
          //Serial.printf("channel mask: 0x%x\n", channelmask);
        }

        LOGV("Format:%d Bits:%d\n", fmtHeader.wFormatTag, fmtHeader.wBitsPerSample);
        if (fmtHeader.dwSamplesPerSec == 0) return false;
        sample_rate = fmtHeader.dwSamplesPerSec;
        channels = fmtHeader.wChannels;
				switch (bytes) {
					case 1: dataFmt = APW_8BIT_UNSIGNED; break;
					case 2: dataFmt = APW_16BIT_SIGNED; break;
					case 3: dataFmt = APW_24BIT_SIGNED; break;
				  default: return false;
				}
        if (fmtHeader.wFormatTag != 1 &&
            fmtHeader.wFormatTag != 7 && //ulaw
            fmtHeader.wFormatTag != 65534) return false;
        if (fmtHeader.wFormatTag == 7) {
          if (bytes != 1) return false;
          dataFmt = APW_ULAW; //ulaw
          //Serial.println("ULAW!");
        }
        fmtok = true;
      }
      else if (dataHeader.chunkID == cDATA) {
        total_length = dataHeader.chunkSize;
        break;
      }

      position += sizeof(dataHeader) + dataHeader.chunkSize;
    } while (true);

    if (fmtok != true) return false;

  }  //wav
  else
    return false; //unknown format

  if (channels == 0 || channels > _AudioPlayWav_MaxChannels) return false;
  if (dataFmt == APW_NONE) return false;
  if (sample_rate == 0) sample_rate = AUDIO_SAMPLE_RATE_EXACT;

  last_err = ERR_OK;
  bytes = bytesPerSample[dataFmt];
  decoder = decoders[dataFmt];
  padding = (dataFmt != APW_8BIT_UNSIGNED) ? 0 : 0x80;
  LOGV("SampleRate: %d, Channels: %d, Bytes: %d, Length: %dms",
       sample_rate, channels, bytes,
       round( msPerSample * total_length / (bytes * channels) )
      );

  sz_frame = AUDIO_BLOCK_SAMPLES * channels * bytes;
  data_length = total_length / sz_frame;

  //calculate the needed buffer memory:
  sz_mem = _instances * sz_frame;
  sz_mem *= _sz_mem_additional;
	if (wavfile.isSD())
		while (sz_mem < MIN_BUFFER_SIZE) sz_mem *=2;
  if (!createBuffer()) return false;

  // pre-load according to instance number

  bool irq = stopInt();

  size_t x = (my_instance + ((_instances - 1) - updateStep)) % _instances;

  LOGV("Inst:%d ustep:%d = %d\n", my_instance, updateStep, x);
  buffer_rd = sz_mem - x * AUDIO_BLOCK_SAMPLES * channels * bytes;

  if (buffer_rd < sz_mem) {
    size_t len = sz_mem - buffer_rd;
    rd = wavfile.read(&buffer[buffer_rd], len);
    if (rd < len)
      memset(&buffer[buffer_rd + rd], padding, sz_mem - len);
  }

  state = newState;
  startInt(irq);

  return true;
}


__attribute__((hot))
void  AudioPlayWav::update(void)
{

  if (++updateStep >= _instances) updateStep = 0;
  if (state != STATE_RUNNING) return;

  unsigned int chan;

  // allocate the audio blocks to transmit
  chan = 0;
  do {
    queue[chan] = AudioStream::allocate();
    if ( unlikely(queue[chan] == nullptr) )
    {
      for (unsigned int i = 0; i != chan; ++i)
        AudioStream::release(queue[i]);
      last_err = ERR_NO_AUDIOBLOCKS;
      LOGE("Waveplayer stopped: Not enough AudioMemory().");
      stop();
      return;
    }
  } while (++chan < channels);


  if (buffer_rd >= sz_mem) buffer_rd = 0;
  if (updateStep == my_instance &&
      buffer_rd == 0) //! rmv to dbg interleave
  {

    /*
      if (buffer_rd != 0) {
      LOGV("Inst: %d, ustep:%d Buf: 0x%x SZ: 0x%x, (Frame:0x%x - %d)\n", my_instance,updateStep, buffer_rd, sz_mem, channels * bytes * 128, buffer_rd / (channels * bytes * 128));
      } */ // Debug interleave
    // buffer_rd is alway 0 here.. the compiler knows about that and will optimize it away.
    // just leave it in for easier debug.
    buffer_rd = 0;
    size_t len = sz_mem - buffer_rd;
    size_t rd = wavfile.readInISR(&buffer[buffer_rd], len);
    if (unlikely(rd < len)) {
      LOGD("EOF");
      if (rd == 0) {
        stop();
        return;
      } else
      {
        len -= rd;
        memset(&buffer[rd + buffer_rd], padding, len);
        data_length = 1;
      }
    }
  }

  // copy the samples to the audio blocks:
  buffer_rd += decoder(&buffer[buffer_rd], queue, channels);

  // transmit them:
  chan = 0;
  do
  {
    AudioStream::transmit(queue[chan], chan);
    AudioStream::release(queue[chan]);
    //queue[chan] = nullptr;
  } while (++chan < channels);

  --data_length;
  if (unlikely(data_length <= 0)) {
    LOGI("No Data anymore.");
    stop(); // proper tidy up when playing is done
  }

}


uint32_t AudioPlayWav::positionMillis(void)
{
  //use an interrupt detector to make sure all vars are consistent.
  //strex will fail if an interrupt occured. An other way would be to block audio interrupts.

  uint32_t safe_read;
  size_t total_length;
  int data_length;
  uint8_t bytes, channels;

  do
  {
    __ldrexw(&safe_read);
    total_length = this->total_length;
    bytes = this->bytes;
    channels = this->channels;
    data_length = this->data_length;
  } while ( __strexw(1, &safe_read) );

  if (data_length < 0) data_length = 0;

  return round( msPerSample * ((total_length / (bytes * channels)) - data_length * AUDIO_BLOCK_SAMPLES ));
}

uint32_t AudioPlayWav::lengthMillis(void)
{
  return round( msPerSample * total_length / (bytes * channels) );
}

//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------


typedef struct
{
  tFileHeader fileHeader;
  struct {
    tDataHeader dataHeader;
    tFmtHeaderEx fmtHeader;
  } PACKED file;
} PACKED tWaveFileHeader;

typedef struct {
  tDataHeader dataHeader;
  uint32_t dwSampleLength;
} PACKED tfactChunk;

//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
#pragma GCC push_options
#pragma GCC optimize ("unroll-loops")

// 8 bit unsigned:
__attribute__((hot)) static
size_t encode_8bit(int8_t buffer[], audio_block_t *queue[], const unsigned int channels)
{
  int8_t *p = buffer;
  size_t i = 0;

  switch (channels)
  {
    case 1:
      //todo: 32 bit
      do {
        *p++ = (queue[0]->data[i] >> 8) + 128; //8 bit fmt is unsigned
      } while (++i < AUDIO_BLOCK_SAMPLES);
      return AUDIO_BLOCK_SAMPLES * 1;

    case 2:
      //todo: 32 bit
      do {
        *p++ = (queue[0]->data[i] >> 8) + 128;
        *p++ = (queue[1]->data[i] >> 8) + 128;
      } while (++i < AUDIO_BLOCK_SAMPLES);
      return AUDIO_BLOCK_SAMPLES * 2;

    default:
      do {
        unsigned int chan = 0;
        do {
          *p++ = (queue[chan]->data[i] >> 8) + 128;
        } while (++chan < channels);
      } while (++i < AUDIO_BLOCK_SAMPLES);
      return AUDIO_BLOCK_SAMPLES * channels;
  }
}
// 16 bit:
__attribute__((hot)) static
size_t encode_16bit(int8_t buffer[], audio_block_t *queue[], const unsigned int channels)
{
  size_t i = 0;
  switch (channels)
  {
    case 1:
      memcpy(buffer, &queue[0]->data[0], AUDIO_BLOCK_SAMPLES * 2);
      return AUDIO_BLOCK_SAMPLES * 2 * 1;
    case 2:
      {
        uint32_t *p = (uint32_t*) buffer;
        uint32_t sample;
        do {
          sample = (int32_t)queue[0]->data[i] << 16;
          sample |= (uint16_t)queue[1]->data[i];
          *p++ = sample;
        } while (++i < AUDIO_BLOCK_SAMPLES);
        return AUDIO_BLOCK_SAMPLES * 2 * 2;
      }
    default:
      {
        int16_t *p = (int16_t*) buffer;
        do {
          unsigned int chan = 0;
          do {
            *p++ = queue[chan]->data[i];
          } while (++chan < channels);
        } while (++i < AUDIO_BLOCK_SAMPLES);
        return AUDIO_BLOCK_SAMPLES * 2 * channels;
      }
  }

}

static const _tEncoderDecoder encoders[numDecoders] = {
  encode_8bit, encode_8bit, encode_8bit,
  encode_16bit, encode_16bit, encode_16bit
};

#pragma GCC pop_options
//----------------------------------------------------------------------------------------------------
#if 1

void AudioRecordWav::stop(bool closeFile)
{
  APW_STATE oldstate;
  oldstate = state;
  state = STATE_STOP;
  if (oldstate == STATE_RUNNING)
  {
    LOGV("Stopped, data-left:0x%x", buffer_wr);
    wavfile.write(buffer, buffer_wr);
  }
  freeBuffer();
  if (oldstate != STATE_STOP)
    LOGD("Recording stop. Length %dms, Filesize %0.2fkB",
         lengthMillis(), wavfile.size() / 1024.0f);
  if (likely(wavfile)) {
    writeHeader(wavfile);
    if (likely(closeFile)) {
      wavfile.truncate();
      wavfile.close();
    }
  }

}

void AudioRecordWav::pause(const bool pause)
{
  if (pause && state != STATE_PAUSED) {
    if (wavfile) writeHeader(wavfile);
  }

  AudioBaseWav::pause(pause);
}

uint32_t AudioRecordWav::lengthMillis(void)
{
  return data_length * audioBlockMs;
}

bool AudioRecordWav::record(File file, APW_FORMAT fmt, unsigned int numchannels, bool paused)
{
  state = STATE_STOP;

  if (!file) {
    last_err = ERR_FILE;
    return false;
  }
  wavfile.reset();
  wavfile.use(file);
  return start( fmt, numchannels, paused );
}

bool AudioRecordWav::record(const char *filename, APW_FORMAT fmt, unsigned int numchannels, bool paused)
{
  state = STATE_STOP;
  wavfile.reset();
  wavfile.open(filename, O_RDWR | O_CREAT | O_AT_END);
  uint64_t preAlloc = (uint64_t)AUDIO_SAMPLE_RATE_EXACT * (APW_PREALLOCATE * numchannels * bytesPerSample[fmt]);
  wavfile.preAllocate( preAlloc );
  return start( fmt, numchannels, paused );
}

bool AudioRecordWav::writeHeader(apwFile file)
{

  if (state == STATE_RUNNING) {
    LOGE("WriteHeader: recording in progress");
    return false;
  }
  if (data_length == data_length_old) {
    LOGD("WriteHeader: Datalength unchanged");
    return false;
  }

  size_t pos, sz, wr, wrpos;
  char data[512];

  last_err = ERR_FILE;

  pos = wavfile.position();

  if (pos > 0)
    if (!wavfile.seek(0)) return false;

  data_length_old = data_length;

  const bool extended = false; //may be used later.
  tWaveFileHeader header;
  tFmtHeaderExtensible headerextensible;
  tDataHeader dataHeader;

  wrpos = 0;

  size_t szh = sizeof header;
  if (extended) szh += sizeof headerextensible;

  sz = wavfile.size();
  if (sz == 0) sz = szh;

  header.fileHeader.id = cRIFF;
  header.fileHeader.len = sz - 8;
  header.fileHeader.riffType = cWAVE;
  header.file.dataHeader.chunkID = cFMT;
  if (!extended)
  {
    header.file.dataHeader.chunkSize = sizeof header.file.fmtHeader;
    header.file.fmtHeader.wFormatTag = 1;
  }
  else
  {
    header.file.dataHeader.chunkSize = sizeof header.file.fmtHeader + sizeof headerextensible;
    header.file.fmtHeader.wFormatTag = 65534;
  }
  header.file.fmtHeader.wChannels = channels;
  header.file.fmtHeader.dwSamplesPerSec = sample_rate;
  header.file.fmtHeader.dwAvgBytesPerSec = sample_rate * bytes * channels;
  header.file.fmtHeader.wBlockAlign = bytes * channels;
  header.file.fmtHeader.wBitsPerSample = bytes * 8;
  //header.file.fmtHeader.cbSize = 0;

  memcpy(&data[wrpos], &header, sizeof header);
  wrpos += sizeof header;

  if (extended)
  {
    headerextensible.wValidBitsPerSample = bytes * 8;
    headerextensible.dwChannelMask = (1 << channels) - 1;
    headerextensible.format = 0x01;
    memcpy(headerextensible.guid, cGUID, sizeof headerextensible.guid);
    memcpy(&data[wrpos], &headerextensible, sizeof headerextensible);
    wrpos += sizeof headerextensible;
  }

  //fill in a dummy chunk to make sure the samples start @ position 512
  size_t toFill = 512 - szh - 2 * sizeof dataHeader;
  dataHeader.chunkID = 0x796d7564; //'dumy' chunk
  dataHeader.chunkSize = toFill;

  memcpy(&data[wrpos], &dataHeader, sizeof dataHeader);
  wrpos += sizeof dataHeader;
  memset(&data[wrpos], 0xff, toFill);
  wrpos += toFill;

  szh += sizeof dataHeader + toFill;

  dataHeader.chunkID = cDATA;
  dataHeader.chunkSize = sz - szh - sizeof dataHeader;
  memcpy(&data[wrpos], &dataHeader, sizeof dataHeader);
  wrpos += sizeof dataHeader;

  wr =  wavfile.write(&data, sizeof data);
  wavfile.flush();
  if (pos > 0) wavfile.seek(pos);
  if (wr < sizeof dataHeader) return false;

  last_err = ERR_OK;

  return true;
}

bool AudioRecordWav::start( APW_FORMAT fmt, unsigned int numchannels, bool paused )
{
  bool ok;
  size_t wr;

  reset();
  buffer_wr = data_length_old = 0;
  //memset(queue, 0, sizeof queue);

  if (!wavfile) {
    LOGW("Record: File not open");
    return false;
  }

  last_err = ERR_FORMAT;
  if (numchannels == 0 || numchannels > _AudioRecordWav_MaxChannels) return false;
  if (fmt != APW_8BIT_UNSIGNED && fmt != APW_16BIT_SIGNED) return false;

  last_err = ERR_OK;
  //LOGV("Record:");...

  sz_mem = 0;
  data_length = data_length_old = 0;

  dataFmt = fmt;
  bytes = bytesPerSample[dataFmt];
  encoder = encoders[dataFmt];
  channels = numchannels;

#if defined(KINETISK)
  sample_rate = ((int)(AUDIO_SAMPLE_RATE_EXACT) / 20) * 20; //round (for Teensy 3.x)
#else
  sample_rate = AUDIO_SAMPLE_RATE_EXACT;
#endif

  //write a dummy fileheader and check if it was successful:
  char tmp[512];
  memset(&tmp, 0x00, sizeof tmp);

  ok = wavfile.seek(0);
  wr = wavfile.write(&tmp, sizeof tmp );
  wavfile.flush();

  if (!ok || wr < sizeof tmp) {
    LOGD("Could not write dummy header");
    last_err = ERR_FILE;
    return false;
  }

  size_t sz_frame = AUDIO_BLOCK_SAMPLES * channels * bytes;

  //calculate the needed buffer memory:
  sz_mem = _instances * sz_frame;
  sz_mem *= _sz_mem_additional;
	if (wavfile.isSD())
		while (sz_mem < MIN_BUFFER_SIZE) sz_mem *=2;
  if (!createBuffer()) return false;

  bool irq = stopInt();

  size_t x = (my_instance + ((_instances - 1) - updateStep)) % _instances;
  buffer_wr = sz_mem - x * AUDIO_BLOCK_SAMPLES * channels * bytes;
  if (buffer_wr >= sz_mem) buffer_wr = 0;
  buffer_wr_start = buffer_wr;

  state = paused ? STATE_PAUSED : STATE_RUNNING;
  startInt(irq);

	LOGV("Inst:%d ustep:%d = %d LEN: 0x%x\n", my_instance, updateStep, x, sz_mem - buffer_wr);
  return true;
}


__attribute__((hot))
void  AudioRecordWav::update(void)
{
  if (++updateStep >= _instances) updateStep = 0;
  if (state != STATE_RUNNING) return;

  unsigned int chan;
  chan = 0;
	audio_block_t *qcopy[channels];
  do
  {
    queue[chan] = AudioStream::receiveReadOnly(chan);
    if (!queue[chan]) qcopy[chan] = (audio_block_t*)&zeroblock; // ... data of all zeros
		else qcopy[chan] = queue[chan];
  } while (++chan < channels);


  if (buffer_wr >= sz_mem) buffer_wr = 0;
  if (updateStep == my_instance && buffer_wr == 0)
  {
    wavfile.writeInISR(&buffer[buffer_wr_start], sz_mem - buffer_wr_start);
    buffer_wr_start = 0;
  }

  buffer_wr += encoder(buffer, qcopy, channels);

  // release queues
  chan = 0;
  do
  {
    if (likely(queue[chan] != nullptr))
    {
      AudioStream::release(queue[chan]);
      queue[chan] = nullptr; // < why is this needed?
    }
  } while (++chan < channels);

	++data_length;
}
#endif //enable

#endif
