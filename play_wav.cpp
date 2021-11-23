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

#include "play_wav.h"

#if !defined(KINETISL)

#define MIN_BUFFER_SIZE 1024 //min bytes to read from file


#if defined(LOG_LEVEL)
//https://github.com/FrankBoesing/TeensyLogger
#define LOGTIMESTAMP
#define LOGPRINTLEVEL
#include <TeensyLogger.h>
#else
#define LOGE(...) do {} while(0)
#define LOGD(...) do {} while(0)
#define LOGW(...) do {} while(0)
#define LOGI(...) do {} while(0)
#define LOGV(...) do {} while(0)
#endif

#ifdef HOT
#undef HOT
#endif
#ifdef COLD
#undef COLD
#endif
#ifdef PACKED
#undef PACKED
#endif
#ifdef INLINE
#undef INLINE
#endif

#define HOT __attribute__((hot))
#define COLD __attribute__((cold, noinline))
#define PACKED __attribute__((packed))
#define INLINE inline __attribute__((always_inline))

#if defined(DEBUG_PIN_PLAYWAV)
#define DBGPIN_HIGH	do {digitalWriteFast(DEBUG_PIN_PLAYWAV, HIGH);} while(0)
#define DBGPIN_LOW	do {digitalWriteFast(DEBUG_PIN_PLAYWAV, LOW);} while(0)
#else
#define DBGPIN_HIGH	do {} while(0)
#define DBGPIN_LOW	do {} while(0)
#endif
//#define STOP_ALL_ISRS

enum APW_STATE : char {STATE_STOP = 0, STATE_PAUSED = 1, STATE_RUNNING = 2};
static const char *stateStr[] __attribute__((unused)) = {"Stop","Paused","Run"}; //for debug only

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
  if ( NVIC_IS_ENABLED(IRQ_SOFTWARE) )
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
  if (enabled) __enable_irq();
#else
  if (enabled)
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

void apwFile::use(File *f)
{
	file = *f;
	fpos = file.position();
	fileType = APW_FILE_OTHER;
}

void apwFile::open(const char *filename, int mode)
{
  bool irq = stopInt();
  sdFile = SD.sdfs.open(filename, mode);
  fileType = APW_FILE_SD;
	fpos = 0;
  startInt(irq);
  LOGI("Open \"%s\"", filename);
  if (!sdFile) {
    LOGW("Could not open");
  }
}

void apwFile::close(void)
{
	if (!(sdFile || file)) return;
	bool irq = stopInt();
	if (sdFile) sdFile.close();
	if (file) file.close();
	fpos = 0;
	startInt(irq);
  LOGD("File closed");
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
    case APW_FILE_SD : b = sdFile.seek(pos); fpos = pos; break;
    case APW_FILE_OTHER : b = file.seek(pos); fpos = pos; break;
    default : b = false; LOGW("Seek on unknown file type"); break;
  }
  startInt(irq);
  return b;
}

size_t apwFile::position(void)
{
  return fpos;
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

 size_t apwFile::read(void *buf, size_t nbyte)
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

  if (fileType == APW_FILE_SD)
		r = sdFile.read(buf, nbyte);
  else
		r = file.read(buf, nbyte);
	fpos += r;

	if (r < nbyte) LOGE("Read req. 0x%x, got: 0x%x", nbyte, r);

	DBGPIN_LOW;
	return r;
}

size_t apwFile::write(void *buf, size_t nbyte)
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
  if (fileType == APW_FILE_SD)
		r = sdFile.write(buf, nbyte);
	else
		r = file.write(buf, nbyte);
	fpos += r;
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
	asm("":::"memory");
  wavfile.close();
  freeBuffer();
}

FLASHMEM
void AudioBaseWav::reset(void)
{
  state = STATE_STOP;
	asm("":::"memory");
  last_err = ERR_OK;
  dataFmt = APW_NONE;
  buffer = nullptr;
  sample_rate = channels = bytes = 0;
}

bool AudioBaseWav::isPaused(void) {
	asm("":::"memory");
  return (state == STATE_PAUSED);
}

bool AudioBaseWav::isStopped(void) {
	asm("":::"memory");
  return (state == STATE_STOP);
}

bool AudioBaseWav::isRunning(void)
{
	asm("":::"memory");
  return (state == STATE_RUNNING);
}

bool AudioBaseWav::togglePause(void)
{
  APW_STATE s = state;
	if (s != STATE_STOP)
		pause(s == STATE_RUNNING);
	return isPaused();
}

bool AudioBaseWav::addMemory(size_t mult)
{

  if (mult < 1) mult = 1;
  _sz_mem_additional = mult;

  return true;
}


size_t AudioBaseWav::calcBufferIdx(void)
{
 auto idx = (my_instance + ((_instances - 1) - updateStep)) % _instances;
 idx *= AUDIO_BLOCK_SAMPLES * channels * bytes /* * _sz_mem_additional */;
 return idx;
}

FLASHMEM
bool AudioBaseWav::createBuffer(void)
{

	sz_mem = _instances * AUDIO_BLOCK_SAMPLES * channels * bytes * _sz_mem_additional;

	if (wavfile.isSD()) {
		size_t m = MIN_BUFFER_SIZE / sz_mem;
		if (m > 1 ) sz_mem *= m;
	}

	LOGV("malloc() 0x%x bytes", sz_mem);

  buffer = (int8_t*)malloc(sz_mem);

  if (buffer == nullptr) {
    sz_mem = 0;
    last_err = ERR_OUT_OF_MEMORY;
    LOGE("Out of Memory");
		return false;
  }

  return true;
}


void AudioBaseWav::freeBuffer(void)
{
  if (buffer) {
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
HOT static
size_t decode_8bit(int8_t buffer[], audio_block_t *queue[], uint8_t channels)
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
        auto chan = 0;
        do {
          queue[chan]->data[i] = ( *p++ - 128 ) << 8;
        } while (++chan < channels);
      } while (++i < AUDIO_BLOCK_SAMPLES);
      return AUDIO_BLOCK_SAMPLES * channels;
  }
}


// 8 bit signed:
HOT static
size_t decode_8bit_signed(int8_t buffer[], audio_block_t *queue[], uint8_t channels)
{
  int8_t *p = buffer;

  size_t i = 0;
  do {
    auto chan = 0;
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
HOT static
size_t decode_8bit_ulaw(int8_t buffer[], audio_block_t *queue[], uint8_t channels)
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
        auto chan = 0;
        do {
          queue[chan]->data[i] = ulaw_decode[*p++];
        } while (++chan < channels);
      } while (++i < AUDIO_BLOCK_SAMPLES);
      return AUDIO_BLOCK_SAMPLES * channels;
  }
}

// 16 bit:
HOT static
size_t decode_16bit(int8_t buffer[],  audio_block_t *queue[], uint8_t channels)
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
          auto chan = 0;
          do {
            queue[chan]->data[i] = *p++;
          } while (++chan < channels);
        } while (++i < AUDIO_BLOCK_SAMPLES);
        return AUDIO_BLOCK_SAMPLES * 2 * channels;
      }
  }
}

// 16 bit big endian:

HOT static
size_t decode_16bit_bigendian(int8_t buffer[], audio_block_t *queue[], uint8_t channels)
{

  int16_t *p = (int16_t*) buffer;
  size_t i = 0;
  do {
    auto chan = 0;
    do {
      queue[chan]->data[i] = __builtin_bswap16(*p++);
    } while (++chan < channels);
  } while (++i < AUDIO_BLOCK_SAMPLES);
  return AUDIO_BLOCK_SAMPLES * 2 * channels;
}


// 24 bit:
HOT static
size_t decode_24bit(int8_t buffer[], audio_block_t *queue[], uint8_t channels)
{
  //TODO: Optimize
	size_t i = 0;
	uint8_t *p = (uint8_t*) buffer;
	p++;
	do {
		auto chan = 0;
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

bool AudioPlayWav::play(File file, bool paused, bool autorewind)
{
  stop();
  wavfile.reset();
  wavfile.use(&file);
  return _play(APW_NONE, 0, 0, paused, autorewind);
}

bool AudioPlayWav::play(const char *filename, bool paused, bool autorewind)
{
  stop();
  wavfile.reset();
  wavfile.open(filename);
  return _play(APW_NONE, 0, 0, paused, autorewind);
}

bool AudioPlayWav::playRaw(File file, APW_FORMAT fmt, uint32_t sampleRate, uint8_t number_of_channels, bool paused, bool autorewind)
{
  stop();
  wavfile.reset();
  wavfile.use(&file);
  return _play(fmt, sampleRate, number_of_channels, paused, autorewind);
}

bool AudioPlayWav::playRaw(const char *filename, APW_FORMAT fmt, uint32_t sampleRate, uint8_t number_of_channels, bool paused, bool autorewind)
{
  stop();
  wavfile.reset();
  File file = SD.open(filename);
  return _play(fmt, sampleRate, number_of_channels, paused, autorewind);
}

bool AudioPlayWav::_play(APW_FORMAT fmt, uint32_t sampleRate, uint8_t number_of_channels, bool paused, bool autorewind)
{
	LOGI("Play: start");
  this->autorewind = autorewind;
  if (!readHeader(fmt, sampleRate, number_of_channels, paused ? STATE_PAUSED : STATE_RUNNING ))
  {
		LOGI("can not play.");
    stop();
    return false;
  }
  return true;
}

COLD
void AudioPlayWav::stop(void)
{
  APW_STATE oldstate;
  oldstate = state;
  state = STATE_STOP;
	asm("":::"memory"); // don't remove
  wavfile.close();
  freeBuffer();
	reset();
	total_length = 0;
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


COLD
bool AudioPlayWav::readHeader(APW_FORMAT fmt, uint32_t sampleRate, uint8_t number_of_channels, APW_STATE newState)
{

  size_t rd;
  tFileHeader fileHeader;
  tDataHeader dataHeader;

  reset();
  _loopCount = buffer_rd = total_length = channelmask = 0;
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
      if (!COMMread && dataHeader.chunkID == cCOMM) {
        taifcCommonChunk commonChunk;
        rd = wavfile.read(&commonChunk, sizeof(commonChunk));
        if (rd < sizeof(commonChunk)) return false;

        channels = __builtin_bswap16(commonChunk.numChannels);
        commonChunk.sampleSize = __builtin_bswap16(commonChunk.sampleSize);
        bytes = commonChunk.sampleSize / 8;
        total_length = __builtin_bswap32(commonChunk.numSampleFrames) * channels * bytes;

        if (total_length == 0) return false;
        if (commonChunk.sampleSize != 8 && commonChunk.sampleSize != 16) return false;

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

        SSNDread = true;
        if (COMMread) break;
      } ;

      position += sizeof(dataHeader) + dataHeader.chunkSize ;
      if (position & 1) position++; //make position even
    } while (true);

    if (!SSNDread || !COMMread) return false;

  }  // AIFF
  else if ( fileHeader.id == cRIFF &&
                   fileHeader.riffType == cWAVE )
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
        }

        LOGV("Format: %d Bits: %d", fmtHeader.wFormatTag, fmtHeader.wBitsPerSample);
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

  if (channels == 0) return false;
  if (dataFmt == APW_NONE) return false;
  if (sample_rate == 0) sample_rate = AUDIO_SAMPLE_RATE_EXACT;

  last_err = ERR_OK;
  bytes = bytesPerSample[dataFmt];
  decoder = decoders[dataFmt];
  padding = (dataFmt == APW_8BIT_UNSIGNED) ? 0x80 : 0x00;
  LOGV("SampleRate: %d, Channels: %d, Bytes: %d, Length: %dms",
       sample_rate, channels, bytes,
       round( msPerSample * total_length / (bytes * channels) )
      );

	size_t wfp = wavfile.position();
	size_t wfz = wavfile.size();

	//the total_length may be wrong. try to adjust it.
	if (total_length > wfz - wfp) total_length = wfz - wfp;

  firstSampleOffset = wfp;
	lastSample = firstSampleOffset + total_length;

  if (!createBuffer()) return false;
  buffer_rd = sz_mem;

  LOGD("Start with state \"%s\".", stateStr[newState]);

	if (newState == STATE_RUNNING) start();
	else state = STATE_PAUSED;

  return true;
}

bool AudioPlayWav::setPosition(size_t sample)
{
	if (state == STATE_STOP) return false;
	LOGD("setPosition(0x%x)", sample);
	if (state == STATE_PAUSED) {
		bool irq = stopInt();
		sample = sample * bytes * channels + firstSampleOffset;
		if (sample < lastSample - AUDIO_BLOCK_SAMPLES)
		{
			buffer_rd = sz_mem;
			wavfile.seek(sample);
		}
		startInt(irq);
		return true;
	}
	/*
	if (state == STATE_RUNNING)
	Im Detail steckt der Teufel... :-)
	Dies  sollte möglicherweise nicht implementiert werden.
	-> Durch die interleaved reads und den großen buffer ist es nicht möglich,
	die Position zeitnah zu setzen.
	Für die loops ist es möglich, da es im voraus geschieht.
	Für setPosition() geht das aber nicht!
  */
	LOGE("setPosition(): Not implemented.");
	return false;
}

void AudioPlayWav::loop(bool enable)
{
	if (state == STATE_STOP) return;
	if (enable && _loopCount == 0)
	{
		_loopCount = -1;
		loopFirst = firstSampleOffset;
		loopLast = lastSample;
	}
	else
		_loopCount = 0;
}

void AudioPlayWav::loop(size_t first, size_t last, uint16_t count)
{
 if (state == STATE_STOP) return;

 bool irq = stopInt();
 loopFirst = first * bytes * channels + firstSampleOffset;
 if (last == 0) loopLast = lastSample;
 else loopLast = last * bytes * channels + firstSampleOffset;

 if (loopLast - loopFirst < sz_mem) loopLast = loopFirst + sz_mem;
 if (loopLast > lastSample) loopLast = lastSample;
 if (loopLast <= loopFirst) count = 0;
 LOGD("loop(0x%x, 0x%x, %d)", loopFirst, loopLast, count);

 if (wavfile.position() >= loopLast) count += 1;
 _loopCount = count;
 startInt(irq);
}

int AudioPlayWav::loopCount(void) {
	asm("":::"memory"); //<- don't remove, will not work in a loop without!
	return _loopCount;
}

HOT
size_t AudioPlayWav::dataReader(int8_t *buffer, int len)
{
	// this runs in update();

	/*
	Loops:
	IF enabled {
	  IF read would read above the loop-end-position
		  (the loop-end-position MUST NOT exceed EOF)
		then:
		- read until loop-end-position
		- seek(loop-start-position)
		- read the remaining data
		return
	}
	- just read
	- on EOF fill remaining data with "padding"

	On EOF, the dataReader should return a number < len
	*/

  int rd;

	if (_loopCount > 0)
	{
		size_t pos = wavfile.position();
		if (pos + len > loopLast)
		{
			_loopCount--;
			LOGD("LoopCount: %d", _loopCount);
			LOGV("Loop-1: last: 0x%x pos: 0x%x diff: 0x%x len: 0x%x", loopLast, pos, loopLast - pos, len);
			int l = loopLast - pos;
			if (l > 0)
			{
				if (l > len ) l = len;
				LOGV("Loop-2: read: 0x%x", l);
				rd = wavfile.readInISR(buffer, l);
			}
		  else
				rd = 0;

			wavfile.seek(loopFirst);
			LOGV("Loop-3: pos: 0x%x read: 0x%x ", loopFirst, len - rd);
			rd += wavfile.readInISR(buffer + rd, len - rd);
			return rd;
		}
	}

	rd = wavfile.readInISR(buffer, len);

  // here would be right place to add playing backwards, up-/downsampling etc ....

	if ( rd < len )
	{
		LOGV("EOF: 0x%x Bytes read, needed: 0x%x -> fill 0x%x with 0x%x", rd, len, len - rd, padding);
		memset(buffer + rd, padding, len - rd);
	}

	return rd;
}

COLD
void AudioPlayWav::start(void)
{
	size_t len, buffer_rd_old;

	LOGV("Play: Pause -> RUN");
  bool irq = stopInt();

	buffer_rd_old = buffer_rd;
  buffer_rd = sz_mem - calcBufferIdx();

	// pre-load according to instance number and update step

	if (buffer_rd < sz_mem) //Do we need to read data?
	{
		len = sz_mem - buffer_rd;
		// we need some data!
		// 1. are there remaining data in the buffer? (maybe we had paused?)
		if (buffer_rd_old == sz_mem)
			goto readData; // Nope!

		// We can use the already read data
		if (buffer_rd_old == buffer_rd)
			goto end; // 1.a ) amazing! exactly the amount we need.

		else if (buffer_rd_old < buffer_rd)
		{ // 1.b ) it is too much! No other way than to throw a part of them away.

			LOGV("Too much data.");
			len = sz_mem - buffer_rd;
			if (len > 0) memmove(&buffer[buffer_rd], &buffer[buffer_rd_old], len);
			goto end;
		}

#if 1
		// 1.c ) but it is too less! Good news: we can use some.
		// else if (buffer_rd_old > buffer_rd)
		{
			size_t elen = sz_mem - buffer_rd_old; //Amount of data we already have.

			LOGV("Move %d already read bytes from 0x%x to 0x%x", elen, buffer_rd_old, buffer_rd);
			memmove(&buffer[buffer_rd], &buffer[buffer_rd_old], elen);
			len -= elen;
			dataReader(&buffer[buffer_rd + elen], len);
			goto end;
		}

#endif

readData:
  // 2. load data from file
	  dataReader(&buffer[buffer_rd], len);
  }

end:
  // 4. (re)start the whole thing
	state = STATE_RUNNING;
	startInt(irq);
}


bool AudioPlayWav::pause(bool pause)
{
	APW_STATE s = state;

	if (pause) {
		if (s == STATE_RUNNING) state = STATE_PAUSED;
	} else {
		if (s == STATE_PAUSED) start();
	}

	LOGV("State: \"%s\"", stateStr[state]);
  return isPaused();
}

COLD
void AudioPlayWav::stopFromUpdate(void)
{
	if (autorewind) {
		state = STATE_PAUSED;
		asm("":::"memory");// dont't remove
		buffer_rd = sz_mem;
		wavfile.seek(firstSampleOffset);
	}
	else stop();
}

HOT
void AudioPlayWav::update(void)
{

  if (++updateStep >= _instances) updateStep = 0;
  if (state != STATE_RUNNING) return;

  if (buffer_rd >= sz_mem) buffer_rd = 0;

	bool last = false;
  if (updateStep == my_instance /*&& buffer_rd == 0*/)
  {
    size_t len = sz_mem - buffer_rd;
		size_t rd = dataReader(&buffer[buffer_rd], len);
		if (rd == 0) {
			stopFromUpdate();
			return;
		}
		last = rd < len;
		buffer_rd = sz_mem - len;
  }

  // allocate the audio blocks to transmit
	auto chan = channels;
	audio_block_t *queue[channels];
  do {
		chan--;
		audio_block_t *p;
    p = AudioStream::allocate();
    if ( p == nullptr ) {
				while(chan < channels)
					AudioStream::release(queue[chan]);
				last_err = ERR_NO_AUDIOBLOCKS;
				LOGE("Waveplayer stopped: Not enough AudioMemory().");
				stop();
				return;

		}
		queue[chan] = p;
  } while (chan > 0);

  // copy the samples to the audio blocks:
  buffer_rd += decoder(&buffer[buffer_rd], queue, channels);

  // transmit them:
	chan = 0;
  do
  {
    AudioStream::transmit(queue[chan], chan);
    AudioStream::release(queue[chan]);
  } while (++chan < channels);

  if (last) stopFromUpdate();

}

size_t AudioPlayWav::position()
{
#if 0 // not needed anymore.
  uint32_t safe_read;
  int data_length;
	APW_STATE state;

  //use an interrupt detector to make sure all vars are consistent.
  //strex will fail if an interrupt occured. An other way would be to block audio interrupts.
  do
  {
    __ldrexw(&safe_read);
		state = this->state;
    data_length = this->data_length;
		asm("":::"memory");
  } while ( __strexw(1, &safe_read) );


#endif
  asm("":::"memory");
  if (state == STATE_STOP) return 0;
  return (wavfile.position() - firstSampleOffset + sz_mem - buffer_rd) / (bytes * channels);
}

uint32_t AudioPlayWav::positionMillis(void)
{
  return roundf( position() * msPerSample);
}

uint32_t AudioPlayWav::lengthMillis(void)
{
	asm("":::"memory");
  //if (state == STATE_STOP) return 0; // will return 0 anyway
  return roundf( msPerSample * total_length / (bytes * channels) );
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
HOT static
size_t encode_8bit(int8_t buffer[], audio_block_t *queue[], uint8_t channels)
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
        auto chan = 0;
        do {
          *p++ = (queue[chan]->data[i] >> 8) + 128;
        } while (++chan < channels);
      } while (++i < AUDIO_BLOCK_SAMPLES);
      return AUDIO_BLOCK_SAMPLES * channels;
  }
}
// 16 bit:
HOT static
size_t encode_16bit(int8_t buffer[], audio_block_t *queue[], uint8_t channels)
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
          auto chan = 0;
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

void AudioRecordWav::stop()
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

	if (wavfile) {
    writeHeader(wavfile);
    wavfile.truncate();
    wavfile.close();
  }

}

bool AudioRecordWav::pause(const bool pause)
{
  if (pause && state != STATE_PAUSED) {
    if (wavfile) writeHeader(wavfile);
  }

  switch (state)
  {
    case STATE_STOP: break;
    case STATE_PAUSED: state = STATE_RUNNING; break;
    case STATE_RUNNING: state = STATE_PAUSED; break;
  }

	return isPaused();
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
  wavfile.use(&file);
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

COLD
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

COLD
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

  if (!createBuffer()) return false;

  bool irq = stopInt();

  buffer_wr = sz_mem - calcBufferIdx();
  if (buffer_wr >= sz_mem) buffer_wr = 0;
  buffer_wr_start = buffer_wr;

  state = paused ? STATE_PAUSED : STATE_RUNNING;
  startInt(irq);

  return true;
}

HOT
void  AudioRecordWav::update(void)
{
  if (++updateStep >= _instances) updateStep = 0;
  if (state != STATE_RUNNING) return;

  auto chan = 0;
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
    if (queue[chan] == nullptr) continue;

    AudioStream::release(queue[chan]);
    queue[chan] = nullptr; // < why is this needed?

  } while (++chan < channels);

	++data_length;
}
#endif //enable

#endif
