#include "fatfs.h"
#include "stdio.h"

// reference: https://ccrma.stanford.edu/courses/422-winter-2014/projects/WaveFormat/

typedef struct wav_header_s {
	// RIFF Header
	char ChunkID[4];
	uint32_t ChunkSize;
	char Format[4];

	// "fmt " subchunk
	char Subchunk1ID[4];
	uint32_t Subchunk1Size;
	uint16_t AudioFormat;
	uint16_t NumChannels;
	uint32_t SampleRate;
	uint32_t ByteRate;
	uint16_t BlockAlign;
	uint16_t BitsPerSample;

	// "data" subchunk;
	char Subchunk2ID[4];
	uint32_t Subchunk2Size;
	// the actual raw data starts here, although we'll write it separately to the header
} wav_header_t;

FRESULT open_append(FIL* fp, const char* path);

void init_wav_header(wav_header_t *wav_header);

int create_new_wavfile(wav_header_t *wav_h, char *path, int is_raw);

int update_wavfile(char *path, wav_header_t *wav_header, uint16_t *buffer, uint32_t buffer_size);
