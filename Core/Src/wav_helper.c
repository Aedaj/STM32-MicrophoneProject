#include "wav_helper.h"

// reference: https://ccrma.stanford.edu/courses/422-winter-2014/projects/WaveFormat/

#define AUDIO_FORMAT 1
#define NUM_CHANNELS 1
#define SAMPLE_RATE 18181
#define BITS_PER_SAMPLE 16
#define SUBCHUNK1_SIZE 16



void init_wav_header(wav_header_t *wav_header)
{
	// Init RIFF Header
	// big-endian format
	wav_header->ChunkID[0] = 'R';
	wav_header->ChunkID[1] = 'I';
	wav_header->ChunkID[2] = 'F';
	wav_header->ChunkID[3] = 'F';

	wav_header->ChunkSize = 4 + (8 + SUBCHUNK1_SIZE) + (8 + 0); // subchunk2size is 0 as a placeholder

	// big-endian format
	wav_header->Format[0] = 'W';
	wav_header->Format[1] = 'A';
	wav_header->Format[2] = 'V';
	wav_header->Format[3] = 'E';

	// Init "fmt " Subchunk
	// big-endian format
	wav_header->Subchunk1ID[0] = 'f';
	wav_header->Subchunk1ID[1] = 'm';
	wav_header->Subchunk1ID[2] = 't';
	wav_header->Subchunk1ID[3] = ' ';

	wav_header->Subchunk1Size = SUBCHUNK1_SIZE;

	wav_header->AudioFormat = AUDIO_FORMAT; // 1: PCM, other values indicate some kind of compression
	wav_header->NumChannels = NUM_CHANNELS; // 1: Mono, 2: Stereo, etc.
	wav_header->SampleRate = SAMPLE_RATE; // 18.2KHz
	wav_header->BitsPerSample = BITS_PER_SAMPLE; // 12 bits per sample, but we're using 16 for block alignment
	wav_header->ByteRate = SAMPLE_RATE * NUM_CHANNELS * (BITS_PER_SAMPLE/8);
	wav_header->BlockAlign = NUM_CHANNELS * (BITS_PER_SAMPLE/8);

	// Init "data" Subchunk
	// big-endian format
	wav_header->Subchunk2ID[0] = 'd';
	wav_header->Subchunk2ID[1] = 'a';
	wav_header->Subchunk2ID[2] = 't';
	wav_header->Subchunk2ID[3] = 'a';

	wav_header->Subchunk2Size = 0; // Placeholder
}


// return 0 if successful, -1 if generic failure, -2 if failure with a partial write
int create_new_wavfile(wav_header_t *wav_h, char *path, int is_raw)
{
	init_wav_header(wav_h);

	// save the wav_header to the SD card
	FRESULT fresult;
	FIL fp;
	UINT bw;
	char *format = (is_raw == 1) ? "raw-%i.wav" : "mod-%i.wav";
	sprintf(path, format, 0);
	int i = 0;
	while ((fresult = f_stat(path, 0)) == FR_OK) { // while the filename exists, do:
		i++;
		sprintf(path, format, i); // increment the filename
	}

	// open the file
	fresult = f_open(&fp, path, FA_CREATE_ALWAYS | FA_WRITE);
	if (fresult != FR_OK) {
		f_close(&fp);
		return -1;
	}
	// start from the beginning
	fresult = f_lseek(&fp, 0);
	if (fresult != FR_OK) {
		f_close(&fp);
		return -1;
	}

	// write each field individually, as if we just write the struct, the strings won't be properly written (it will write the pointer values instead of the strings)

	fresult = f_write(&fp, wav_h->ChunkID, 4, &bw);
	if (fresult != FR_OK) {
		return -1;
	}
	fresult = f_write(&fp, &(wav_h->ChunkSize), 4, &bw);
	if (fresult != FR_OK) {
		return -1;
	}
	fresult = f_write(&fp, wav_h->Format, 4, &bw);
	if (fresult != FR_OK) {
		return -1;
	}
	fresult = f_write(&fp, wav_h->Subchunk1ID, 4, &bw);
	if (fresult != FR_OK) {
		return -1;
	}
	fresult = f_write(&fp, &(wav_h->Subchunk1Size), 20, &bw);
	if (fresult != FR_OK) {
		return -1;
	}
	fresult = f_write(&fp, wav_h->Subchunk2ID, 4, &bw);
	if (fresult != FR_OK) {
		return -1;
	}
	fresult = f_write(&fp, &(wav_h->Subchunk2Size), 4, &bw);
	if (fresult != FR_OK) {
		return -1;
	}
	fresult = f_close(&fp);
	return 0;
}


int update_wavfile(char *path, wav_header_t *wav_header, uint16_t *buffer, uint32_t buffer_size)
{
	// update wav header metadata (size fields)
	wav_header->Subchunk2Size += buffer_size * NUM_CHANNELS * (BITS_PER_SAMPLE/8);
	wav_header->ChunkSize = 4 + (8 + wav_header->Subchunk1Size) + (8 + wav_header->Subchunk2Size);

	UINT bw;
	FIL fp;

	// open existing file
	FRESULT fresult = f_open(&fp, path, FA_OPEN_ALWAYS | FA_WRITE);
	if (fresult != FR_OK) {
		f_close(&fp);
		return -1;
	}
	// Seek to the ChunkSize field in the .wav file
	fresult = f_lseek(&fp, 4);
	if (fresult != FR_OK) {
		f_close(&fp);
		return -1;
	}
	// update the overall chunk size
	fresult = f_write(&fp, &wav_header->ChunkSize, 4, &bw);
	if (fresult != FR_OK) {
		f_close(&fp);
		if (bw != sizeof(wav_header->ChunkSize)) {
			return -2;
		}
		return -1;
	}
	fresult = f_lseek(&fp, 40); // offset for Subchunk2Size field
	if (fresult != FR_OK) {
		f_close(&fp);
		return -1;
	}
	// update the subchunk2size (subchunk2 is the chunk housing the audio data
	fresult = f_write(&fp, &wav_header->Subchunk2Size, 4, &bw);
	if (fresult != FR_OK) {
		f_close(&fp);
		if (bw != sizeof(wav_header->Subchunk2Size)) {
			return -2;
		}
		return -1;
	}
	// begin writing the audio signal data to the data chunk
	fresult = f_lseek(&fp, f_size(&fp));
	if (fresult != FR_OK) {
		f_close(&fp);
		return -1;
	}
	fresult = f_write(&fp, buffer, buffer_size, &bw);
	if (fresult != FR_OK) {
		f_close(&fp);
		if (bw != buffer_size) {
			return -2;
		}
		return -1;
	}
	f_close(&fp);
	return 0;
}
