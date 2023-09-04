#include <SD.h>
#include <SPI.h>
int MIN_DATA_VALUE = 0;
int MAX_DATA_VALUE = 555;

File wavFile;
const char* filename = "TEN_FILE.wav";

/// The first 4 byte of a wav file should be the characters "RIFF" */
char chunkID[4] = {'R', 'I', 'F', 'F'};
/// 36 + SubChunk2Size
uint32_t chunkSize = 36; // You Don't know this until you write your data but at a minimum it is 36 for an empty file
/// "should be characters "WAVE"
char format[4] = {'W', 'A', 'V', 'E'};
/// " This should be the letters "fmt ", note the space character
char subChunk1ID[4] = {'f', 'm', 't', ' '};
///: For PCM == 16, since audioFormat == uint16_t
uint32_t subChunk1Size = 16;
///: For PCM this is 1, other values indicate compression
uint16_t audioFormat = 1;
///: Mono = 1, Stereo = 2, etc.
uint16_t numChannels = 1;
///: Sample Rate of file
uint32_t sampleRate = 44100;
///: SampleRate * NumChannels * BitsPerSample/8
uint32_t byteRate = 44100 * 2;
///: The number of byte for one frame NumChannels * BitsPerSample/8
uint16_t blockAlign = 2;
///: 8 bits = 8, 16 bits = 16
uint16_t bitsPerSample = 16;
///: Contains the letters "data"
char subChunk2ID[4] = {'d', 'a', 't', 'a'};
///: == NumSamples * NumChannels * BitsPerSample/8  i.e. number of byte in the data.
uint32_t subChunk2Size = 0; // You Don't know this until you write your data

int16_t sampleValue;

void writeWavHeader()
{
   wavFile.seek(0);
   wavFile.write(chunkID,4);
   wavFile.write((byte*)&chunkSize,4);
   wavFile.write(format,4);
   wavFile.write(subChunk1ID,4);
   wavFile.write((byte*)&subChunk1Size,4);
   wavFile.write((byte*)&audioFormat,2);
   wavFile.write((byte*)&numChannels,2);
   wavFile.write((byte*)&sampleRate,4);
   wavFile.write((byte*)&byteRate,4);
   wavFile.write((byte*)&blockAlign,2);
   wavFile.write((byte*)&bitsPerSample,2);
   wavFile.write(subChunk2ID,4);
   wavFile.write((byte*)&subChunk2Size,4);
}


void writeDataToWavFile(int data)
{
  sampleValue = map(data, MIN_DATA_VALUE, MAX_DATA_VALUE,-32767,32767);

  subChunk2Size += numChannels * bitsPerSample/8;
  wavFile.seek(40);
  wavFile.write((byte*)&subChunk2Size,4);

  wavFile.seek(4);
  chunkSize = 36 + subChunk2Size;
  wavFile.write((byte*)&chunkSize,4);

  int n = wavFile.size() - 1;
  wavFile.seek(n);
  wavFile.write((byte*)&sampleValue,4);
}

void setup()
{
  
  Serial.begin(115200);
  pinMode(A0, INPUT);
  while (!Serial);

  if (!SD.begin(10)) {
    Serial.println("Fail to connect to SD");
    while (1);
  }

  SD.remove(filename);
  wavFile = SD.open(filename, FILE_WRITE);


  if (!wavFile) {
    Serial.println("Fail to open file");
    while (1);
  }
  writeWavHeader();

}

void loop()
{
  int data = analogRead(A0);
  Serial.println(data);
  writeDataToWavFile(data);
}