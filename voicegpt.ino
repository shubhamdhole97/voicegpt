#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "driver/i2s.h"
#include "esp_heap_caps.h"
#include "esp_system.h"

// =========================
// Wi-Fi
// =========================
const char* WIFI_SSID = "Mi Home";
const char* WIFI_PASS = "8087891105";

// EC2 backend API
const char* API_URL = "http://3.110.130.40:8000/api/esp32/voice-chat";

// =========================
// I2S Ports
// =========================
#define MIC_PORT I2S_NUM_0
#define SPK_PORT I2S_NUM_1

// INMP441 MIC pins
#define MIC_BCLK 14
#define MIC_WS   15
#define MIC_SD   32

// MAX98357A Speaker pins
#define SPK_BCLK 26
#define SPK_LRC  25
#define SPK_DIN  22

// =========================
// Audio Config
// =========================
#define SAMPLE_RATE 16000
#define SPEAKER_RATE 24000
#define BUFFER_SIZE 512
#define RECORD_SECONDS 2

// =========================
// Mic Noise Control
// =========================
// Increase this if ESP32 sends noise/silence.
// Decrease slightly if voice is not detected.
#define NOISE_GATE 1200

// Mic recording volume divider.
// Lower value = louder mic recording.
// Keep 3 for less noise.
#define VOLUME_DIV 3

#define MAX_VOLUME 18000

// =========================
// Speaker Volume Control
// =========================
// Increase if speaker is low.
// If speaker cracks/noisy, reduce to 1.5
#define SPEAKER_GAIN 3.0
#define SPEAKER_LIMIT 32000

// =========================
// Voice Detection
// =========================
// Increased to avoid false trigger from noise.
#define VOICE_DETECT_LEVEL 1200
#define VOICE_DETECT_MAX   5000
#define COOLDOWN_MS 2500

int32_t micBuffer[BUFFER_SIZE];

unsigned long lastPrint = 0;

// Filter variables
float smoothSample = 0;
float dcOffset = 0;

uint8_t* wavBuffer = NULL;
size_t wavSize = 0;

// =========================
// WAV Header
// =========================
void writeWavHeader(uint8_t* header, uint32_t dataSize) {
  uint32_t fileSize = dataSize + 36;
  uint32_t byteRate = SAMPLE_RATE * 1 * 16 / 8;
  uint16_t blockAlign = 1 * 16 / 8;

  memcpy(header, "RIFF", 4);

  header[4] = fileSize & 0xff;
  header[5] = (fileSize >> 8) & 0xff;
  header[6] = (fileSize >> 16) & 0xff;
  header[7] = (fileSize >> 24) & 0xff;

  memcpy(header + 8, "WAVE", 4);
  memcpy(header + 12, "fmt ", 4);

  header[16] = 16;
  header[17] = 0;
  header[18] = 0;
  header[19] = 0;

  header[20] = 1;
  header[21] = 0;

  header[22] = 1;
  header[23] = 0;

  header[24] = SAMPLE_RATE & 0xff;
  header[25] = (SAMPLE_RATE >> 8) & 0xff;
  header[26] = (SAMPLE_RATE >> 16) & 0xff;
  header[27] = (SAMPLE_RATE >> 24) & 0xff;

  header[28] = byteRate & 0xff;
  header[29] = (byteRate >> 8) & 0xff;
  header[30] = (byteRate >> 16) & 0xff;
  header[31] = (byteRate >> 24) & 0xff;

  header[32] = blockAlign;
  header[33] = 0;

  header[34] = 16;
  header[35] = 0;

  memcpy(header + 36, "data", 4);

  header[40] = dataSize & 0xff;
  header[41] = (dataSize >> 8) & 0xff;
  header[42] = (dataSize >> 16) & 0xff;
  header[43] = (dataSize >> 24) & 0xff;
}

// =========================
// Wi-Fi
// =========================
void connectWiFi() {
  Serial.print("Connecting Wi-Fi");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("Wi-Fi connected");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());
}

// =========================
// Mic Setup
// =========================
void setupMic() {
  i2s_config_t micConfig = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t micPins;
  micPins.mck_io_num = I2S_PIN_NO_CHANGE;
  micPins.bck_io_num = MIC_BCLK;
  micPins.ws_io_num = MIC_WS;
  micPins.data_out_num = I2S_PIN_NO_CHANGE;
  micPins.data_in_num = MIC_SD;

  i2s_driver_install(MIC_PORT, &micConfig, 0, NULL);
  i2s_set_pin(MIC_PORT, &micPins);
  i2s_zero_dma_buffer(MIC_PORT);

  Serial.println("Mic ready");
}

// =========================
// Speaker Setup
// =========================
void setupSpeaker() {
  i2s_config_t spkConfig = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SPEAKER_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };

  i2s_pin_config_t spkPins;
  spkPins.mck_io_num = I2S_PIN_NO_CHANGE;
  spkPins.bck_io_num = SPK_BCLK;
  spkPins.ws_io_num = SPK_LRC;
  spkPins.data_out_num = SPK_DIN;
  spkPins.data_in_num = I2S_PIN_NO_CHANGE;

  i2s_driver_install(SPK_PORT, &spkConfig, 0, NULL);
  i2s_set_pin(SPK_PORT, &spkPins);
  i2s_zero_dma_buffer(SPK_PORT);

  Serial.println("Speaker ready");
}

// =========================
// Mic Filter
// =========================
int16_t processMicSample(int32_t raw32) {
  int32_t rawSample = raw32 >> 14;

  // Remove DC offset / hum
  dcOffset = (dcOffset * 0.995) + (rawSample * 0.005);
  int32_t sample = rawSample - dcOffset;

  // Noise gate
  if (abs(sample) < NOISE_GATE) {
    sample = 0;
  }

  // Smooth filter
  smoothSample = (smoothSample * 0.75) + (sample * 0.25);
  sample = (int32_t)smoothSample;

  // Mic volume control
  sample = sample / VOLUME_DIV;

  // Limiter
  if (sample > MAX_VOLUME) sample = MAX_VOLUME;
  if (sample < -MAX_VOLUME) sample = -MAX_VOLUME;

  return (int16_t)sample;
}

// =========================
// Detect Voice
// =========================
bool voiceDetected() {
  size_t bytesRead = 0;

  i2s_read(MIC_PORT, micBuffer, sizeof(micBuffer), &bytesRead, 100 / portTICK_PERIOD_MS);

  if (bytesRead == 0) {
    return false;
  }

  int samples = bytesRead / 4;
  long level = 0;
  int16_t maxSample = 0;

  for (int i = 0; i < samples; i++) {
    int16_t sample = processMicSample(micBuffer[i]);

    int absVal = abs(sample);
    level += absVal;

    if (absVal > maxSample) {
      maxSample = absVal;
    }
  }

  if (samples > 0) {
    level = level / samples;
  }

  if (millis() - lastPrint > 500) {
    Serial.print("Mic Level: ");
    Serial.print(level);
    Serial.print(" Max: ");
    Serial.println(maxSample);
    lastPrint = millis();
  }

  if (level > VOICE_DETECT_LEVEL || maxSample > VOICE_DETECT_MAX) {
    return true;
  }

  return false;
}

// =========================
// Record WAV
// =========================
bool recordWav() {
  Serial.println("Voice detected. Recording...");

  uint32_t pcmDataSize = SAMPLE_RATE * RECORD_SECONDS * 2;
  wavSize = pcmDataSize + 44;

  if (wavBuffer != NULL) {
    free(wavBuffer);
    wavBuffer = NULL;
  }

  if (psramFound()) {
    Serial.println("Using PSRAM");
    wavBuffer = (uint8_t*)ps_malloc(wavSize);
  } else {
    Serial.println("PSRAM not found, using normal RAM");
    wavBuffer = (uint8_t*)malloc(wavSize);
  }

  if (!wavBuffer) {
    Serial.println("Memory allocation failed");
    Serial.print("Required bytes: ");
    Serial.println(wavSize);
    Serial.print("Free heap: ");
    Serial.println(ESP.getFreeHeap());
    return false;
  }

  writeWavHeader(wavBuffer, pcmDataSize);

  uint32_t writeIndex = 44;
  uint32_t samplesNeeded = SAMPLE_RATE * RECORD_SECONDS;
  uint32_t samplesWritten = 0;

  long long sumAbs = 0;
  int16_t maxSample = 0;

  while (samplesWritten < samplesNeeded) {
    size_t bytesRead = 0;
    i2s_read(MIC_PORT, micBuffer, sizeof(micBuffer), &bytesRead, portMAX_DELAY);

    int samples = bytesRead / 4;

    for (int i = 0; i < samples; i++) {
      if (samplesWritten >= samplesNeeded) {
        break;
      }

      int16_t sample16 = processMicSample(micBuffer[i]);

      int absVal = abs(sample16);
      sumAbs += absVal;

      if (absVal > maxSample) {
        maxSample = absVal;
      }

      wavBuffer[writeIndex++] = sample16 & 0xff;
      wavBuffer[writeIndex++] = (sample16 >> 8) & 0xff;

      samplesWritten++;
    }
  }

  Serial.print("Recording done. WAV bytes: ");
  Serial.println(wavSize);

  Serial.print("Recording max sample: ");
  Serial.println(maxSample);

  Serial.print("Recording avg level: ");
  Serial.println(sumAbs / samplesWritten);

  return true;
}

// =========================
// Speaker Volume Increase
// =========================
void increaseSpeakerVolume(uint8_t* buffer, int len) {
  for (int i = 0; i + 1 < len; i += 2) {
    int16_t sample = buffer[i] | (buffer[i + 1] << 8);

    int32_t out = sample * SPEAKER_GAIN;

    // Limiter to avoid cracking/noise
    if (out > SPEAKER_LIMIT) out = SPEAKER_LIMIT;
    if (out < -SPEAKER_LIMIT) out = -SPEAKER_LIMIT;

    sample = (int16_t)out;

    buffer[i] = sample & 0xff;
    buffer[i + 1] = (sample >> 8) & 0xff;
  }
}

// =========================
// Send to Backend and Play AI Answer
// =========================
void sendToBackendAndPlay() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi disconnected. Reconnecting...");
    connectWiFi();
  }

  HTTPClient http;
  http.begin(API_URL);
  http.addHeader("Content-Type", "audio/wav");

  Serial.println("Sending audio to backend...");

  int httpCode = http.POST(wavBuffer, wavSize);

  Serial.print("HTTP code: ");
  Serial.println(httpCode);

  if (httpCode != 200) {
    String error = http.getString();
    Serial.println("Backend error:");
    Serial.println(error);
    http.end();
    return;
  }

  WiFiClient* stream = http.getStreamPtr();

  uint8_t audioBuffer[1024];
  int totalPlayed = 0;

  int contentLength = http.getSize();
  int remaining = contentLength;

  Serial.print("Audio content length: ");
  Serial.println(contentLength);

  Serial.println("Playing AI answer from speaker...");

  // Clear old speaker buffer before new answer
  i2s_zero_dma_buffer(SPK_PORT);

  unsigned long lastDataTime = millis();

  while (http.connected() && (remaining > 0 || remaining == -1)) {
    int availableBytes = stream->available();

    if (availableBytes > 0) {
      int toRead = availableBytes;

      if (toRead > 1024) {
        toRead = 1024;
      }

      if (remaining > 0 && toRead > remaining) {
        toRead = remaining;
      }

      int readLen = stream->readBytes(audioBuffer, toRead);

      if (readLen > 0) {
        lastDataTime = millis();

        increaseSpeakerVolume(audioBuffer, readLen);

        size_t bytesWritten = 0;
        i2s_write(SPK_PORT, audioBuffer, readLen, &bytesWritten, portMAX_DELAY);

        totalPlayed += bytesWritten;

        if (remaining > 0) {
          remaining -= readLen;
        }
      }
    } else {
      delay(2);

      // Safety timeout if stream gets stuck
      if (millis() - lastDataTime > 5000) {
        Serial.println("Playback stream timeout");
        break;
      }
    }

    if (remaining == 0) {
      break;
    }
  }

  // Small delay so last audio chunk fully plays
  delay(100);

  Serial.print("Playback done. Bytes played: ");
  Serial.println(totalPlayed);

  http.end();
}

// =========================
// Setup
// =========================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("VoiceGPT ESP32 Starting...");

  Serial.print("Reset reason: ");
  Serial.println(esp_reset_reason());

  Serial.print("Free heap: ");
  Serial.println(ESP.getFreeHeap());

  Serial.print("PSRAM found: ");
  Serial.println(psramFound() ? "YES" : "NO");

  connectWiFi();
  setupMic();
  setupSpeaker();

  Serial.println("VoiceGPT ESP32 ready");
  Serial.println("Speak near mic. No button needed.");
}

// =========================
// Loop
// =========================
void loop() {
  if (voiceDetected()) {
    bool ok = recordWav();

    if (ok) {
      sendToBackendAndPlay();
    }

    if (wavBuffer != NULL) {
      free(wavBuffer);
      wavBuffer = NULL;
    }

    Serial.println("Cooldown...");
    delay(COOLDOWN_MS);

    Serial.println("Listening again...");
  }

  delay(150);
}