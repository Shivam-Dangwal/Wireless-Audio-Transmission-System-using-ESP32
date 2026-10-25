/* Transmitter - INMP441 -> ESP-NOW (µ-law, fixed frames) */
#include <WiFi.h>
#include <esp_now.h>
#include <driver/i2s.h>

#define SAMPLE_RATE 16000
#define FRAME_SIZE 200          // mono samples per packet (200 bytes payload)
#define PAYLOAD_SIZE (1 + FRAME_SIZE) // 1 byte seq + FRAME_SIZE bytes

uint8_t receiverAddress[] = {0x38, 0x18, 0x2B, 0x8A, 0x7A, 0xC4}; // <-- change

// I2S mic config (legacy API)
i2s_config_t i2s_mic_config = {
  .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
  .sample_rate = SAMPLE_RATE,
  .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
  .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
  .communication_format = I2S_COMM_FORMAT_I2S_MSB,
  .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
  .dma_buf_count = 4,
  .dma_buf_len = FRAME_SIZE,
  .use_apll = false,
  .tx_desc_auto_clear = false,
  .fixed_mclk = 0
};

i2s_pin_config_t i2s_mic_pin_config = {
  .bck_io_num = 26,
  .ws_io_num  = 25,
  .data_out_num = -1,
  .data_in_num  = 22
};

void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  // we don't inspect info fields (varies by core); keep small
}

uint8_t seq = 0;
uint8_t packet[PAYLOAD_SIZE];

static inline uint8_t mu_law_encode(int16_t sample) {
  // µ-law standard companding (A-law alternative possible)
  const uint16_t BIAS = 0x84; // 132
  uint16_t sign = (sample < 0) ? 0x80 : 0x00;
  uint16_t magnitude = (sample < 0) ? -sample : sample;
  magnitude += BIAS;
  if (magnitude > 32635) magnitude = 32635;
  int exponent = 7;
  for (int expMask = 0x4000; (magnitude & expMask) == 0 && exponent > 0; expMask >>= 1) exponent--;
  int mantissa = (magnitude >> (exponent + 3)) & 0x0F;
  uint8_t mu = ~(sign | (exponent << 4) | mantissa);
  return mu;
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  i2s_driver_install(I2S_NUM_0, &i2s_mic_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &i2s_mic_pin_config);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init fail");
    while(1);
  }
  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, receiverAddress, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  Serial.println("TX ready");
}

void loop() {
  // read FRAME_SIZE 32-bit samples
  int32_t raw[FRAME_SIZE];
  size_t bytes_read = 0;
  i2s_read(I2S_NUM_0, raw, sizeof(raw), &bytes_read, portMAX_DELAY);
  int samples = bytes_read / 4;
  if (samples < FRAME_SIZE) {
    // if fewer samples read, pad zeros (shouldn't normally happen)
    for (int i = samples; i < FRAME_SIZE; ++i) raw[i] = 0;
    samples = FRAME_SIZE;
  }

  // build packet: [seq][FRAME_SIZE bytes of mu-law]
  packet[0] = seq++;
  for (int i = 0; i < FRAME_SIZE; ++i) {
    // apply gain: shift right; tweak >>10/>>11 if too loud/soft
    int32_t s = raw[i] >> 14;     // gain control (adjust)
    int16_t s16 = (int16_t) s;    // signed 16-bit
    packet[1 + i] = mu_law_encode(s16);
  }

  esp_err_t res = esp_now_send(receiverAddress, packet, PAYLOAD_SIZE);
  // small pacing so we don't flood (FRAME_SIZE at 16k => ~78 packets/sec; this loop rate is fine)
  // If you want exact timing: delayMicroseconds(FRAME_SIZE * 1000000 / SAMPLE_RATE);
}