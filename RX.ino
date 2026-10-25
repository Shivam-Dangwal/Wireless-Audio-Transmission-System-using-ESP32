/* Receiver - ESP-NOW -> MAX98357A (µ-law decode, jitter conceal) */
#include <WiFi.h>
#include <esp_now.h>
#include <driver/i2s.h>

#define SAMPLE_RATE 16000
#define FRAME_SIZE 200
#define PAYLOAD_SIZE (1 + FRAME_SIZE)
#define STEREO_FRAME (FRAME_SIZE * 2)

i2s_config_t i2s_config = {
  .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
  .sample_rate = SAMPLE_RATE,
  .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
  .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
  .communication_format = I2S_COMM_FORMAT_I2S,
  .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
  .dma_buf_count = 4,
  .dma_buf_len = STEREO_FRAME,
  .use_apll = false,
  .tx_desc_auto_clear = true,
  .fixed_mclk = 0
};

i2s_pin_config_t pin_config = {
  .bck_io_num = 26,
  .ws_io_num  = 25,
  .data_out_num = 22,
  .data_in_num  = -1
};

static int16_t lastStereo[STEREO_FRAME];
static uint8_t lastSeq = 0xFF;
static bool haveLast = false;

static inline int16_t mu_law_decode(uint8_t mu) {
  mu = ~mu;
  int sign = (mu & 0x80) ? -1 : 1;
  int exponent = (mu >> 4) & 0x07;
  int mantissa = mu & 0x0F;
  int magnitude = ((mantissa << 3) + 0x84) << exponent;
  int16_t sample = (int16_t)(sign * (magnitude - 0x84));
  return sample;
}

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != PAYLOAD_SIZE) {
    // unexpected size -> ignore
    return;
  }
  uint8_t seq = data[0];
  const uint8_t *payload = data + 1;

  // If packet lost (non-consecutive), use last packet to conceal
  bool lost = false;
  if (haveLast && ((uint8_t)(lastSeq + 1) != seq)) lost = true;

  int16_t stereoBuf[STEREO_FRAME];

  if (lost) {
    // repeat last buffer (simple concealment)
    memcpy(stereoBuf, lastStereo, sizeof(stereoBuf));
  } else {
    // decode mu-law and duplicate to stereo
    for (int i = 0; i < FRAME_SIZE; ++i) {
      int16_t s = mu_law_decode(payload[i]);
      stereoBuf[i*2]   = s; // left
      stereoBuf[i*2+1] = s; // right
    }
    memcpy(lastStereo, stereoBuf, sizeof(stereoBuf));
    haveLast = true;
    lastSeq = seq;
  }

  // write exactly STEREO_FRAME samples (each sample is int16_t)
  size_t written;
  i2s_write(I2S_NUM_0, stereoBuf, sizeof(stereoBuf), &written, portMAX_DELAY);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init fail");
    while(1);
  }
  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("RX ready");
}

void loop() {
  // nothing here; callback driven
}