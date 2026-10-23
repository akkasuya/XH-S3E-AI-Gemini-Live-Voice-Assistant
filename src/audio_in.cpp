#include "audio_in.h"
#include "config.h"
#include <math.h>

static i2s_chan_handle_t rx_chan = NULL;

bool audioInBegin() {
    if (rx_chan != NULL) {
        return true;
    }

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear   = true;
    chan_cfg.dma_desc_num  = 6;
    chan_cfg.dma_frame_num = 256;

    esp_err_t err = i2s_new_channel(&chan_cfg, NULL, &rx_chan);
    if (err != ESP_OK) {
        Serial.printf("[MIC] i2s_new_channel failed: 0x%X\n", err);
        rx_chan = NULL;
        return false;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(MIC_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT,
                                                        I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = (gpio_num_t)I2S_MIC_BCLK_PIN,
            .ws   = (gpio_num_t)I2S_MIC_WS_PIN,
            .dout = I2S_GPIO_UNUSED,
            .din  = (gpio_num_t)I2S_MIC_DATA_PIN,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    // Only the left channel (INMP441 L/R tied low).
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

    err = i2s_channel_init_std_mode(rx_chan, &std_cfg);
    if (err != ESP_OK) {
        Serial.printf("[MIC] i2s_channel_init_std_mode failed: 0x%X\n", err);
        i2s_del_channel(rx_chan);
        rx_chan = NULL;
        return false;
    }

    err = i2s_channel_enable(rx_chan);
    if (err != ESP_OK) {
        Serial.printf("[MIC] i2s_channel_enable failed: 0x%X\n", err);
        i2s_del_channel(rx_chan);
        rx_chan = NULL;
        return false;
    }

    delay(50);

    // Sanity check
    int32_t testBuf[64] = {0};
    size_t testBytes = 0;
    err = i2s_channel_read(rx_chan, testBuf, sizeof(testBuf), &testBytes, pdMS_TO_TICKS(500));
    if (err != ESP_OK || testBytes == 0) {
        Serial.printf("[MIC] post-init read failed (err=0x%X, bytes=%u). Check GPIO %d/%d/%d.\n",
                      err, (unsigned)testBytes, I2S_MIC_BCLK_PIN, I2S_MIC_WS_PIN, I2S_MIC_DATA_PIN);
        i2s_channel_disable(rx_chan);
        i2s_del_channel(rx_chan);
        rx_chan = NULL;
        return false;
    }

    Serial.printf("[MIC] I2S0 STD RX live @ %d Hz (%u bytes in test read)\n",
                  MIC_SAMPLE_RATE, (unsigned)testBytes);
    return true;
}

size_t audioInRead(int16_t* dst, size_t maxSamples, uint32_t timeoutMs) {
    if (dst == nullptr || maxSamples == 0 || rx_chan == NULL) {
        return 0;
    }

    // Read in chunks to keep the stack small.
    size_t total = 0;
    const size_t chunk = 256;
    while (total < maxSamples) {
        size_t want = maxSamples - total;
        if (want > chunk) want = chunk;

        static int32_t i2sBuf[256];
        size_t bytesRead = 0;
        esp_err_t err = i2s_channel_read(rx_chan, i2sBuf, want * sizeof(int32_t),
                                          &bytesRead, pdMS_TO_TICKS(timeoutMs));
        if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
            if (total == 0) {
                Serial.printf("[MIC] read failed: 0x%X\n", err);
            }
            break;
        }
        size_t got = bytesRead / sizeof(int32_t);
        if (got == 0) {
            break;
        }
        for (size_t i = 0; i < got; i++) {
            int32_t v = i2sBuf[i] >> 12;
            if (v > 32767) v = 32767;
            else if (v < -32768) v = -32768;
            dst[total + i] = (int16_t)v;
        }
        total += got;
        if (got < want) {
            break;
        }
    }
    return total;
}

void audioInDrain() {
    if (rx_chan == NULL) {
        return;
    }

    int32_t scratch[256];
    size_t bytesRead = 0;
    while (i2s_channel_read(rx_chan, scratch, sizeof(scratch), &bytesRead, 0) == ESP_OK
           && bytesRead > 0) {
        bytesRead = 0;
    }
}

void audioApplyGain(int16_t* buf, size_t sampleCount, float gain) {
    if (buf == nullptr || gain == 1.0f) {
        return;
    }

    for (size_t i = 0; i < sampleCount; i++) {
        int32_t v = (int32_t)((float)buf[i] * gain);
        if (v > 32767)       v = 32767;
        else if (v < -32768) v = -32768;
        buf[i] = (int16_t)v;
    }
}

float audioRms(const int16_t* buf, size_t sampleCount) {
    if (buf == nullptr || sampleCount == 0) {
        return 0.0f;
    }

    double acc = 0.0;
    for (size_t i = 0; i < sampleCount; i++) {
        double s = (double)buf[i];
        acc += s * s;
    }
    return (float)sqrt(acc / (double)sampleCount);
}

void audioInEnd() {
    if (rx_chan != NULL) {
        i2s_channel_disable(rx_chan);
        i2s_del_channel(rx_chan);
        rx_chan = NULL;
    }
}