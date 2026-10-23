#include "radio.h"
#include "app_state.h"
#include "config.h"
#include "oled_display.h"

#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <esp_heap_caps.h>
#include <atomic>
#include <string.h>
#include <cstdarg>
#include <cstdlib>

#include "MP3DecoderHelix.h"

using namespace libhelix;

// -----------------------------------------------------------------------------
// Station table
// -----------------------------------------------------------------------------
struct Station { const char* name; const char* url; };
static const Station kStations[] = {
    { "coolfh",          "https://coolism-web.cdn.byteark.com/;stream/1" },
    { "greenwave",       "https://atimeonline3.smartclick.co.th/green" },
    { "efm",             "https://atimeonline3.smartclick.co.th/efm" },
    { "looktungnetwork", "https://cdn-th2.livestreaming.in.th/shoutcast/8200" },
};
static const size_t kStationCount = sizeof(kStations) / sizeof(kStations[0]);

// -----------------------------------------------------------------------------
// State
// -----------------------------------------------------------------------------
typedef struct { char station[32]; } RadioReq;

static QueueHandle_t sReqQ = nullptr;
static TaskHandle_t  sTask = nullptr;

static std::atomic<bool> sActive(false);
static std::atomic<bool> sPlaying(false);
static std::atomic<bool> sStop(false);

static char              sNotice[160];
static std::atomic<bool> sNoticeReady(false);

static void postNotice(const char* fmt, ...) {
    if (sNoticeReady.load()) return;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(sNotice, sizeof(sNotice), fmt, ap);
    va_end(ap);
    sNoticeReady.store(true);
}

bool radioIsActive()  { return sActive.load(); }
bool radioIsPlaying() { return sPlaying.load(); }

void radioStop() {
    if (sActive.load()) sStop.store(true);
}

bool radioTakeNotice(char* dst, size_t cap) {
    if (!sNoticeReady.load() || dst == nullptr || cap == 0) return false;
    snprintf(dst, cap, "%s", sNotice);
    sNoticeReady.store(false);
    return true;
}

// -----------------------------------------------------------------------------
// URL helpers
// -----------------------------------------------------------------------------
static bool parseUrl(const char* url, char* host, size_t hostCap,
                     uint16_t* port, char* path, size_t pathCap, bool* tls) {
    // very small parser: scheme://host[:port]/path
    const char* p = url;
    if (strncmp(p, "https://", 8) == 0) { *tls = true; p += 8; *port = 443; }
    else if (strncmp(p, "http://", 7) == 0) { *tls = false; p += 7; *port = 80; }
    else { *tls = false; *port = 80; }

    const char* slash = strchr(p, '/');
    const char* colon = strchr(p, ':');
    size_t hostLen;
    if (colon && (!slash || colon < slash)) {
        hostLen = colon - p;
        *port = (uint16_t)atoi(colon + 1);
    } else {
        hostLen = slash ? (size_t)(slash - p) : strlen(p);
    }
    if (hostLen == 0 || hostLen >= hostCap) return false;
    memcpy(host, p, hostLen);
    host[hostLen] = '\0';

    if (slash) {
        snprintf(path, pathCap, "%s", slash);
    } else {
        snprintf(path, pathCap, "/");
    }
    return true;
}

static bool readLine(WiFiClient& c, char* out, size_t cap, uint32_t deadline) {
    size_t n = 0;
    out[0] = '\0';
    for (;;) {
        if (millis() > deadline) return false;
        if (!c.available()) {
            if (!c.connected()) return false;
            delay(2);
            continue;
        }
        int ch = c.read();
        if (ch < 0) continue;
        if (ch == '\n') {
            if (n > 0 && out[n-1] == '\r') n--;
            out[n] = '\0';
            return true;
        }
        if (n + 1 < cap) out[n++] = (char)ch;
        out[n] = '\0';
    }
}

// -----------------------------------------------------------------------------
// Resampling state (src -> 24 kHz mono)
// -----------------------------------------------------------------------------
static double   sNextOut = 0.0;   // next output position in input samples
static uint64_t sInIdx  = 0;
static int16_t  sPrev   = 0;
static bool     sHavePrev = false;
static int16_t  sOutBuf[512];
static size_t   sOutN   = 0;

static void pushOut() {
    if (sOutN == 0) return;
    size_t sent = xStreamBufferSend(gPcmOut, sOutBuf, sOutN * sizeof(int16_t), pdMS_TO_TICKS(2000));
    (void)sent;
    sOutN = 0;
}

static void emitSample(int16_t v) {
    if (sOutN >= 512) pushOut();
    sOutBuf[sOutN++] = v;
}

static void resetResampler() {
    sNextOut = 0.0;
    sInIdx = 0;
    sPrev = 0;
    sHavePrev = false;
    sOutN = 0;
}

// We need srcRate; store global
static int sSrcRate = 44100;

static void handleMonoSample2(int16_t cur) {
    if (!sHavePrev) {
        sPrev = cur;
        sHavePrev = true;
        sInIdx = 1;
        emitSample(cur);
        double ratio = (double)sSrcRate / 24000.0;
        sNextOut = ratio;
        return;
    }
    double ratio = (double)sSrcRate / 24000.0;
    uint64_t idx = sInIdx;
    while (sNextOut <= (double)idx) {
        double t = (sNextOut - (double)(idx - 1));
        if (t < 0) t = 0;
        if (t > 1) t = 1;
        int32_t v = sPrev + (int32_t)((cur - sPrev) * t);
        if (v > 32767) v = 32767;
        if (v < -32768) v = -32768;
        emitSample((int16_t)v);
        sNextOut += ratio;
    }
    sPrev = cur;
    sInIdx++;
}

static uint32_t sPcmFrames = 0;
static void pcmCallback(MP3FrameInfo &info, short *pcm, size_t len, void *ref) {
    if (len == 0) return;
    sPcmFrames++;
    if (sPcmFrames % 50 == 0) {
        Serial.printf("[RADIO] decoded frames=%u rate=%d ch=%d\n", (unsigned)sPcmFrames, info.samprate, info.nChans);
    }
    sSrcRate = info.samprate > 0 ? info.samprate : 44100;
    int ch = info.nChans > 0 ? info.nChans : 1;
    size_t frames = len / ch;
    sPlaying.store(true);
    for (size_t i = 0; i < frames; i++) {
        int32_t mono = 0;
        for (int c = 0; c < ch; c++) mono += pcm[i * ch + c];
        mono /= ch;
        handleMonoSample2((int16_t)mono);
    }
}

// -----------------------------------------------------------------------------
// Streaming
// -----------------------------------------------------------------------------
static void streamStation(const char* url) {
    char host[128];
    uint16_t port = 443;
    char path[256];
    bool tls = true;
    if (!parseUrl(url, host, sizeof(host), &port, path, sizeof(path), &tls)) {
        postNotice("Invalid radio URL.");
        return;
    }

    WiFiClientSecure client;
    if (tls) client.setInsecure();
    WiFiClient plain;
    // choose
    // We'll use secure for all to simplify? Use WiFiClient* via pointer.
    // Simpler: if tls use secure else plain, but code duplication. Use secure always setInsecure for https; for http use plain.
    // Implement with a generic pointer is complex; do if/else.
    bool connected = false;
    if (tls) {
        connected = client.connect(host, port);
    } else {
        connected = plain.connect(host, port);
    }
    if (!connected) {
        postNotice("Could not connect to radio station.");
        return;
    }
    Serial.printf("[RADIO] connected to %s:%u\n", host, port);

    WiFiClient* c = tls ? (WiFiClient*)&client : &plain;

    c->printf("GET %s HTTP/1.1\r\nHost: %s\r\nIcy-MetaData: 0\r\nConnection: close\r\n\r\n", path, host);

    // read headers
    uint32_t deadline = millis() + 10000;
    char line[256];
    bool chunked = false;
    for (;;) {
        if (!readLine(*c, line, sizeof(line), deadline)) break;
        if (line[0] == '\0') break;
        Serial.printf("[RADIO] hdr: %s\n", line);
        if (strstr(line, "Transfer-Encoding") && strstr(line, "chunked")) {
            chunked = true;
        }
    }
    Serial.printf("[RADIO] headers done (chunked=%d)\n", (int)chunked);

    MP3DecoderHelix decoder(pcmCallback);
    if (!decoder.begin()) {
        postNotice("Radio decoder init failed.");
        c->stop();
        return;
    }

    resetResampler();
    uint8_t buf[1024];
    size_t chunkRemaining = 0;
    while (!sStop.load()) {
        if (chunked) {
            if (chunkRemaining == 0) {
                // read next chunk size
                char sz[32];
                if (!readLine(*c, sz, sizeof(sz), millis() + 5000)) break;
                // parse hex
                chunkRemaining = (size_t)strtoul(sz, nullptr, 16);
                if (chunkRemaining == 0) break; // last chunk
                continue;
            }
            int avail = c->available();
            if (avail <= 0) {
                if (!c->connected()) break;
                delay(2);
                continue;
            }
            int toRead = avail;
            if (toRead > (int)sizeof(buf)) toRead = sizeof(buf);
            if ((size_t)toRead > chunkRemaining) toRead = (int)chunkRemaining;
            int r = c->read(buf, toRead);
            if (r <= 0) continue;
            decoder.write(buf, r);
            chunkRemaining -= r;
            if (chunkRemaining == 0) {
                // consume trailing CRLF
                char crlf[4];
                readLine(*c, crlf, sizeof(crlf), millis() + 1000);
            }
        } else {
            int avail = c->available();
            if (avail <= 0) {
                if (!c->connected()) break;
                delay(2);
                continue;
            }
            int toRead = avail;
            if (toRead > (int)sizeof(buf)) toRead = sizeof(buf);
            int r = c->read(buf, toRead);
            if (r <= 0) continue;
            decoder.write(buf, r);
        }
    }

    pushOut();
    decoder.end();
    c->stop();
}

// -----------------------------------------------------------------------------
// Task
// -----------------------------------------------------------------------------
static void radioTask(void* arg) {
    (void)arg;
    for (;;) {
        RadioReq req;
        if (xQueueReceive(sReqQ, &req, portMAX_DELAY) != pdTRUE) continue;
        sStop.store(false);
        sActive.store(true);
        sPlaying.store(false);

        // find station
        const char* url = nullptr;
        for (size_t i = 0; i < kStationCount; i++) {
            if (strcasecmp(req.station, kStations[i].name) == 0) {
                url = kStations[i].url;
                break;
            }
        }
        if (url == nullptr) {
            // maybe req.station is a URL
            if (strncmp(req.station, "http", 4) == 0) url = req.station;
        }
        if (url == nullptr) {
            sActive.store(false);
            postNotice("Unknown radio station: %s", req.station);
            continue;
        }

        Serial.printf("[RADIO] playing %s\n", req.station);
        oledSubtitleRio("radio");
        streamStation(url);
        sPlaying.store(false);
        sActive.store(false);
    }
}

bool radioBegin() {
    if (sTask != nullptr) return true;
    sReqQ = xQueueCreate(2, sizeof(RadioReq));
    if (sReqQ == nullptr) return false;
    if (xTaskCreatePinnedToCore(radioTask, "radio", 12288, nullptr, 3, &sTask, 0) != pdPASS) {
        sTask = nullptr;
        return false;
    }
    return true;
}

void radioRequest(const char* station) {
    if (sReqQ == nullptr || station == nullptr || *station == '\0') return;
    // stop current
    radioStop();
    delay(50);
    RadioReq req;
    snprintf(req.station, sizeof(req.station), "%s", station);
    xQueueSend(sReqQ, &req, 0);
}
