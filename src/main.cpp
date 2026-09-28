#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <Update.h>
#include <driver/i2s.h>
#include <esp_heap_caps.h>
#include "wfas.h"

static constexpr char WIFI_SSID[] = "YOUR_WIFI_SSID";
static constexpr char WIFI_PASSWORD[] = "YOUR_WIFI_PASSWORD";

static constexpr char HOSTNAME[] = "wifi-receiver-s3";
static constexpr uint16_t STREAM_PORT = 9090;
static constexpr uint16_t DISCOVERY_PORT = 9091;
static constexpr char DISCOVERY_GROUP[] = "239.255.0.1";
static constexpr int SAMPLE_RATE = 48000;
static constexpr int CHANNELS = 2;
static constexpr int BITS = 16;
static constexpr int I2S_BCLK = 4;
static constexpr int I2S_LRCK = 5;
static constexpr int I2S_DOUT = 21;
static constexpr size_t RX_PACKET_MAX = 1500;
static constexpr size_t AUDIO_BUFFER_BYTES = 1024 * 1024;
static constexpr size_t START_BUFFER_BYTES = SAMPLE_RATE * CHANNELS * 2 / 5; // 400 ms

WiFiUDP udp;
WiFiUDP discoveryUdp;
WebServer web(80);

uint8_t* audioBuffer = nullptr;
size_t rbHead = 0, rbTail = 0, rbCount = 0;
portMUX_TYPE rbMux = portMUX_INITIALIZER_UNLOCKED;

IPAddress activeClient;
bool clientConnected = false;
uint32_t expectedSample = 0;
uint16_t expectedSeq = 0;
bool haveExpected = false;
uint32_t packetsRx = 0;
uint32_t packetsDropped = 0;
uint32_t packetsLost = 0;
uint64_t bytesRx = 0;
uint32_t lastAudioMs = 0;
uint32_t lastPingMs = 0;
uint32_t lastDiscoveryMs = 0;
uint32_t streamStartedMs = 0;
uint32_t underruns = 0;
bool playbackStarted = false;

static void rbReset() {
  portENTER_CRITICAL(&rbMux);
  rbHead = rbTail = rbCount = 0;
  portEXIT_CRITICAL(&rbMux);
}

static size_t rbAvailable() {
  portENTER_CRITICAL(&rbMux);
  size_t n = rbCount;
  portEXIT_CRITICAL(&rbMux);
  return n;
}

static size_t rbFree() {
  portENTER_CRITICAL(&rbMux);
  size_t n = AUDIO_BUFFER_BYTES - rbCount;
  portEXIT_CRITICAL(&rbMux);
  return n;
}

static size_t rbWrite(const uint8_t* data, size_t len) {
  portENTER_CRITICAL(&rbMux);
  size_t n = min(len, AUDIO_BUFFER_BYTES - rbCount);
  size_t first = min(n, AUDIO_BUFFER_BYTES - rbHead);
  memcpy(audioBuffer + rbHead, data, first);
  if (n > first) memcpy(audioBuffer, data + first, n - first);
  rbHead = (rbHead + n) % AUDIO_BUFFER_BYTES;
  rbCount += n;
  portEXIT_CRITICAL(&rbMux);
  return n;
}

static size_t rbRead(uint8_t* out, size_t len) {
  portENTER_CRITICAL(&rbMux);
  size_t n = min(len, rbCount);
  size_t first = min(n, AUDIO_BUFFER_BYTES - rbTail);
  memcpy(out, audioBuffer + rbTail, first);
  if (n > first) memcpy(out + first, audioBuffer, n - first);
  rbTail = (rbTail + n) % AUDIO_BUFFER_BYTES;
  rbCount -= n;
  portEXIT_CRITICAL(&rbMux);
  return n;
}

static void sendText(const IPAddress& ip, uint16_t port, const char* msg) {
  udp.beginPacket(ip, port);
  udp.write((const uint8_t*)msg, strlen(msg));
  udp.endPacket();
}

static void sendDiscovery() {
  IPAddress group;
  group.fromString(DISCOVERY_GROUP);
  char msg[256];
  snprintf(msg, sizeof(msg),
    "WIFI_AUDIO_STREAMER_DISCOVERY;%s;UNICAST;%u;protocols=WFAS;sr=%d;ch=%d;bd=%d;auth=OFF;enc=0",
    HOSTNAME, STREAM_PORT, SAMPLE_RATE, CHANNELS, BITS);
  discoveryUdp.beginPacket(group, DISCOVERY_PORT);
  discoveryUdp.write((const uint8_t*)msg, strlen(msg));
  discoveryUdp.endPacket();
  lastDiscoveryMs = millis();
}

static void configureI2S() {
  i2s_config_t cfg{};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  cfg.sample_rate = SAMPLE_RATE;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_I2S_MSB;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = 8;
  cfg.dma_buf_len = 256;
  cfg.use_apll = true;
  cfg.tx_desc_auto_clear = true;
  cfg.fixed_mclk = 0;

  i2s_driver_install(I2S_NUM_0, &cfg, 0, nullptr);

  i2s_pin_config_t pins{};
  pins.bck_io_num = I2S_BCLK;
  pins.ws_io_num = I2S_LRCK;
  pins.data_out_num = I2S_DOUT;
  pins.data_in_num = I2S_PIN_NO_CHANGE;
  i2s_set_pin(I2S_NUM_0, &pins);
  i2s_set_clk(I2S_NUM_0, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
  i2s_zero_dma_buffer(I2S_NUM_0);
}

static void audioTask(void*) {
  static uint8_t block[4096];
  static uint8_t silence[4096] = {};
  bool primed = false;

  for (;;) {
    size_t available = rbAvailable();

    if (!primed) {
      if (available >= START_BUFFER_BYTES) {
        primed = true;
        playbackStarted = true;
        streamStartedMs = millis();
      } else {
        vTaskDelay(pdMS_TO_TICKS(5));
        continue;
      }
    }

    size_t n = rbRead(block, sizeof(block));
    if (n == 0) {
      underruns++;
      i2s_write(I2S_NUM_0, silence, sizeof(silence), nullptr, portMAX_DELAY);
      primed = false;
      playbackStarted = false;
      continue;
    }

    size_t written = 0;
    i2s_write(I2S_NUM_0, block, n, &written, portMAX_DELAY);

    if (rbAvailable() < 512) {
      primed = false;
      playbackStarted = false;
    }
  }
}

static bool sameClient(const IPAddress& ip) {
  return clientConnected && ip == activeClient;
}

static void acceptClient(const IPAddress& ip) {
  activeClient = ip;
  clientConnected = true;
  haveExpected = false;
  rbReset();
  playbackStarted = false;
  lastPingMs = millis();
  Serial.printf("[WFAS] Client accepted: %s\n", ip.toString().c_str());
}

static void disconnectClient(const char* reason) {
  Serial.printf("[WFAS] Client disconnected: %s\n", reason);
  clientConnected = false;
  haveExpected = false;
  playbackStarted = false;
  rbReset();
}

static void handleControl(const IPAddress& from, uint16_t fromPort, char* msg) {
  wfas_packet_type type = wfas_classify((const uint8_t*)msg, strlen(msg));

  if (type == WFAS_PKT_MODE_PROBE) {
    sendText(from, fromPort, "WFAS_UNICAST");
    return;
  }

  if (type == WFAS_PKT_HELLO) {
    int version = wfas_parse_version(msg);
    if (version > 0 && version != WFAS_PROTOCOL_VERSION) {
      char reply[64];
      snprintf(reply, sizeof(reply), "WFAS_INCOMPATIBLE;v=%d", WFAS_PROTOCOL_VERSION);
      sendText(from, fromPort, reply);
      return;
    }

    if (clientConnected && !sameClient(from)) {
      sendText(from, fromPort, WFAS_MSG_BUSY);
      return;
    }

    char reply[64];
    snprintf(reply, sizeof(reply), "%s;v=%d", WFAS_MSG_HELLO_ACK, WFAS_PROTOCOL_VERSION);
    sendText(from, fromPort, reply);
    if (!clientConnected) acceptClient(from);
    return;
  }

  if (type == WFAS_PKT_CLIENT_BYE) {
    if (sameClient(from)) disconnectClient("CLIENT_BYE");
    return;
  }

  if (type == WFAS_PKT_PING) {
    if (sameClient(from)) {
      lastPingMs = millis();
    }
  }
}

static void handleAudioPacket(const IPAddress& from, const uint8_t* data, size_t len) {
  if (!clientConnected || !sameClient(from)) {
    packetsDropped++;
    return;
  }

  wfas_header hdr{};
  const uint8_t* pcm = nullptr;
  size_t pcmLen = 0;

  if (wfas_parse_audio(data, len, &hdr, &pcm, &pcmLen) != 0) {
    packetsDropped++;
    return;
  }

  if (hdr.version != WFAS_PROTOCOL_VERSION || (hdr.flags & WFAS_FLAG_ENCRYPTED)) {
    packetsDropped++;
    return;
  }

  if (pcmLen == 0) {
    lastAudioMs = millis();
    return;
  }

  const size_t frameBytes = CHANNELS * 2;
  if ((pcmLen % frameBytes) != 0) {
    packetsDropped++;
    return;
  }

  const uint32_t frames = pcmLen / frameBytes;

  if (!haveExpected) {
    expectedSeq = hdr.seq;
    expectedSample = hdr.sample_pos;
    haveExpected = true;
  }

  const uint16_t seqDelta = (uint16_t)(hdr.seq - expectedSeq);
  const uint32_t sampleDelta = hdr.sample_pos - expectedSample;

  if (seqDelta != 0 || sampleDelta != 0) {
    if (seqDelta < 0x8000u && sampleDelta < 0x80000000u) {
      // Forward gap: insert silence for the missing samples, bounded by buffer space.
      size_t gapBytes = (size_t)sampleDelta * frameBytes;
      if (gapBytes > AUDIO_BUFFER_BYTES / 2) {
        rbReset();
        gapBytes = 0;
      }
      static uint8_t zeros[1024] = {};
      while (gapBytes && rbFree() >= 1024) {
        size_t n = min(gapBytes, sizeof(zeros));
        rbWrite(zeros, n);
        gapBytes -= n;
      }
      packetsLost += seqDelta;
    } else {
      // Late/reordered packet.
      packetsDropped++;
      return;
    }
  }

  if (rbFree() < pcmLen) {
    // Preserve newest audio instead of allowing an old backlog to grow forever.
    rbReset();
    playbackStarted = false;
  }

  if (rbWrite(pcm, pcmLen) != pcmLen) {
    packetsDropped++;
    return;
  }

  expectedSeq = (uint16_t)(hdr.seq + 1);
  expectedSample = hdr.sample_pos + frames;
  packetsRx++;
  bytesRx += pcmLen;
  lastAudioMs = millis();
}

static void serviceUdp() {
  int packetSize = udp.parsePacket();
  while (packetSize > 0) {
    uint8_t packet[RX_PACKET_MAX];
    size_t n = udp.read(packet, min(packetSize, (int)sizeof(packet)));
    IPAddress from = udp.remoteIP();
    uint16_t fromPort = udp.remotePort();

    wfas_packet_type type = wfas_classify(packet, n);
    if (type == WFAS_PKT_AUDIO) {
      handleAudioPacket(from, packet, packet, n); // corrected below
    } else {
      char text[512];
      size_t m = min(n, sizeof(text) - 1);
      memcpy(text, packet, m);
      text[m] = 0;
      handleControl(from, fromPort, text);
    }

    packetSize = udp.parsePacket();
  }
}

static String htmlPage() {
  String s;
  s.reserve(5000);
  s += F("<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
         "<title>WiFi Audio Receiver S3</title><style>"
         "body{font-family:system-ui;background:#070b12;color:#eee;max-width:700px;margin:40px auto;padding:20px}"
         ".card{background:#111827;border:1px solid #263244;border-radius:18px;padding:20px;margin:14px 0}"
         "h1{font-size:24px}.ok{color:#34d399}.muted{color:#9ca3af}code{color:#fbbf24}"
         "a,button{background:#f59e0b;color:#111827;border:0;border-radius:10px;padding:10px 14px;font-weight:700}"
         "input{width:100%;padding:10px;background:#0b1220;color:white;border:1px solid #334155;border-radius:10px;box-sizing:border-box}"
         "</style></head><body>");
  s += F("<h1>WiFi Audio Receiver S3</h1><div class='card'>");
  s += "<b>Status:</b> <span class='ok'>";
  s += clientConnected ? "STREAMING / CONNECTED" : "WAITING FOR ANDROID";
  s += F("</span><br><br><b>IP:</b> ");
  s += WiFi.localIP().toString();
  s += F("<br><b>Hostname:</b> ");
  s += HOSTNAME;
  s += F(".local<br><b>WFAS:</b> v2 / UDP 9090<br><b>Audio:</b> 48 kHz / stereo / 16-bit<br><b>I2S:</b> GPIO ");
  s += I2S_BCLK; s += "/"; s += I2S_LRCK; s += "/"; s += I2S_DOUT;
  s += F("</div><div class='card'>Packets: ");
  s += packetsRx; s += F("<br>Bytes: "); s += bytesRx;
  s += F("<br>Lost packets: "); s += packetsLost;
  s += F("<br>Dropped: "); s += packetsDropped;
  s += F("<br>Underruns: "); s += underruns;
  s += F("<br>Buffer: "); s += rbAvailable();
  s += F(" bytes</div><div class='card'><form method='POST' action='/update' enctype='multipart/form-data'>"
         "<input type='file' name='firmware' accept='.bin'><br><br><button>Upload OTA Firmware</button></form>"
         "<p class='muted'>Only upload a valid ESP32-S3 firmware image.</p></div></body></html>");
  return s;
}

static void handleRoot() { web.send(200, "text/html", htmlPage()); }

static void handleUpdateUpload() {
  HTTPUpload& upload = web.upload();
  if (upload.status == UPLOAD_FILE_START) {
    Serial.printf("[OTA] %s\n", upload.filename.c_str());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) Update.printError(Serial);
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) Serial.printf("[OTA] Success: %u bytes\n", upload.totalSize);
    else Update.printError(Serial);
  }
}

static void handleUpdateDone() {
  web.sendHeader("Connection", "close");
  web.send(200, "text/plain", Update.hasError() ? "OTA FAILED" : "OTA OK - rebooting");
  delay(500);
  if (!Update.hasError()) ESP.restart();
}

static void startWeb() {
  web.on("/", HTTP_GET, handleRoot);
  web.on("/update", HTTP_POST, handleUpdateDone, handleUpdateUpload);
  web.begin();
}

static void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setHostname(HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] FAILED. Check WIFI_SSID/WIFI_PASSWORD in main.cpp.");
    return;
  }
  Serial.printf("[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
  if (MDNS.begin(HOSTNAME)) {
    MDNS.addService("http", "tcp", 80);
    MDNS.addService("wfas", "udp", STREAM_PORT);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== WiFi Audio Receiver S3 / N16R8 ===");

  audioBuffer = (uint8_t*)heap_caps_malloc(AUDIO_BUFFER_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!audioBuffer) {
    Serial.println("[FATAL] Could not allocate 1 MB PSRAM audio buffer.");
    while (true) delay(1000);
  }
  memset(audioBuffer, 0, AUDIO_BUFFER_BYTES);

  configureI2S();
  xTaskCreatePinnedToCore(audioTask, "audio", 8192, nullptr, 4, nullptr, 1);

  connectWiFi();
  if (WiFi.status() != WL_CONNECTED) return;

  udp.begin(STREAM_PORT);
  discoveryUdp.begin(0);
  startWeb();
  sendDiscovery();

  Serial.printf("[READY] WFAS v2 UDP %u / discovery %u\n", STREAM_PORT, DISCOVERY_PORT);
  Serial.printf("[READY] UDA1334A I2S: BCLK=%d LRCK=%d DIN=%d\n", I2S_BCLK, I2S_LRCK, I2S_DOUT);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    static uint32_t lastRetry = 0;
    if (millis() - lastRetry > 5000) {
      lastRetry = millis();
      WiFi.reconnect();
    }
    delay(10);
    return;
  }

  if (millis() - lastDiscoveryMs >= 3000) sendDiscovery();
  serviceUdp();
  web.handleClient();

  if (clientConnected && millis() - lastPingMs > 3500 && millis() - lastAudioMs > 3500) {
    disconnectClient("PING/audio timeout");
  }

  delay(1);
}
