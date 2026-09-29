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
volatile bool uiPaused = false;
volatile bool uiMuted = false;
volatile uint8_t uiVolume = 100;

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

  // WFAS v2 discovery beacon: multicast to 239.255.0.1:9091.
  if (discoveryUdp.beginPacket(group, DISCOVERY_PORT)) {
    discoveryUdp.write((const uint8_t*)msg, strlen(msg));
    discoveryUdp.endPacket();
  }

  // Send a second copy immediately, matching the Android/desktop discovery
  // behaviour so one lost multicast frame does not hide the receiver.
  delay(20);
  if (discoveryUdp.beginPacket(group, DISCOVERY_PORT)) {
    discoveryUdp.write((const uint8_t*)msg, strlen(msg));
    discoveryUdp.endPacket();
  }

  // Fallback for access points that filter IPv4 multicast: also announce on
  // the local subnet broadcast address. Android listens on UDP 9091 as well.
  IPAddress ip = WiFi.localIP();
  IPAddress mask = WiFi.subnetMask();
  IPAddress broadcast(
    (uint8_t)(ip[0] | (uint8_t)~mask[0]),
    (uint8_t)(ip[1] | (uint8_t)~mask[1]),
    (uint8_t)(ip[2] | (uint8_t)~mask[2]),
    (uint8_t)(ip[3] | (uint8_t)~mask[3])
  );
  delay(20);
  if (discoveryUdp.beginPacket(broadcast, DISCOVERY_PORT)) {
    discoveryUdp.write((const uint8_t*)msg, strlen(msg));
    discoveryUdp.endPacket();
  }

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

    if (uiPaused || uiMuted || uiVolume == 0) {
      memset(block, 0, n);
    } else if (uiVolume < 100) {
      int16_t* samples = reinterpret_cast<int16_t*>(block);
      size_t sampleCount = n / sizeof(int16_t);
      for (size_t i = 0; i < sampleCount; ++i) {
        samples[i] = (int16_t)(((int32_t)samples[i] * uiVolume) / 100);
      }
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
      handleAudioPacket(from, packet, n);
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

static void handleApiStatus() {
  String json = "{";
  json += "\"connected\":" + String(clientConnected ? "true" : "false");
  json += ",\"paused\":" + String(uiPaused ? "true" : "false");
  json += ",\"muted\":" + String(uiMuted ? "true" : "false");
  json += ",\"volume\":" + String(uiVolume);
  json += ",\"ip\":\"" + WiFi.localIP().toString() + "\"";
  json += ",\"sender\":\"" + String(clientConnected ? activeClient.toString() : "Waiting") + "\"";
  json += ",\"packets\":" + String(packetsRx);
  json += ",\"lost\":" + String(packetsLost);
  json += ",\"underruns\":" + String(underruns);
  json += ",\"buffer\":" + String(rbAvailable());
  json += "}";
  web.send(200, "application/json", json);
}

static void handleApiControl() {
  if (web.hasArg("pause")) uiPaused = web.arg("pause") == "1";
  if (web.hasArg("mute")) uiMuted = web.arg("mute") == "1";
  if (web.hasArg("volume")) {
    int v = constrain(web.arg("volume").toInt(), 0, 100);
    uiVolume = (uint8_t)v;
    if (v > 0) uiMuted = false;
  }
  web.send(200, "application/json", "{\"ok\":true}");
}

static String htmlPage() {
  String s;
  s.reserve(12000);
  s += F(R"rawliteral(
<!doctype html><html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>WiFi Receiver S3</title>
<style>
:root{color-scheme:dark;--bg:#050609;--card:#111318;--line:#262a33;--text:#f5f5f5;--muted:#9297a3;--accent:#d7ff45}
*{box-sizing:border-box}body{margin:0;background:radial-gradient(circle at 50% -10%,#20251b 0,#08090b 38%,#050609 75%);color:var(--text);font-family:system-ui,-apple-system,sans-serif}
main{max-width:480px;margin:auto;padding:22px 16px 36px}.top{display:flex;justify-content:space-between;align-items:center;margin-bottom:18px}
.brand{font-weight:800;font-size:18px}.live{font-size:11px;border:1px solid #394029;border-radius:20px;padding:6px 10px;color:var(--accent)}
.art{height:360px;border-radius:28px;overflow:hidden;position:relative;background:linear-gradient(145deg,#1c2415,#080a08 58%,#171a12);border:1px solid #30352a;box-shadow:0 20px 60px #0008}
.art:before,.art:after{content:"";position:absolute;border-radius:50%;filter:blur(1px)}
.art:before{width:250px;height:250px;background:radial-gradient(circle,#d7ff45 0,transparent 66%);opacity:.17;top:-55px;right:-55px}
.art:after{width:220px;height:220px;background:radial-gradient(circle,#fff 0,transparent 65%);opacity:.06;bottom:-90px;left:-70px}
.cover{position:absolute;inset:0;display:flex;flex-direction:column;justify-content:flex-end;padding:25px}
.wave{display:flex;gap:5px;align-items:center;height:70px;margin-bottom:18px}.bar{width:6px;border-radius:10px;background:var(--accent);opacity:.8;animation:b 1.1s ease-in-out infinite alternate}
.bar:nth-child(1){height:25%;animation-delay:.1s}.bar:nth-child(2){height:55%;animation-delay:.3s}.bar:nth-child(3){height:80%;animation-delay:.2s}.bar:nth-child(4){height:42%;animation-delay:.4s}.bar:nth-child(5){height:95%;animation-delay:.15s}.bar:nth-child(6){height:65%;animation-delay:.35s}.bar:nth-child(7){height:35%;animation-delay:.25s}.bar:nth-child(8){height:72%;animation-delay:.45s}.bar:nth-child(9){height:48%;animation-delay:.05s}.bar:nth-child(10){height:88%;animation-delay:.3s}
@keyframes b{to{transform:scaleY:.35}}
.kicker{font-size:11px;letter-spacing:2px;color:var(--accent);font-weight:800}.title{font-size:28px;font-weight:850;margin-top:6px}.sub{color:var(--muted);margin-top:3px}
.card{background:#0e1014cc;border:1px solid var(--line);border-radius:20px;padding:16px;margin-top:14px}
.controls{display:flex;align-items:center;justify-content:center;gap:14px}.btn{border:0;border-radius:50%;width:48px;height:48px;background:#20232a;color:white;font-size:18px}.play{width:64px;height:64px;background:var(--accent);color:#080900;font-size:25px}
.row{display:flex;justify-content:space-between;align-items:center;font-size:13px}.label{color:var(--muted)}input[type=range]{width:100%;accent-color:var(--accent);margin-top:13px}
.status{display:grid;grid-template-columns:1fr 1fr;gap:10px}.stat{background:#15171c;border-radius:14px;padding:12px}.stat b{display:block;font-size:15px}.stat span{font-size:11px;color:var(--muted)}
a{color:var(--accent);text-decoration:none}.ota{display:flex;gap:10px}.ota a{flex:1;text-align:center;background:#191c21;padding:11px;border-radius:12px}
.small{text-align:center;color:var(--muted);font-size:11px;margin-top:18px}
</style></head><body><main>
<div class="top"><div class="brand">WiFi Receiver S3</div><div class="live" id="live">WAITING</div></div>
<div class="art"><div class="cover"><div class="wave">)rawliteral");
  for (int i=0;i<10;i++) s += "<i class='bar'></i>";
  s += F(R"rawliteral(</div><div class="kicker">WIRELESS AUDIO</div><div class="title" id="track">Android Audio</div><div class="sub" id="sender">Waiting for Android sender</div></div></div>
<div class="card"><div class="controls">
<button class="btn" onclick="send({mute:1})">🔇</button>
<button class="btn" onclick="send({pause:1})">Ⅱ</button>
<button class="btn play" id="play" onclick="toggle()">▶</button>
<button class="btn" onclick="send({mute:0})">🔊</button>
</div></div>
<div class="card"><div class="row"><span class="label">Output volume</span><b id="vol">100%</b></div><input id="slider" type="range" min="0" max="100" value="100" oninput="volume(this.value)"></div>
<div class="card status">
<div class="stat"><b id="conn">Offline</b><span>Connection</span></div>
<div class="stat"><b>48 kHz</b><span>16-bit stereo</span></div>
<div class="stat"><b id="buf">0 KB</b><span>Audio buffer</span></div>
<div class="stat"><b id="drop">0</b><span>Lost packets</span></div>
</div>
<div class="card ota"><a href="/update">OTA endpoint</a><a href="/">Refresh</a></div>
<div class="small">)rawliteral");
  s += HOSTNAME;
  s += F(R"rawliteral(.local · UDA1334A · WFAS v2</div>
<script>
let paused=false;
async function send(o){await fetch('/api/control?'+new URLSearchParams(o));refresh()}
function volume(v){document.getElementById('vol').textContent=v+'%';fetch('/api/control?volume='+v)}
function toggle(){paused=!paused;send({pause:paused?1:0})}
async function refresh(){try{let x=await (await fetch('/api/status')).json();
document.getElementById('live').textContent=x.connected?'LIVE':'WAITING';
document.getElementById('conn').textContent=x.connected?'Streaming':'Offline';
document.getElementById('sender').textContent=x.connected?'Android · '+x.sender:'Waiting for Android sender';
document.getElementById('buf').textContent=Math.round(x.buffer/1024)+' KB';
document.getElementById('drop').textContent=x.lost;
document.getElementById('slider').value=x.volume;document.getElementById('vol').textContent=x.volume+'%';
paused=x.paused;document.getElementById('play').textContent=paused?'▶':'Ⅱ';
}catch(e){}}setInterval(refresh,1000);refresh();
</script></main></body></html>)rawliteral");
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
  web.on("/api/status", HTTP_GET, handleApiStatus);
  web.on("/api/control", HTTP_GET, handleApiControl);
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

  // Bind discovery to the connected Wi-Fi interface.
  if (!discoveryUdp.begin(WiFi.localIP(), 0)) {
    Serial.println("[WFAS] Discovery UDP bind failed.");
  }
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
