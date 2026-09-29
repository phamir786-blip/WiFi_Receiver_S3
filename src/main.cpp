#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <Update.h>
#include <driver/i2s.h>
#include <esp_heap_caps.h>
#include "wfas.h"

// WFAS v2 headless receiver for WiFiAudioStreaming-Android v1.2.
// Boot -> Wi-Fi -> discovery -> unicast HELLO or multicast join -> PCM -> UDA1334A.
// UDA1334A: BCLK GPIO4, LRCK GPIO5, DIN GPIO21.

static constexpr char WIFI_SSID[] = "GFiber_2.4_Coverage_AECD9";
static constexpr char WIFI_PASSWORD[] = "006BF4FD";
static constexpr char HOSTNAME[] = "wifi-receiver-s3";
static constexpr char MCAST[] = "239.255.0.1";
static constexpr uint16_t DISCOVERY_PORT = 9091;
static constexpr uint32_t DEFAULT_SR = 48000;
static constexpr uint8_t DEFAULT_CH = 2;
static constexpr size_t UDP_MAX = 1500;
static constexpr size_t RB_SIZE = 1024 * 1024;
static constexpr size_t PREBUFFER = 48 * 1024;
static constexpr int BCLK = 4, LRCK = 5, DOUT = 21;

WiFiUDP discovery;
WiFiUDP audio;
IPAddress peer;
String peerName;
uint16_t peerPort = 0;
bool multicastSession = false, session = false, i2sReady = false;
uint32_t sr = DEFAULT_SR, lastDiscovery = 0, lastHello = 0, lastAudio = 0;
uint8_t ch = DEFAULT_CH;
uint16_t expectedSeq = 0;
uint32_t expectedPos = 0;
bool haveExpected = false;
uint32_t packets = 0, lost = 0, dropped = 0, underruns = 0;

uint8_t* rb = nullptr;
volatile size_t head = 0, tail = 0, count = 0;
portMUX_TYPE rbMux = portMUX_INITIALIZER_UNLOCKED;
WebServer web(80);
volatile bool uiPaused=false, uiMuted=false;
volatile uint8_t uiVolume=100;

static void rbClear(){portENTER_CRITICAL(&rbMux);head=tail=count=0;portEXIT_CRITICAL(&rbMux);}
static size_t rbAvail(){portENTER_CRITICAL(&rbMux);size_t n=count;portEXIT_CRITICAL(&rbMux);return n;}
static size_t rbFree(){portENTER_CRITICAL(&rbMux);size_t n=RB_SIZE-count;portEXIT_CRITICAL(&rbMux);return n;}
static size_t rbPut(const uint8_t*p,size_t n){portENTER_CRITICAL(&rbMux);size_t c=count;size_t w=min(n,RB_SIZE-c);size_t h=head;size_t a=min(w,RB_SIZE-h);memcpy(rb+h,p,a);if(w>a)memcpy(rb,p+a,w-a);head=(h+w)%RB_SIZE;count=c+w;portEXIT_CRITICAL(&rbMux);return w;}
static size_t rbGet(uint8_t*p,size_t n){portENTER_CRITICAL(&rbMux);size_t c=count;size_t t=tail;size_t r=min(n,c);size_t a=min(r,RB_SIZE-t);memcpy(p,rb+t,a);if(r>a)memcpy(p+a,rb,r-a);tail=(t+r)%RB_SIZE;count=c-r;portEXIT_CRITICAL(&rbMux);return r;}

static bool setupI2S(uint32_t rate,uint8_t channels){
  if(rate<8000||rate>192000||channels<1||channels>2)return false;
  if(i2sReady){i2s_stop(I2S_NUM_0);i2s_driver_uninstall(I2S_NUM_0);i2sReady=false;}
  i2s_config_t c{};
  c.mode=(i2s_mode_t)(I2S_MODE_MASTER|I2S_MODE_TX);
  c.sample_rate=rate;c.bits_per_sample=I2S_BITS_PER_SAMPLE_16BIT;
  c.channel_format=channels==1?I2S_CHANNEL_FMT_ONLY_LEFT:I2S_CHANNEL_FMT_RIGHT_LEFT;
  c.communication_format=I2S_COMM_FORMAT_I2S_MSB;c.intr_alloc_flags=ESP_INTR_FLAG_LEVEL1;
  c.dma_buf_count=8;c.dma_buf_len=256;c.use_apll=true;c.tx_desc_auto_clear=true;
  if(i2s_driver_install(I2S_NUM_0,&c,0,nullptr)!=ESP_OK)return false;
  i2s_pin_config_t p{};p.bck_io_num=BCLK;p.ws_io_num=LRCK;p.data_out_num=DOUT;p.data_in_num=I2S_PIN_NO_CHANGE;
  if(i2s_set_pin(I2S_NUM_0,&p)!=ESP_OK){i2s_driver_uninstall(I2S_NUM_0);return false;}
  if(i2s_set_clk(I2S_NUM_0,rate,I2S_BITS_PER_SAMPLE_16BIT,channels==1?I2S_CHANNEL_MONO:I2S_CHANNEL_STEREO)!=ESP_OK){i2s_driver_uninstall(I2S_NUM_0);return false;}
  i2s_zero_dma_buffer(I2S_NUM_0);i2s_start(I2S_NUM_0);sr=rate;ch=channels;i2sReady=true;
  Serial.printf("[I2S] %lu Hz %u ch 16-bit | GPIO %d/%d/%d\n",(unsigned long)sr,ch,BCLK,LRCK,DOUT);return true;
}

static void audioTask(void*){
  uint8_t block[4096],zero[4096]={};bool primed=false;
  for(;;){
    if(!i2sReady){vTaskDelay(pdMS_TO_TICKS(20));continue;}
    if(uiPaused||uiMuted){i2s_write(I2S_NUM_0,zero,sizeof(zero),nullptr,portMAX_DELAY);vTaskDelay(pdMS_TO_TICKS(2));continue;}
    if(!primed){if(rbAvail()<PREBUFFER){vTaskDelay(pdMS_TO_TICKS(2));continue;}primed=true;}
    size_t n=rbGet(block,sizeof(block));
    if(!n){underruns++;i2s_write(I2S_NUM_0,zero,sizeof(zero),nullptr,portMAX_DELAY);primed=false;continue;}
    uint8_t v=uiVolume;
    if(v<100){int16_t* pcm=(int16_t*)block;size_t samples=n/2;for(size_t i=0;i<samples;i++)pcm[i]=(int16_t)(((int32_t)pcm[i]*v)/100);}
    i2s_write(I2S_NUM_0,block,n,nullptr,portMAX_DELAY);
    if(rbAvail()<4096)primed=false;
  }
}

static String field(const String&s,const char*key){
  String k=String(key)+"=";int p=0;
  while(p<(int)s.length()){int e=s.indexOf(';',p);if(e<0)e=s.length();String t=s.substring(p,e);if(t.startsWith(k))return t.substring(k.length());p=e+1;}
  return "";
}

static void sendHello(){
  audio.beginPacket(peer,peerPort);const char msg[]="HELLO_FROM_CLIENT;v=2";
  audio.write((const uint8_t*)msg,sizeof(msg)-1);audio.endPacket();lastHello=millis();
}

static void stopSession(const char*why){
  Serial.printf("[WFAS] stop: %s\n",why);
  audio.stop();session=false;multicastSession=false;peerPort=0;haveExpected=false;rbClear();
}

static bool startUnicast(const IPAddress&ip,uint16_t port,uint32_t rate,uint8_t channels){
  audio.stop();if(!audio.begin(0))return false;peer=ip;peerPort=port;multicastSession=false;session=true;lastAudio=0;haveExpected=false;rbClear();
  setupI2S(rate,channels);sendHello();
  Serial.printf("[WFAS] unicast -> %s:%u | HELLO sent\n",peer.toString().c_str(),peerPort);return true;
}

static bool startMulticast(uint16_t port,uint32_t rate,uint8_t channels){
  IPAddress group;group.fromString(MCAST);audio.stop();
  if(!audio.beginMulticast(group,port))return false;
  peerPort=port;multicastSession=true;session=true;haveExpected=false;rbClear();setupI2S(rate,channels);
  Serial.printf("[WFAS] multicast joined %s:%u\n",MCAST,port);return true;
}

static void discoveryPacket(const uint8_t*data,size_t len,IPAddress from){
  String msg;msg.reserve(len+1);for(size_t i=0;i<len;i++)msg+=(char)data[i];
  if(!msg.startsWith("WIFI_AUDIO_STREAMER_DISCOVERY;")||from==WiFi.localIP())return;
  int a=msg.indexOf(';'),b=msg.indexOf(';',a+1),c=msg.indexOf(';',b+1);if(a<0||b<0||c<0)return;
  String name=msg.substring(a+1,b),mode=msg.substring(b+1,c);int d=msg.indexOf(';',c+1);
  String portText=d<0?msg.substring(c+1):msg.substring(c+1,d);uint16_t port=portText.toInt();if(!port)return;
  if(mode.equalsIgnoreCase("BYE")){if(peer==from)stopSession("server BYE");return;}
  if(session)return;
  String srt=field(msg,"sr"),cht=field(msg,"ch"),bdt=field(msg,"bd");
  uint32_t rate=srt.length()?srt.toInt():DEFAULT_SR;uint8_t channels=cht.length()?cht.toInt():DEFAULT_CH;uint8_t bits=bdt.length()?bdt.toInt():16;
  if(bits!=16||rate<8000||rate>192000||channels<1||channels>2)return;
  peer=from;peerName=name;lastDiscovery=millis();
  Serial.printf("[DISCOVERY] %s %s:%u %s %lu/%u/%u\n",name.c_str(),from.toString().c_str(),port,mode.c_str(),(unsigned long)rate,channels,bits);
  if(mode.equalsIgnoreCase("MULTICAST"))startMulticast(port,rate,channels);else if(mode.equalsIgnoreCase("UNICAST"))startUnicast(from,port,rate,channels);
}

static void serviceDiscovery(){
  int n=discovery.parsePacket();
  while(n>0){uint8_t b[700];int r=discovery.read(b,min(n,(int)sizeof(b)));if(r>0)discoveryPacket(b,r,discovery.remoteIP());n=discovery.parsePacket();}
}

static void audioPacket(const uint8_t*data,size_t len,IPAddress from){
  if(len<10||data[0]!=0x57||data[1]!=0x46)return;
  if(!multicastSession&&from!=peer)return;
  wfas_header h{};const uint8_t*pcm=nullptr;size_t pcmLen=0;
  if(wfas_parse_audio(data,len,&h,&pcm,&pcmLen)!=0){dropped++;return;}
  if(h.version!=2){stopSession("protocol mismatch");return;}
  if(h.flags&WFAS_FLAG_ENCRYPTED){dropped++;return;}
  size_t frameBytes=ch*2;if(!frameBytes||pcmLen%frameBytes){dropped++;return;}
  session=true;lastAudio=millis();uint32_t frames=pcmLen/frameBytes;
  if(!haveExpected){expectedSeq=h.seq;expectedPos=h.sample_pos;haveExpected=true;}
  uint16_t seqGap=(uint16_t)(h.seq-expectedSeq);uint32_t posGap=h.sample_pos-expectedPos;
  if(seqGap||posGap){
    if(seqGap<0x8000u&&posGap<0x80000000u){
      size_t gap=(size_t)posGap*frameBytes;
      if(gap>RB_SIZE/4||rbFree()<gap)rbClear();else{static uint8_t z[1024]={};while(gap){size_t n=min(gap,sizeof(z));if(rbPut(z,n)!=n)break;gap-=n;}}
      if(seqGap)lost+=seqGap;
    }else{dropped++;return;}
  }
  if(rbFree()<pcmLen)rbClear();
  if(rbPut(pcm,pcmLen)!=pcmLen){dropped++;return;}
  expectedSeq=(uint16_t)(h.seq+1);expectedPos=h.sample_pos+frames;packets++;
}

static void serviceAudio(){
  int n=audio.parsePacket();
  while(n>0){
    uint8_t b[UDP_MAX];int r=audio.read(b,min(n,(int)sizeof(b)));IPAddress from=audio.remoteIP();
    if(r>0){
      if(wfas_classify(b,r)==WFAS_PKT_AUDIO)audioPacket(b,r,from);
      else{char t[256];int m=min(r,(int)sizeof(t)-1);memcpy(t,b,m);t[m]=0;String msg(t);
        if(msg.startsWith("HELLO_ACK")){if(wfas_parse_version(t)!=2)stopSession("HELLO_ACK mismatch");else{session=true;Serial.println("[WFAS] HELLO_ACK v2");}}
        else if(msg.startsWith("WFAS_INCOMPATIBLE"))stopSession("server incompatible");
        else if(msg.startsWith("WFAS_BUSY"))stopSession("server busy");
        else if(msg=="BYE")stopSession("server BYE");
      }
    }
    n=audio.parsePacket();
  }
}


static void handleApiStatus(){String j="{\"connected\":";j+=session?"true":"false";j+=",\"paused\":"+(String)(uiPaused?"true":"false");j+=",\"muted\":"+(String)(uiMuted?"true":"false");j+=",\"volume\":"+String(uiVolume);j+=",\"ip\":\""+WiFi.localIP().toString()+"\"";j+=",\"sender\":\""+(session?peer.toString():"Waiting for Android sender")+"\"";j+=",\"senderName\":\""+(peerName.length()?peerName:"Android streamer")+"\"";j+=",\"sampleRate\":"+String(sr)+",\"channels\":"+String(ch);j+=",\"packets\":"+String(packets)+",\"lost\":"+String(lost)+",\"dropped\":"+String(dropped)+",\"underruns\":"+String(underruns)+",\"buffer\":"+String(rbAvail())+"}";web.send(200,"application/json",j);}
static void handleApiControl(){if(web.hasArg("pause"))uiPaused=web.arg("pause")=="1";if(web.hasArg("mute"))uiMuted=web.arg("mute")=="1";if(web.hasArg("volume")){int v=constrain(web.arg("volume").toInt(),0,100);uiVolume=(uint8_t)v;if(v>0)uiMuted=false;}web.send(200,"application/json","{\"ok\":true}");}
static void handleRoot(){web.send(200,"text/html",String(F("<!doctype html><html><head><meta name=\"viewport\" content=\"width=device-width,initial-scale=1,viewport-fit=cover\"><meta name=\"theme-color\" content=\"#07080b\"><title>WiFi Receiver S3</title>\n<style>:root{color-scheme:dark;--bg:#06070a;--panel:#111318ee;--line:#252a33;--text:#f5f7fb;--muted:#8e95a3;--accent:#d8ff4d}*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}body{margin:0;min-height:100vh;background:radial-gradient(800px 420px at 50% -120px,#20271b,#090b0e 55%,#06070a);color:var(--text);font-family:system-ui,-apple-system,BlinkMacSystemFont,\"Segoe UI\",Roboto,Arial,sans-serif}main{width:min(100%,520px);margin:auto;padding:20px 16px 34px}.top{display:flex;justify-content:space-between;align-items:center;margin-bottom:16px}.brand{font-size:18px;font-weight:800}.pill{display:flex;gap:7px;align-items:center;border:1px solid #30352a;background:#10130d;border-radius:999px;padding:7px 10px;color:var(--muted);font-size:11px;font-weight:700}.dot{width:7px;height:7px;border-radius:50%;background:#626773}.live{background:var(--accent);box-shadow:0 0 12px #d8ff4d}.hero{position:relative;min-height:330px;border:1px solid #2a3024;border-radius:28px;overflow:hidden;background:linear-gradient(145deg,#1a2115,#080a09 58%,#12150f);box-shadow:0 24px 70px #0009}.hero:before{content:\"\";position:absolute;width:300px;height:300px;right:-110px;top:-120px;background:radial-gradient(circle,#d8ff4d,transparent 68%);opacity:.15}.heroIn{position:absolute;inset:0;display:flex;flex-direction:column;justify-content:flex-end;padding:24px}.mark{width:82px;height:82px;border-radius:24px;border:1px solid #414a35;background:#0d110cbb;display:grid;place-items:center}.mark svg{width:46px;height:46px;fill:none;stroke:currentColor;stroke-width:2.8;stroke-linecap:round}.eyebrow{margin-top:22px;color:var(--accent);font-size:10px;letter-spacing:2px;font-weight:850}.title{font-size:29px;font-weight:850;letter-spacing:-.8px;margin-top:5px}.sub{font-size:13px;color:var(--muted);margin-top:3px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}.wave{display:flex;align-items:center;gap:4px;height:45px;margin-top:18px}.wave i{display:block;width:5px;border-radius:8px;background:var(--accent);opacity:.8;animation:w 1s ease-in-out infinite alternate}.wave i:nth-child(1){height:22%}.wave i:nth-child(2){height:52%;animation-delay:.12s}.wave i:nth-child(3){height:80%;animation-delay:.22s}.wave i:nth-child(4){height:38%;animation-delay:.08s}.wave i:nth-child(5){height:96%;animation-delay:.28s}.wave i:nth-child(6){height:62%;animation-delay:.16s}.wave i:nth-child(7){height:32%;animation-delay:.34s}.wave i:nth-child(8){height:76%;animation-delay:.2s}.wave i:nth-child(9){height:46%;animation-delay:.1s}.wave i:nth-child(10){height:88%;animation-delay:.25s}@keyframes w{to{transform:scaleY(.38)}}.card{margin-top:14px;padding:16px;border:1px solid var(--line);border-radius:20px;background:var(--panel);backdrop-filter:blur(18px)}.controls{display:flex;justify-content:center;align-items:center;gap:14px}.iconBtn{width:50px;height:50px;border:1px solid #2b3039;border-radius:50%;background:#1a1d23;color:#eef1f5;display:grid;place-items:center;cursor:pointer}.iconBtn:active{transform:scale(.94)}.iconBtn svg{width:21px;height:21px;fill:none;stroke:currentColor;stroke-width:2;stroke-linecap:round;stroke-linejoin:round}.play{width:68px;height:68px;background:var(--accent);border-color:var(--accent);color:#090b05}.play svg{width:27px;height:27px}.head{display:flex;justify-content:space-between;font-size:13px}.muted{color:var(--muted)}.value{font-weight:800}input[type=range]{width:100%;margin-top:17px;accent-color:var(--accent)}.grid{display:grid;grid-template-columns:1fr 1fr;gap:10px}.stat{padding:13px;border-radius:15px;background:#171a20;min-width:0}.stat b{display:block;font-size:14px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}.stat span{display:block;margin-top:4px;color:var(--muted);font-size:10px}.section{font-size:12px;font-weight:800;margin-bottom:12px}.links{display:grid;grid-template-columns:1fr 1fr;gap:10px}.link{display:flex;align-items:center;justify-content:center;gap:8px;padding:12px;border-radius:13px;background:#191c22;border:1px solid #272b33;color:#e9edf3;text-decoration:none;font-size:12px;font-weight:700}.link svg{width:16px;height:16px;fill:none;stroke:currentColor;stroke-width:2;stroke-linecap:round;stroke-linejoin:round}.foot{text-align:center;color:#6f7682;font-size:10px;margin-top:16px}</style></head><body><main>\n<div class=\"top\"><div class=\"brand\">WiFi Receiver S3</div><div class=\"pill\"><span id=\"dot\" class=\"dot\"></span><span id=\"live\">WAITING</span></div></div>\n<section class=\"hero\"><div class=\"heroIn\"><div class=\"mark\"><svg viewBox=\"0 0 48 48\"><path d=\"M8 19a22 22 0 0 1 32 0M13 24a15 15 0 0 1 22 0M18 29a8 8 0 0 1 12 0\"/><circle cx=\"24\" cy=\"35\" r=\"2.5\" fill=\"currentColor\" stroke=\"none\"/></svg></div><div class=\"eyebrow\">WIRELESS AUDIO</div><div class=\"title\">Android Audio</div><div id=\"sender\" class=\"sub\">Waiting for Android sender</div><div class=\"wave\">\n<i></i>\n<i></i>\n<i></i>\n<i></i>\n<i></i>\n<i></i>\n<i></i>\n<i></i>\n<i></i>\n<i></i>\n</div></div></section>\n<div class=\"card\"><div class=\"controls\">\n<button class=\"iconBtn\" aria-label=\"Mute\" onclick=\"control({mute:1})\"><svg viewBox=\"0 0 24 24\"><path d=\"M11 5 6 9H3v6h3l5 4V5Z\"/><path d=\"m19 9-6 6m0-6 6 6\"/></svg></button>\n<button id=\"play\" class=\"iconBtn play\" aria-label=\"Pause\" onclick=\"toggle()\"><svg viewBox=\"0 0 24 24\"><path d=\"M8 5v14M16 5v14\"/></svg></button>\n<button class=\"iconBtn\" aria-label=\"Unmute\" onclick=\"control({mute:0})\"><svg viewBox=\"0 0 24 24\"><path d=\"M11 5 6 9H3v6h3l5 4V5Z\"/><path d=\"M15 9.5a4 4 0 0 1 0 5M18 7a8 8 0 0 1 0 10\"/></svg></button>\n</div></div>\n<div class=\"card\"><div class=\"head\"><span class=\"muted\">Output volume</span><span id=\"vol\" class=\"value\">100%</span></div><input id=\"slider\" type=\"range\" min=\"0\" max=\"100\" value=\"100\" oninput=\"setVol(this.value)\"></div>\n<div class=\"card\"><div class=\"section\">Receiver status</div><div class=\"grid\"><div class=\"stat\"><b id=\"conn\">Offline</b><span>Connection</span></div><div class=\"stat\"><b id=\"format\">48 kHz · stereo</b><span>Audio format</span></div><div class=\"stat\"><b id=\"buf\">0 KB</b><span>Buffer</span></div><div class=\"stat\"><b id=\"lost\">0</b><span>Lost packets</span></div><div class=\"stat\"><b id=\"drop\">0</b><span>Dropped packets</span></div><div class=\"stat\"><b id=\"under\">0</b><span>Underruns</span></div></div></div>\n<div class=\"card\"><div class=\"section\">Device</div><div class=\"links\"><a class=\"link\" href=\"/update\">↥ OTA Update</a><a class=\"link\" href=\"/\" >↻ Refresh</a></div></div>\n<div class=\"foot\">wifi-receiver-s3.local · UDA1334A · WFAS v2</div></main>\n<script>let paused=false;async function control(o){try{await fetch(\"/api/control?\"+new URLSearchParams(o));await refresh()}catch(e){}}function setVol(v){document.getElementById(\"vol\").textContent=v+\"%\";fetch(\"/api/control?volume=\"+v)}function toggle(){paused=!paused;control({pause:paused?1:0})}function icon(p){return p?\"▶\":\"Ⅱ\"}async function refresh(){try{let x=await(await fetch(\"/api/status\",{cache:\"no-store\"})).json();let live=x.connected;document.getElementById(\"live\").textContent=live?\"LIVE\":\"WAITING\";document.getElementById(\"dot\").className=\"dot\"+(live?\" live\":\"\");document.getElementById(\"conn\").textContent=live?\"Streaming\":\"Waiting\";document.getElementById(\"sender\").textContent=live?(x.senderName+\" · \"+x.sender):\"Waiting for Android sender\";document.getElementById(\"format\").textContent=(x.sampleRate/1000)+\" kHz · \"+(x.channels===2?\"stereo\":\"mono\");document.getElementById(\"buf\").textContent=Math.round(x.buffer/1024)+\" KB\";document.getElementById(\"lost\").textContent=x.lost;document.getElementById(\"drop\").textContent=x.dropped;document.getElementById(\"under\").textContent=x.underruns;document.getElementById(\"slider\").value=x.volume;document.getElementById(\"vol\").textContent=x.volume+\"%\";paused=x.paused;document.getElementById(\"play\").textContent=icon(paused)}catch(e){document.getElementById(\"live\").textContent=\"OFFLINE\";document.getElementById(\"dot\").className=\"dot\"}}</script></body></html>")));}
static void handleUpdatePage(){web.send(200,"text/html","<!doctype html><html><body style=\"font-family:system-ui;background:#08090b;color:#fff;padding:30px\"><h2>WiFi Receiver S3 OTA</h2><form method=\"POST\" action=\"/update\" enctype=\"multipart/form-data\"><input type=\"file\" name=\"firmware\" accept=\".bin\"><button type=\"submit\">Upload firmware</button></form><p><a href=\"/\">Back</a></p></body></html>");}static void handleUpdateUpload(){HTTPUpload& u=web.upload();if(u.status==UPLOAD_FILE_START){if(!Update.begin(UPDATE_SIZE_UNKNOWN))Update.printError(Serial);}else if(u.status==UPLOAD_FILE_WRITE){if(Update.write(u.buf,u.currentSize)!=u.currentSize)Update.printError(Serial);}else if(u.status==UPLOAD_FILE_END){if(Update.end(true))Serial.printf("[OTA] Success: %u bytes\\n",u.totalSize);else Update.printError(Serial);}}
static void handleUpdateDone(){web.sendHeader("Connection","close");web.send(200,"text/plain",Update.hasError()?"OTA FAILED":"OTA OK - rebooting");delay(400);if(!Update.hasError())ESP.restart();}
static void startWeb(){web.on("/",HTTP_GET,handleRoot);web.on("/api/status",HTTP_GET,handleApiStatus);web.on("/api/control",HTTP_GET,handleApiControl);web.on("/update",HTTP_GET,handleUpdatePage);web.on("/update",HTTP_POST,handleUpdateDone,handleUpdateUpload);web.begin();Serial.println("[WEB] Dashboard ready");}

static void connectWiFi(){
  WiFi.mode(WIFI_STA);WiFi.setSleep(false);WiFi.setHostname(HOSTNAME);WiFi.begin(WIFI_SSID,WIFI_PASSWORD);
  Serial.printf("[WiFi] connecting to %s",WIFI_SSID);uint32_t t=millis();
  while(WiFi.status()!=WL_CONNECTED&&millis()-t<30000){delay(250);Serial.print('.');}Serial.println();
  if(WiFi.status()!=WL_CONNECTED)return;
  Serial.printf("[WiFi] %s RSSI=%d\n",WiFi.localIP().toString().c_str(),WiFi.RSSI());
  if(MDNS.begin(HOSTNAME))Serial.printf("[mDNS] %s.local\n",HOSTNAME);
}

static bool webReady=false;
static bool discoveryReady=false;
static bool mdnsReady=false;
static uint32_t lastWiFiAttempt=0;

static void ensureNetworkServices(){
  if(WiFi.status()!=WL_CONNECTED)return;
  if(!mdnsReady){
    if(MDNS.begin(HOSTNAME)){
      mdnsReady=true;
      MDNS.addService("http","tcp",80);
      Serial.printf("[mDNS] http://%s.local/\n",HOSTNAME);
    }else Serial.println("[mDNS] start failed; will retry");
  }
  if(!webReady){startWeb();webReady=true;}
  if(!discoveryReady){
    IPAddress group;group.fromString(MCAST);
    if(discovery.beginMulticast(group,DISCOVERY_PORT)){
      discoveryReady=true;
      Serial.printf("[DISCOVERY] listening %s:%u\n",MCAST,DISCOVERY_PORT);
      Serial.println("[READY] waiting for WiFiAudioStreaming-Android v1.2");
    }else Serial.println("[DISCOVERY] multicast join FAILED; will retry");
  }
}

void setup(){
  Serial.begin(115200);delay(300);
  Serial.println("\n=== WiFi_Receiver_S3 | WFAS v2 | AUTO RECEIVER ===");
  Serial.printf("[UDA1334A] BCLK=%d LRCK=%d DIN=%d\n",BCLK,LRCK,DOUT);
  rb=(uint8_t*)heap_caps_malloc(RB_SIZE,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
  if(!rb)rb=(uint8_t*)heap_caps_malloc(RB_SIZE,MALLOC_CAP_8BIT);
  if(!rb){Serial.println("[FATAL] audio buffer allocation failed");while(true)delay(1000);}rbClear();
  setupI2S(DEFAULT_SR,DEFAULT_CH);xTaskCreatePinnedToCore(audioTask,"WFAS-AUDIO",8192,nullptr,20,nullptr,0);
  connectWiFi();
  ensureNetworkServices();
}

void loop(){
  if(WiFi.status()!=WL_CONNECTED){
    if(millis()-lastWiFiAttempt>5000){lastWiFiAttempt=millis();Serial.println("[WiFi] reconnecting...");WiFi.reconnect();}
    delay(10);return;
  }
  ensureNetworkServices();
  serviceDiscovery();serviceAudio();web.handleClient();
  if(session&&!multicastSession&&!lastAudio&&millis()-lastHello>1500)sendHello();
  if(session&&!multicastSession&&lastAudio&&millis()-lastAudio>5000)stopSession("audio timeout");
  delay(1);
}
