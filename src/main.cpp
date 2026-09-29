#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <driver/i2s.h>
#include <esp_heap_caps.h>
#include "wfas.h"

// WFAS v2 headless receiver for WiFiAudioStreaming-Android v1.2.
// Boot -> Wi-Fi -> discovery -> unicast HELLO or multicast join -> PCM -> UDA1334A.
// UDA1334A: BCLK GPIO4, LRCK GPIO5, DIN GPIO21.

static constexpr char WIFI_SSID[] = "YOUR_WIFI_SSID";
static constexpr char WIFI_PASSWORD[] = "YOUR_WIFI_PASSWORD";
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

static void rbClear(){portENTER_CRITICAL(&rbMux);head=tail=count=0;portEXIT_CRITICAL(&rbMux);}
static size_t rbAvail(){portENTER_CRITICAL(&rbMux);size_t n=count;portEXIT_CRITICAL(&rbMux);return n;}
static size_t rbFree(){portENTER_CRITICAL(&rbMux);size_t n=RB_SIZE-count;portEXIT_CRITICAL(&rbMux);return n;}
static size_t rbPut(const uint8_t*p,size_t n){portENTER_CRITICAL(&rbMux);size_t w=min(n,RB_SIZE-count),a=min(w,RB_SIZE-head);memcpy(rb+head,p,a);if(w>a)memcpy(rb,p+a,w-a);head=(head+w)%RB_SIZE;count+=w;portEXIT_CRITICAL(&rbMux);return w;}
static size_t rbGet(uint8_t*p,size_t n){portENTER_CRITICAL(&rbMux);size_t r=min(n,count),a=min(r,RB_SIZE-tail);memcpy(p,rb+tail,a);if(r>a)memcpy(p+a,rb,r-a);tail=(tail+r)%RB_SIZE;count-=r;portEXIT_CRITICAL(&rbMux);return r;}

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
    if(!primed){if(rbAvail()<PREBUFFER){vTaskDelay(pdMS_TO_TICKS(2));continue;}primed=true;}
    size_t n=rbGet(block,sizeof(block));
    if(!n){underruns++;i2s_write(I2S_NUM_0,zero,sizeof(zero),nullptr,portMAX_DELAY);primed=false;continue;}
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
  audio.stop();if(!audio.begin(0))return false;peer=ip;peerPort=port;multicastSession=false;session=false;haveExpected=false;rbClear();
  setupI2S(rate,channels);sendHello();
  Serial.printf("[WFAS] unicast -> %s:%u local=%u\n",peer.toString().c_str(),peerPort,audio.localPort());return true;
}

static bool startMulticast(uint16_t port,uint32_t rate,uint8_t channels){
  IPAddress group;group.fromString(MCAST);audio.stop();
  if(!audio.beginMulticast(WiFi.localIP(),group,port))return false;
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

static void connectWiFi(){
  WiFi.mode(WIFI_STA);WiFi.setSleep(false);WiFi.setHostname(HOSTNAME);WiFi.begin(WIFI_SSID,WIFI_PASSWORD);
  Serial.printf("[WiFi] connecting to %s",WIFI_SSID);uint32_t t=millis();
  while(WiFi.status()!=WL_CONNECTED&&millis()-t<30000){delay(250);Serial.print('.');}Serial.println();
  if(WiFi.status()!=WL_CONNECTED)return;
  Serial.printf("[WiFi] %s RSSI=%d\n",WiFi.localIP().toString().c_str(),WiFi.RSSI());
  if(MDNS.begin(HOSTNAME))Serial.printf("[mDNS] %s.local\n",HOSTNAME);
}

void setup(){
  Serial.begin(115200);delay(300);
  Serial.println("\n=== WiFi_Receiver_S3 | WFAS v2 | AUTO RECEIVER ===");
  Serial.printf("[UDA1334A] BCLK=%d LRCK=%d DIN=%d\n",BCLK,LRCK,DOUT);
  rb=(uint8_t*)heap_caps_malloc(RB_SIZE,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
  if(!rb)rb=(uint8_t*)heap_caps_malloc(RB_SIZE,MALLOC_CAP_8BIT);
  if(!rb){Serial.println("[FATAL] audio buffer allocation failed");while(true)delay(1000);}rbClear();
  setupI2S(DEFAULT_SR,DEFAULT_CH);xTaskCreatePinnedToCore(audioTask,"WFAS-AUDIO",8192,nullptr,20,nullptr,0);
  connectWiFi();if(WiFi.status()!=WL_CONNECTED)return;
  IPAddress group;group.fromString(MCAST);
  if(discovery.beginMulticast(WiFi.localIP(),group,DISCOVERY_PORT))Serial.printf("[DISCOVERY] listening %s:%u\n",MCAST,DISCOVERY_PORT);
  else Serial.println("[DISCOVERY] multicast join FAILED");
  Serial.println("[READY] waiting for WiFiAudioStreaming-Android v1.2");
}

void loop(){
  if(WiFi.status()!=WL_CONNECTED){static uint32_t retry=0;if(millis()-retry>5000){retry=millis();WiFi.reconnect();}delay(10);return;}
  serviceDiscovery();serviceAudio();
  if(session&&!multicastSession&&!lastAudio&&millis()-lastHello>1500)sendHello();
  if(session&&!multicastSession&&lastAudio&&millis()-lastAudio>5000)stopSession("audio timeout");
  delay(1);
}
