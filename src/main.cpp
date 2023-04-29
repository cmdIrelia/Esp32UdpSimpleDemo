#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#define LED_PIN 2 

#define KEY_1 35
#define KEY_2 32
#define KEY_3 25
#define KEY_4 27
#define KEY_5 23
#define KEY_6 22
#define KEY_7 21
#define KEY_8 4

//	WiFi的初始化和连接
#define WIFI_SSID "wifiwifi"
#define WIFI_PASSWORD "qweqweqwe"
bool wifi_connected = false;

// UDP类
WiFiUDP udp_Data;
const int udpPort_data = 8888;
IPAddress remoteIp(WiFi.localIP()); //远端地址初始化，Connect()以后更新

// 上一拍按键值
uint8_t previous_key_state;

void WiFi_Connect()
{
  Serial.println("Start Wifi Connecting.");
	WiFi.begin(WIFI_SSID,WIFI_PASSWORD);
	while (WiFi.status() != WL_CONNECTED)
	{ //这里是阻塞程序，直到连接成功
		delay(300);
		Serial.print(".");
	}
  Serial.print("Start Wifi Connected at: ");
  Serial.println(WiFi.localIP());
  //远端地址
  remoteIp = WiFi.localIP();
  remoteIp[3] = 190;
  Serial.print("Remote IP: ");
  Serial.println(remoteIp);
}

void Wifi_connected(WiFiEvent_t event, WiFiEventInfo_t info) {
  wifi_connected = true;
  udp_Data.begin(WiFi.localIP(),udpPort_data);
  Serial.println("Successfully connected to Access Point");
}

void Wifi_disconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  wifi_connected = false;
  Serial.println("Disconnected from WIFI access point");
  Serial.print("WiFi lost connection. Reason: ");
  //Serial.println(info.disconnected.reason);
  Serial.println("Reconnecting...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  //注册WiFi断开连接事件
  WiFi.onEvent(Wifi_disconnected, arduino_event_id_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  //WiFi连接事件注册
  WiFi.onEvent(Wifi_connected, arduino_event_id_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
  //连接WiFi
  WiFi_Connect();
  udp_Data.begin(WiFi.localIP(),udpPort_data);

  //按键输入初始化
  pinMode(KEY_1, INPUT_PULLDOWN);
  pinMode(KEY_2, INPUT_PULLDOWN);
  pinMode(KEY_3, INPUT_PULLDOWN);
  pinMode(KEY_4, INPUT_PULLDOWN);
  pinMode(KEY_5, INPUT_PULLDOWN);
  pinMode(KEY_6, INPUT_PULLDOWN);
  pinMode(KEY_7, INPUT_PULLDOWN);
  pinMode(KEY_8, INPUT_PULLDOWN);
  
  previous_key_state = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t input_state = 0;
  input_state = digitalRead(KEY_1)<<7 | 
                digitalRead(KEY_2)<<6 | 
                digitalRead(KEY_3)<<5 | 
                digitalRead(KEY_4)<<4 | 
                digitalRead(KEY_5)<<3 | 
                digitalRead(KEY_6)<<2 | 
                digitalRead(KEY_7)<<1 |
                digitalRead(KEY_8);

  if(input_state!=previous_key_state)
  {
    previous_key_state = input_state;

    udp_Data.beginPacket(remoteIp, udpPort_data);
    uint8_t data = (unsigned char)('2');
    udp_Data.write(&data, 1);
    udp_Data.endPacket();
    
    Serial.print("input state: 8'b");
    Serial.println(input_state, BIN);

    Serial.print("Send Data: ");
    Serial.println(data);
  }
  if(wifi_connected==false)  delay(100);
  else delay(1000);
  digitalWrite(LED_PIN,1-digitalRead(LED_PIN));
}
