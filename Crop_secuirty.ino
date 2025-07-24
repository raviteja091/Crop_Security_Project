#include <LiquidCrystal.h>
#include <DFRobot_DHT11.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "esp_camera.h"
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
#include <NewPing.h>

// LCD pins
const int rs=8,en=9,d4=10,d5=11,d6=12,d7=13;
LiquidCrystal lcd(rs,en,d4,d5,d6,d7);

// DHT11 sensor
#define DHT11_PIN A1
DFRobot_DHT11 DHT;

// Ultrasonic pins
#define TRIG_PIN 6
#define ECHO_PIN 7
NewPing sonar(TRIG_PIN, ECHO_PIN);

// Light sensors
const int L1=A2, L2=A3, L3=A4, L4=A5, LDR=36;
// Motor / buzzer
const int MOTOR_PIN=4, BUZ=5;

// Wi-Fi & Telegram
const char* ssid="123456789";
const char* password="123456789";
String BOTtoken="6829296393:AAFEZW4L3hIUcUN7Q1x-5UO3be026lZVh0I";
String chatId="5468964605";
WiFiClientSecure clientTCP;
UniversalTelegramBot bot(BOTtoken, clientTCP);

// Frame capture pins
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

bool sendPhoto=false;
unsigned long botLastTime=0;
#define BOT_MTBS 1000
#define FIRE_SENSOR_PIN 13
#define BUTTON_PIN 12
#define FLASH_LED 4

void setup(){
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(L1, INPUT);
  pinMode(L2, INPUT);
  pinMode(L3, INPUT);
  pinMode(L4, INPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(BUZ, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(FLASH_LED, OUTPUT);

  lcd.begin(16,2);
  lcd.print("WELCOME");
  DHT.read(DHT11_PIN);

  // Wi-Fi + camera init
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid,password);
  clientTCP.setCACert(TELEGRAM_CERTIFICATE_ROOT);
  while(WiFi.status()!=WL_CONNECTED){ delay(500); Serial.print("."); }
  camera_config_t config = {
    .ledc_channel = LEDC_CHANNEL_0, .ledc_timer = LEDC_TIMER_0,
    .pin_d0 = Y2_GPIO_NUM, .pin_d1 = Y3_GPIO_NUM, .pin_d2 = Y4_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM, .pin_d4 = Y6_GPIO_NUM, .pin_d5 = Y7_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM, .pin_d7 = Y9_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM, .pin_pclk = PCLK_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM, .pin_href = HREF_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM, .pin_sscb_scl = SIOC_GPIO_NUM,
    .pin_pwdn = PWDN_GPIO_NUM, .pin_reset = RESET_GPIO_NUM,
    .xclk_freq_hz = 20000000, .pixel_format = PIXFORMAT_JPEG,
    .frame_size = (psramFound()? FRAMESIZE_UXGA:FRAMESIZE_SVGA),
    .jpeg_quality = (psramFound()?10:12), .fb_count = (psramFound()?2:1)
  };
  esp_camera_init(&config);
}

void sendPhotoTelegram(const String &msg){
  bot.sendMessage(chatId, msg, "");
}

void loop(){
  // Read sensors
  DHT.read(DHT11_PIN);
  int tval=DHT.temperature, hval=DHT.humidity;
  digitalWrite(TRIG_PIN,1); delayMicroseconds(10);
  digitalWrite(TRIG_PIN,0);
  int dist=sonar.ping_cm();
  int l1Val=analogRead(L1)/100;
  int l2Val=9-analogRead(L2)/100;
  int l3Val=analogRead(L3)/100;
  int l4Val=analogRead(L4)/100;
  int mVal=10-analogRead(LDR)/102;

  // Display on LCD
  lcd.clear();
  lcd.print("E:"+String(l1Val)+" W:"+String(l2Val));
  lcd.setCursor(0,1);
  lcd.print("M:"+mVal+" D:"+dist+" T:"+tval+" H:"+hval);

  // Motor control
  digitalWrite(MOTOR_PIN, mVal<4?1:0);

  // Alerts
  if(l1Val<5||l2Val<5||l3Val<5||l4Val<5||tval>36||dist<10){
    digitalWrite(BUZ,1); delay(300); digitalWrite(BUZ,0);
  }

  // Telegram triggers
  if(digitalRead(FIRE_SENSOR_PIN)==LOW){
    sendPhotoTelegram("Flame detected!");
    sendPhoto=true;
  }
  if(dist<=10 && !sendPhoto){
    sendPhotoTelegram("Obstacle detected!");
    sendPhoto=true;
  }
  if(sendPhoto){
    digitalWrite(FLASH_LED,1); delay(200);
    camera_fb_t *fb=esp_camera_fb_get();
    if(fb){
      WiFiClientSecure &client=clientTCP;
      if(client.connect("api.telegram.org",443)){
        String head="--IotHub\r\nContent-Disposition: form-data; name=\"chat_id\"\r\n\r\n"+chatId+"\r\n";
        head+="--IotHub\r\nContent-Disposition: form-data; name=\"photo\"; filename=\"img.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
        String tail="\r\n--IotHub--\r\n";
        client.printf("POST /bot%s/sendPhoto HTTP/1.1\r\nHost: api.telegram.org\r\nContent-Length: %d\r\nContent-Type: multipart/form-data; boundary=IotHub\r\n\r\n",
                      BOTtoken.c_str(), head.length()+fb->len+tail.length());
        client.print(head);
        client.write(fb->buf, fb->len);
        client.print(tail);
        esp_camera_fb_return(fb);
        while(client.available()){ client.read(); }
        client.stop();
      }
      digitalWrite(FLASH_LED,0);
    }
    sendPhoto=false;
  }

  // Button-triggered photo
  if(digitalRead(BUTTON_PIN)==LOW){
    sendPhotoTelegram("Button pressed!");
    sendPhoto=false;
  }

  // Telegram polling
  if(millis()-botLastTime>BOT_MTBS){
    int msgs=bot.getUpdates(bot.last_message_received+1);
    while(msgs){
      msgs=bot.getUpdates(bot.last_message_received+1);
    }
    botLastTime=millis();
  }

  delay(1000);
}
