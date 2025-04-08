#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_now.h>
#include <WiFi.h>
#include <EEPROM.h>
#include "driver/rmt.h"

Adafruit_SSD1306 display(128, 64, &Wire, -1);

unsigned long displayDelay = 25;
unsigned long nextUpdate = 0;

float alpha = 0.40;

bool printData = false;

hw_timer_t* timer = NULL;
hw_timer_t* timer2 = NULL;

int ch1_min = 0,ch1_max =0;
int ch2_min = 0,ch2_max =0;
int ch3_min = 0,ch3_max =0;
int ch4_min = 0,ch4_max =0;


int tch1_min = 0,tch1_max =0;
int tch2_min = 0,tch2_max =0;
int tch3_min = 0,tch3_max =0;
int tch4_min = 0,tch4_max =0;

int tch1,tch2,tch3,tch4;

bool isCalib = false;

typedef struct struct_message {
    int ch1;
    int ch2;
    int ch3;
    int ch4;
} struct_message;

struct_message packet;

esp_now_peer_info_t peerInfo;

uint8_t broadcastAddress[] = {0x64, 0xE8, 0x33, 0xBA, 0x16, 0x14};

#define CPPM_PIN 25
#define NUM_CHANNELS 8
#define FRAME_LENGTH_US 22500  // total frame length in µs
#define GAP_LENGTH_US 300      // gap between pulses
#define RMT_CHANNEL RMT_CHANNEL_0


int ch1=0,ch2=0,ch3=0,ch4=0;
int trim_ch1 = 0,trim_ch2=0,trim_ch3=0,trim_ch4=0;

void onTimer() {
  packet.ch1 = ch1;
  packet.ch2 = ch2;
  packet.ch3 = ch3;
  packet.ch4 = ch4;
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &packet, sizeof(packet));
}

void sendCPPMFrame() {
  uint16_t channels[] = {ch1+1000, ch2+1000, ch3+1000, ch4+1000,1500,1500,1500,1500};

  rmt_item32_t items[(NUM_CHANNELS * 2) + 2]; // each channel = 2 items, plus sync pulse

  int itemIndex = 0;
  uint16_t totalPulseTime = 0;

  for (int i = 0; i < NUM_CHANNELS; i++) {
    // HIGH pulse (channel data)
    items[itemIndex].duration0 = GAP_LENGTH_US;
    items[itemIndex].level0 = 0;
    items[itemIndex].duration1 = channels[i];
    items[itemIndex].level1 = 1;
    itemIndex++;
    totalPulseTime += channels[i] + GAP_LENGTH_US;
  }

  // Sync pulse = remaining time in frame (stays LOW)
  uint16_t syncLength = 21600 - totalPulseTime;
  if (syncLength < 300) syncLength = 300;  // ensure sync is not too short

  items[itemIndex].duration0 = GAP_LENGTH_US;
  items[itemIndex].level0 = 0;
  items[itemIndex].duration1 = syncLength; // small spacer
  items[itemIndex].level1 = 1;
  itemIndex++;

  // Send frame
  rmt_write_items(RMT_CHANNEL, items, itemIndex, false);  // wait_tx_done = true
}


void setupRMT() {
  rmt_config_t config = RMT_DEFAULT_CONFIG_TX((gpio_num_t)CPPM_PIN, RMT_CHANNEL);
  config.clk_div = 80; // 1 tick = 1 µs (80MHz / 80 = 1MHz)
  rmt_config(&config);
  rmt_driver_install(RMT_CHANNEL, 0, 0);
}


void updateDisplay()
{
  display.clearDisplay();
  display.drawRect(10, 20, 40, 40, SSD1306_WHITE);
  display.drawRect(70, 20, 40, 40, SSD1306_WHITE);
  display.drawFastHLine(28, 40, 5, SSD1306_WHITE);
  display.drawFastVLine(30, 38, 5, SSD1306_WHITE);
  display.drawFastHLine(88, 40, 5, SSD1306_WHITE);
  display.drawFastVLine(90, 38, 5, SSD1306_WHITE);
  int c1map = map(ch1,0,1000,40,0);
  int c2map = map(ch2,0,1000,0,40);
  int c3map = map(ch3,0,1000,0,40);
  int c4map = map(ch4,0,1000,40,0);

  display.drawCircle(10 + c3map, 20+c4map, 2, SSD1306_WHITE);
  display.drawCircle(70 + c2map, 20+c1map, 2, SSD1306_WHITE);


  display.display();
}


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void eraseData()
{
  for(int i=0;i<256;i++)
  {
    EEPROM.write(i,0x00);
  }
  EEPROM.commit();
}

void saveData()
{
  uint8_t buffer[256];
  for(int i=0;i<256;i++)
  {
    buffer[i] = 0x00;
  }
  uint8_t crc = 0x00;
  buffer[0] = 0xCC;
  buffer[1] = 0xAA;
  buffer[2]  = (ch1_min >> 8) & 0xFF;
  buffer[3]  = ch1_min & 0xFF;
  buffer[4]  = (ch1_max >> 8) & 0xFF;
  buffer[5]  = ch1_max & 0xFF;
  buffer[6]  = (ch2_min >> 8) & 0xFF;
  buffer[7]  = ch2_min & 0xFF;
  buffer[8]  = (ch2_max >> 8) & 0xFF;
  buffer[9]  = ch2_max & 0xFF;
  buffer[10] = (ch3_min >> 8) & 0xFF;
  buffer[11] = ch3_min & 0xFF;
  buffer[12] = (ch3_max >> 8) & 0xFF;
  buffer[13] = ch3_max & 0xFF;
  buffer[14] = (ch4_min >> 8) & 0xFF;
  buffer[15] = ch4_min & 0xFF;
  buffer[16] = (ch4_max >> 8) & 0xFF;
  buffer[17] = ch4_max & 0xFF;
  buffer[18] = (((int16_t)trim_ch1) >> 8) & 0xFF;
  buffer[19] = ((int16_t)trim_ch1) & 0xFF;
  buffer[20] = (((int16_t)trim_ch2) >> 8) & 0xFF;
  buffer[21] = ((int16_t)trim_ch2) & 0xFF;
  buffer[22] = (((int16_t)trim_ch3) >> 8) & 0xFF;
  buffer[23] = ((int16_t)trim_ch3) & 0xFF;
  buffer[24] = (((int16_t)trim_ch4) >> 8) & 0xFF;
  buffer[25] = ((int16_t)trim_ch4) & 0xFF;
  for(int i=0;i<255;i++)
  {
    crc ^= buffer[i];
  }
  buffer[255] = crc;
  for(int i=0;i<256;i++)
  {
    EEPROM.write(i,buffer[i]);
  }
  EEPROM.commit();


}

void readData()
{
 bool isValid = false;
 uint8_t buffer[256];
 uint8_t crc = 0x00;
 for(int i=0;i<256;i++)
 {
  buffer[i] = EEPROM.read(i);
  if(i<254)
  {
    crc ^= buffer[i];
  }
  else
  {
    if(buffer[i] == crc)
    {
      isValid = true;
    }
  }
 }

   if(isValid && (buffer[0] == 0xCC) && (buffer[1]==0xAA))
  {
    ch1_min = (int) (buffer[2] << 8) | buffer[3];
    ch1_max = (int) (buffer[4] << 8) | buffer[5];
    ch2_min = (int) (buffer[6] << 8) | buffer[7];
    ch2_max = (int) (buffer[8] << 8) | buffer[9];
    ch3_min = (int) (buffer[10] << 8) | buffer[11];
    ch3_max = (int) (buffer[12] << 8) | buffer[13];
    ch4_min = (int) (buffer[14] << 8) | buffer[15];
    ch4_max = (int) (buffer[16] << 8) | buffer[17];
    trim_ch1 = ((int) buffer[18] <<8 ) | buffer[19];
    trim_ch2 = ((int) buffer[20] <<8 ) | buffer[21];
    trim_ch3 = ((int) buffer[22] <<8 ) | buffer[23];
    trim_ch4 = ((int) buffer[24] <<8 ) | buffer[25];
    Serial.println("Data Load OK");
  }
  else{

    ch1_min = 0;
    ch1_max = 4096;
    ch2_min = 0;
    ch2_max = 4096;
    ch3_min = 0;
    ch3_max = 4096;
    ch4_min = 0;
    ch4_max = 4096;
    trim_ch1 = 0;
    trim_ch2 = 0;
    trim_ch3 = 0;
    trim_ch4 = 0;
    Serial.println("Invalid Data! Loaded defaults");
  }

}


void setup() {
  pinMode(32,INPUT);
  pinMode(33,INPUT);
  pinMode(34,INPUT);
  pinMode(35,INPUT);
  EEPROM.begin(256);
  Serial.begin(9600);
  readData();
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  setupRMT();
  sendCPPMFrame();
  timer = timerBegin(1000000); // Timer 0, prescaler 80 (1µs tick), count up
  timerAttachInterrupt(timer, [](){
    sendCPPMFrame();
    onTimer();
  });
  timerAlarm(timer, 22000, true, 0); // Enable the timer

    // send first frame

}

void loop() {
  // put your main code here, to run repeatedly:
  int temp1=0,temp2=0,temp3=0,temp4=0;
  int chx1,chx2,chx3,chx4;
  

  if(millis() >= nextUpdate)
  {
    updateDisplay();
    nextUpdate = displayDelay + millis();
  }

  for(int i=0;i<5;i++)
  {
    temp1 = temp1 + analogRead(32);
    delay(1);
  }

  chx1 = temp1/5;
  int rch1 = map(chx1,ch1_min,ch1_max,0,1000) + trim_ch1;
  rch1 = constrain(rch1, 0, 1000);
  ch1 = alpha * rch1 + (1-alpha) * ch1;
  

  for(int i=0;i<5;i++)
  {
    temp2 = temp2 + analogRead(33);
    delay(1);
  }

  chx2 = temp2/5;
  int rch2 = map(chx2,ch2_min,ch2_max,0,1000) + trim_ch2;
  rch2 = constrain(rch2, 0, 1000);
  ch2 = alpha * rch2 +(1-alpha) * ch2;

  for(int i=0;i<5;i++)
  {
    temp3 = temp3 + analogRead(34);
    delay(1);
  }

  chx3 = temp3/5;
  int rch3 = map(chx3,ch3_min,ch3_max,0,1000) + trim_ch3;
  rch3 = constrain(rch3, 0, 1000);
  ch3 = alpha * rch3 + (1-alpha) * ch3;

  for(int i=0;i<5;i++)
  {
    temp4 = temp4 + analogRead(35);
    delay(1);
  }

  chx4 = temp4/5;
  int rch4 = map(chx4,ch4_min,ch4_max,0,1000) + trim_ch4;
  rch4 = constrain(rch4, 0, 1000);
  ch4 = alpha * rch4 + (1-alpha) * ch4;



  if(Serial.available() > 0)
  {
      char cmd = Serial.read();
      if(cmd == 'c')
      {
        isCalib = true;
        tch1_min = chx1;
        tch1_max = chx1;
        tch2_min = chx2;
        tch2_max = chx2;
        tch3_min = chx3;
        tch3_max = chx3;
        tch4_min = chx4;
        tch4_max = chx4;
        Serial.println("Calibration Started");
      }
      else if(cmd == 's')
      {
        isCalib = false;
        ch1_min = tch1_min;
        ch1_max = tch1_max;
        ch2_min = tch2_min;
        ch2_max = tch2_max;
        ch3_min = tch3_min;
        ch3_max = tch3_max;
        ch4_min = tch4_min;
        ch4_max = tch4_max;
        trim_ch1 = 500 - ch1;
        trim_ch2 = 500 - ch2;
        trim_ch3 = 500 - ch3;
        trim_ch4 = 500 - ch4;
        saveData();
        Serial.println("Calibration Ended");
        Serial.print("Ch1 -> Min: ");
        Serial.print(ch1_min);
        Serial.print(" Max: ");
        Serial.println(ch1_max);

        Serial.print("Ch2 -> Min: ");
        Serial.print(ch2_min);
        Serial.print(" Max: ");
        Serial.println(ch2_max);

        Serial.print("Ch3 -> Min: ");
        Serial.print(ch3_min);
        Serial.print(" Max: ");
        Serial.println(ch3_max);

        Serial.print("Ch4 -> Min: ");
        Serial.print(ch4_min);
        Serial.print(" Max: ");
        Serial.println(ch4_max);
      }
      else if(cmd == 'd')
      {
        printData = !printData;
      }
  }

  if(isCalib)
  {
    if(chx1 > tch1_max)
    {
      tch1_max = chx1;
    }

    if(chx1 < tch1_min)
    {
      tch1_min = chx1;
    }

    if(chx2 > tch2_max)
    {
      tch2_max = chx2;
    }

    if(chx2 < tch2_min)
    {
      tch2_min = chx2;
    }

    if(chx3 > tch3_max)
    {
      tch3_max = chx3;
    }

    if(chx3 < tch3_min)
    {
      tch3_min = chx3;
    }

    if(chx4 > tch4_max)
    {
      tch4_max = chx4;
    }

    if(chx4 < tch4_min)
    {
      tch4_min = chx4;
    }

  }

  if(printData)
  {
    Serial.print(ch1);
    Serial.print(",");
    Serial.print(ch2);
    Serial.print(",");
    Serial.print(ch3);
    Serial.print(",");
    Serial.println(ch4);
    
  }

  


}
