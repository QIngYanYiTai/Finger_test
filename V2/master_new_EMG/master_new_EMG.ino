#include <Arduino.h>
#include <M5GFX.h>
#include <Wire.h>
#include "logo.h"

// ================== 硬件引脚配置 ==================
#define I2C_SDA         4   // StickS3 扩展口引脚
#define I2C_SCL         5   
#define I2C_FREQ        50000 

#define BTN_PREV        12  
#define BTN_NEXT        11  

// ================== EMG 传感器配置 ==================
#define EMG_PIN 9           // 肌电传感器接入的模拟引脚
#define THRESHOLD 1500      // 触发握拳的模拟量阈值
#define DEBOUNCE_COUNT 8    // 消抖采样次数

enum MuscleState { RELAX, FIST };
MuscleState currentState = RELAX;
MuscleState lastState = RELAX;
int stateCounter = 0;
unsigned long lastEmgTime = 0; // 用于非阻塞采样控制

// ================== 系统参数 ==================
#define MAX_DEVICES     16  
#define DEVICES_PER_PAGE 4  
#define FACTORY_ADDR    0x60 
#define ADDR_POOL_START 0x61 

struct I2CDevice {
  uint8_t addr;
  uint8_t failCount;
  bool online;
  uint8_t lastPos;
  uint8_t lastState;
};

// ================== 全局变量 ==================
M5GFX display;
I2CDevice devices[MAX_DEVICES];
uint8_t deviceCount = 0;
uint8_t currentPage = 0; 
unsigned long lastScanTime = 0;
unsigned long lastStatTime = 0;

// ================== I2C 管理逻辑 ==================

void initI2C() {
  Wire.end();
  delay(10);
  Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);
  Wire.setTimeOut(50);
}

void recoverBus() {
  Wire.end();
  pinMode(I2C_SCL, OUTPUT_OPEN_DRAIN);
  pinMode(I2C_SDA, OUTPUT_OPEN_DRAIN);
  for (int i = 0; i < 9; i++) {
    digitalWrite(I2C_SCL, LOW); delayMicroseconds(5);
    digitalWrite(I2C_SCL, HIGH); delayMicroseconds(5);
  }
  digitalWrite(I2C_SDA, LOW); delayMicroseconds(5);
  digitalWrite(I2C_SCL, HIGH); delayMicroseconds(5);
  digitalWrite(I2C_SDA, HIGH);
  delay(10);
  initI2C();
}

uint8_t findFreeAddress() {
  for (uint8_t a = ADDR_POOL_START; a <= 0x7E; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() != 0) {
      bool assigned = false;
      for (int i = 0; i < deviceCount; i++) {
        if (devices[i].addr == a && devices[i].online) { assigned = true; break; }
      }
      if (!assigned) return a;
    }
  }
  return 0;
}

void autoAssignAddress() {
  Wire.beginTransmission(FACTORY_ADDR);
  if (Wire.endTransmission() == 0) {
    uint8_t newAddr = findFreeAddress();
    if (newAddr != 0) {
      Wire.beginTransmission(FACTORY_ADDR);
      Wire.write(0xF0); 
      Wire.write(newAddr); 
      Wire.endTransmission();
      delay(300);
    }
  }
}

void scanDevices() {
  autoAssignAddress();
  for (uint8_t addr = 0x08; addr < 0x7F; addr++) {
    if (addr == FACTORY_ADDR) continue;
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      bool exists = false;
      for(int i=0; i<deviceCount; i++) {
        if(devices[i].addr == addr) {
          devices[i].online = true;
          devices[i].failCount = 0;
          exists = true;
          break;
        }
      }
      if(!exists && deviceCount < MAX_DEVICES) {
        devices[deviceCount++] = {addr, 0, true, 0, 0};
      }
    }
  }
}

void updateStats() {
  for (int i = 0; i < deviceCount; i++) {
    if (!devices[i].online) continue;
    Wire.beginTransmission(devices[i].addr);
    Wire.write(0x03);
    if (Wire.endTransmission() != 0) {
      if (++devices[i].failCount > 3) devices[i].online = false;
      continue;
    }
    if (Wire.requestFrom(devices[i].addr, (uint8_t)3) == 3) {
      devices[i].lastPos = Wire.read();
      devices[i].lastState = Wire.read();
      Wire.read();
      devices[i].failCount = 0;
    }
  }
}

// ================== 控制指令发送逻辑 ==================

void sendPositionToAll(uint8_t pos) {
  for (int i = 0; i < deviceCount; i++) {
    if (devices[i].online) {
      Wire.beginTransmission(devices[i].addr);
      Wire.write(0x01);
      Wire.write(pos);
      Wire.endTransmission();
    }
  }
}

// ================== 图形界面渲染 (M5GFX) ==================

void drawUI() {
  display.startWrite();
  display.fillScreen(TFT_BLACK);
  int listPages = (deviceCount > 0) ? (deviceCount + DEVICES_PER_PAGE - 1) / DEVICES_PER_PAGE : 1;
  int totalPages = listPages + 1; 

  if (currentPage >= totalPages) {
    currentPage = 0;
  }

  if (currentPage == 0) {
    display.setRotation(0);
    display.drawBitmap(0, 0, epd_bitmap_____liner, 135, 240, TFT_WHITE);
    display.setRotation(1);
    
    display.setTextColor(TFT_WHITE);
    display.setTextSize(1);
    display.setCursor(10, 120);
    display.print("Press G12 for Next");
  } else {
    int listIdx = currentPage - 1;
    display.fillRect(0, 0, display.width(), 20, display.color565(40, 40, 40));
    display.setTextColor(TFT_WHITE);
    display.setCursor(5, 4);
    uint8_t onlineTotal = 0;
    for(int i=0; i<deviceCount; i++) if(devices[i].online) onlineTotal++;
    
    // 顶部状态栏增加 EMG 状态显示
    const char* emgStatusStr = (currentState == FIST) ? "FIST " : "RELAX";
    uint32_t emgColor = (currentState == FIST) ? TFT_RED : TFT_GREEN;
    
    display.printf("On:%d | Pg:%d/%d | EMG:", onlineTotal, listIdx + 1, listPages);
    display.setTextColor(emgColor);
    display.print(emgStatusStr);

    int startIdx = listIdx * DEVICES_PER_PAGE;
    for (int i = 0; i < DEVICES_PER_PAGE; i++) {
      int devIdx = startIdx + i;
      if (devIdx >= deviceCount) break;

      int y = 25 + (i * 32);
      I2CDevice &d = devices[devIdx];
      display.drawRoundRect(2, y, display.width()-4, 30, 4, d.online ? TFT_DARKGREY : display.color565(60, 0, 0));
      display.setCursor(8, y + 4);
      if (d.online) {
        uint32_t stateColor = TFT_GREEN;
        const char* stateText = "READY";
        if(d.lastState == 1) { stateColor = TFT_RED; stateText = "STALL"; }
        else if(d.lastState == 2) { stateColor = TFT_YELLOW; stateText = "HOME"; }
        
        display.setTextColor(TFT_CYAN);
        display.printf("ID:0x%02X", d.addr);
        display.setCursor(65, y + 4);
        display.setTextColor(stateColor);
        display.print(stateText);

        display.drawRect(8, y + 18, 140, 6, TFT_WHITE);
        int barWidth = map(d.lastPos, 0, 100, 0, 138);
        display.fillRect(9, y + 19, barWidth, 4, stateColor);
        
        display.setTextColor(TFT_WHITE);
        display.setCursor(120, y + 4);
        display.printf("%3d%%", d.lastPos);
      } else {
        display.setTextColor(TFT_RED);
        display.printf("ID:0x%02X [OFFLINE]", d.addr);
      }
    }
  }
  display.endWrite();
}

// ================== 主程序 ==================

void setup() {
  Serial.begin(115200);
  display.begin();
  if (display.width() < display.height()) display.setRotation(1); 
  display.setEpdMode(epd_mode_t::epd_fastest);
  display.fillScreen(TFT_BLACK);
  
  pinMode(BTN_PREV, INPUT_PULLUP);
  pinMode(BTN_NEXT, INPUT_PULLUP);

  // EMG 初始化
  pinMode(EMG_PIN, INPUT);
  analogReadResolution(12);

  initI2C();
  scanDevices();
}

void loop() {
  // 1. EMG 非阻塞式高频采样 (约每 8ms 执行一次，125Hz)
  if (millis() - lastEmgTime >= 8) {
    lastEmgTime = millis();
    int raw_val = analogRead(EMG_PIN);
    
    // 消抖与状态机逻辑
    if (raw_val > THRESHOLD) {
        if (currentState == RELAX) {
            stateCounter++;
            if (stateCounter >= DEBOUNCE_COUNT) {
                currentState = FIST;
                stateCounter = 0;
            }
        } else {
            stateCounter = 0;
        }
    } else {
        if (currentState == FIST) {
            stateCounter++;
            if (stateCounter >= DEBOUNCE_COUNT) {
                currentState = RELAX;
                stateCounter = 0;
            }
        } else {
            stateCounter = 0;
        }
    }
    
    // 如果状态发生改变，发送控制指令并更新 UI
    if (currentState != lastState) {
        if (currentState == FIST) {
            Serial.println(">>> EMG DETECTED: FIST! Sending 80 to all devices...");
            sendPositionToAll(80); // 握拳发送 80
        } else {
            Serial.println(">>> EMG DETECTED: RELAX! Sending 0 to all devices...");
            sendPositionToAll(0);  // 放松发送 0 归位
        }
        lastState = currentState;
        drawUI(); // 立即刷新 UI 以反映 EMG 状态的变化
    }
  }

  // 2. 按键检测翻页逻辑
  if (digitalRead(BTN_NEXT) == LOW) {
    currentPage++;
    int listPages = (deviceCount > 0) ? (deviceCount + DEVICES_PER_PAGE - 1) / DEVICES_PER_PAGE : 1;
    int totalPages = listPages + 1;
    if (currentPage >= totalPages) currentPage = 0;
    drawUI(); 
    delay(200); // 这里的延时不会影响上述基于 millis 的 EMG 采样
  }
  
  // 3. I2C 总线扫描与状态更新
  if (millis() - lastScanTime > 5000) {
    lastScanTime = millis();
    scanDevices();
  }
  
  if (millis() - lastStatTime > 1000) {
    lastStatTime = millis();
    updateStats();
    drawUI();
  }

  // 4. 串口命令逻辑保持不变
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;
    if (input.equalsIgnoreCase("r")) {
       for(int i=0; i<deviceCount; i++) if(devices[i].online) {
         Wire.beginTransmission(devices[i].addr);
         Wire.write(0x02); Wire.endTransmission();
       }
    } else if (input.indexOf(':') != -1) {
       uint8_t a = input.substring(0, input.indexOf(':')).toInt();
       uint8_t p = input.substring(input.indexOf(':')+1).toInt();
       Wire.beginTransmission(a); Wire.write(0x01); Wire.write(p); Wire.endTransmission();
    } else {
       bool isNum = true;
       for(uint8_t i=0; i<input.length(); i++) {
         if(!isDigit(input[i])) { isNum = false; break; }
       }
       if (isNum) {
          int val = input.toInt();
          if (val >= 0 && val <= 100) {
             sendPositionToAll(val); // 复用统一的下发函数
          }
       }
    }
  }
}