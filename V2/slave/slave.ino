#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <avr/wdt.h>

// ========================= 配置参数区（快速调参） =========================

// ------------------------- 引脚定义 -------------------------
// MT6826S 磁编码器
#define ENC_CS_PIN     10
#define ENC_MOSI_PIN   11
#define ENC_MISO_PIN   12
#define ENC_SCK_PIN    13

// TMI8180A 电机驱动
#define MOTOR_IN1_PIN  2
#define MOTOR_IN2_PIN  3
#define MOTOR_ISEN_PIN A1

// 限位开关 (低电平触发)
#define LIMIT_SW_PIN   5

// ------------------------- 运行参数 -------------------------
// 电机行程（从零点正转到限位开关的角度，单位：度）
#define FULL_STROKE_DEG 720.0f   // 两圈 = 720°

// 堵转检测阈值 (ADC 原始值，0~1023，根据实际电流采样电阻和电机堵转电流调整)
#define STALL_CURRENT_TH 900

// 位置到达死区（单位：度，误差小于此值即认为已到达目标）
#define POSITION_DEADBAND 2.0f

// 回中时，若上电时限位开关已触发，先反转退出的圈数
#define BACKOFF_TURNS_WHEN_LIMIT_TRIGGERED 1.0f   // 1圈

// 默认 I2C 地址（当 EEPROM 中地址无效时使用）
#define DEFAULT_I2C_ADDR 0x60

// 设备类型 ID（用于主机识别设备类型）
#define DEVICE_TYPE_ID   0x10

// I2C 通信优化参数
#define I2C_TIMEOUT_US     5000UL   // I2C 超时时间（微秒）
#define I2C_RESET_INTERVAL 10000UL  // I2C 健康检查间隔（毫秒）

// ------------------------- 调试开关 -------------------------
// 将此宏改为 1 即可开启串口调试输出，改回 0 关闭调试
#define DEBUG_ENABLE     0

#if DEBUG_ENABLE
  #define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
#endif

// ========================= 全局变量 =========================

// 编码器相关
volatile float rawAngleDeg = 0.0f;       // 当前原始角度 (0~360)
float lastRawAngleDeg = 0.0f;            // 上次读取的原始角度
float totalAngleDeg = 0.0f;              // 累计转过角度（支持多圈）
float zeroOffsetDeg = 0.0f;              // 零点偏移量（使零点对应 totalAngleDeg = 0）

// 电机控制状态
enum MotorDirection { STOP, FORWARD, REVERSE };
volatile MotorDirection currentDir = STOP;

// 位置控制目标 (0~100)
volatile int targetPercent = 0;           // 主机下发的目标位置百分比
float targetAngleDeg = 0.0f;              // 目标绝对角度（基于零点）
bool targetUpdated = false;               // 收到新目标标志

// 状态标志
bool homingCompleted = false;             // 回中是否完成
bool isStalled = false;                   // 堵转标志
bool limitSwitchTriggered = false;        // 限位开关当前状态

// 回中状态机
enum HomingState { 
  HOMING_IDLE, 
  HOMING_CHECK_LIMIT, 
  HOMING_BACKOFF, 
  HOMING_FORWARD_TO_LIMIT, 
  HOMING_REVERSE_TWO_TURNS, 
  HOMING_DONE 
};
HomingState homingState = HOMING_IDLE;
float homingTargetDelta = 0.0f;           // 回中过程中需要转动的目标角度变化量
float homingStartTotalAngle = 0.0f;       // 回中某阶段开始时的累计角度

// I2C 通信相关
volatile uint8_t lastI2CCmd = 0;           // 主机最后写入的命令
volatile uint8_t i2cResponseLen = 0;       // 响应数据长度
volatile uint8_t i2cResponseData[4];       // 响应数据缓冲区（最大4字节）
uint8_t myI2CAddr = DEFAULT_I2C_ADDR;      // 当前使用的 I2C 地址

// I2C 优化：延迟处理变量
volatile bool i2cCommandPending = false;   // 标记有新命令需处理
volatile uint8_t i2cPendingCmd = 0;        // 待处理的命令
volatile uint8_t i2cPendingData = 0;       // 待处理的数据（如目标位置）
volatile bool i2cHasData = false;          // 命令是否携带数据
unsigned long lastI2CHealthCheck = 0;      // 上次 I2C 健康检查时间

// 时间戳（用于非阻塞延时，本设计中未使用延时，保留备用）
unsigned long lastDebugPrint = 0;

#if DEBUG_ENABLE
void handleSerialCommands();
#endif

// ========================= 函数声明 =========================

// 硬件初始化
void initEncoder();
void initMotor();
void initLimitSwitch();
void initI2C(uint8_t addr);
void systemReset();        // 软复位

// 编码器读取
float readRawAngleDeg();   // 返回 0~360 度
void updateTotalAngle();   // 更新累计角度 totalAngleDeg

// 电机控制
void setMotorDirection(MotorDirection dir);
void stopMotor();

// 回中控制
void startHoming();
void processHoming();

// 位置控制
void updatePositionControl();
float getCurrentPercent(); // 获取当前百分比位置 (0~100)

// 堵转检测
bool checkStall();

// I2C 回调
void i2cReceiveEvent(int howMany);
void i2cRequestEvent();

// I2C 优化新增函数
void handleI2CCommand();   // 在主循环中处理 I2C 命令
void checkI2CBusHealth();  // I2C 总线健康监控

// EEPROM 地址管理
void loadI2CAddrFromEEPROM();
void saveI2CAddrToEEPROM(uint8_t addr);

// ========================= 实现部分 =========================

void setup() {
#if DEBUG_ENABLE
  Serial.begin(115200);
  delay(500);
  DEBUG_PRINTLN("\n[INFO] Smart Actuator Module Starting...");
#endif

  // 增加短暂延时，确保从机硬件稳定后再初始化 I2C，使主机扫描时已就绪
  delay(200);

  // 初始化各硬件
  initEncoder();
  initMotor();
  initLimitSwitch();

  // 读取上次保存的 I2C 地址
  loadI2CAddrFromEEPROM();

  // 初始化 I2C 从机
  initI2C(myI2CAddr);

  // 开始回中流程
  startHoming();

  DEBUG_PRINT("[INFO] I2C address: 0x");
  DEBUG_PRINTLN(myI2CAddr, HEX);
  DEBUG_PRINTLN("[INFO] Homing started...");
}

void loop() {
  // 0. 处理 I2C 命令（从中断中延迟处理，避免中断中执行耗时操作）
  handleI2CCommand();

  // 1. 处理回中状态机（未完成时一直执行）
  if (!homingCompleted) {
    processHoming();
  } else {
    // 2. 更新累计角度（基于编码器）
    updateTotalAngle();

    // 3. 处理串口调试命令（仅在调试模式启用时）
#if DEBUG_ENABLE
    handleSerialCommands();
#endif

    // 4. 位置闭环控制
    updatePositionControl();

    // 5. 堵转检测（仅在电机运行时检测）
    if (currentDir != STOP) {
      if (checkStall()) {
        isStalled = true;
        stopMotor();
        DEBUG_PRINTLN("[WARN] Stall detected! Motor stopped.");
      }
    }
  }

  // 6. I2C 总线健康检查（定期检测并复位异常总线）
  checkI2CBusHealth();

  // 可选：周期打印调试信息（每 500ms）
#if DEBUG_ENABLE
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500 && homingCompleted) {
    lastPrint = millis();
    DEBUG_PRINT("Pos%: ");
    DEBUG_PRINT(getCurrentPercent());
    DEBUG_PRINT("  Target: ");
    DEBUG_PRINT(targetPercent);
    DEBUG_PRINT("  Stall: ");
    DEBUG_PRINT(isStalled);
    DEBUG_PRINT("  Angle: ");
    DEBUG_PRINT(totalAngleDeg - zeroOffsetDeg);
    DEBUG_PRINTLN();
  }
#endif

  delay(5);
}

// ======================== 硬件初始化 ========================

void initEncoder() {
  pinMode(ENC_CS_PIN, OUTPUT);
  digitalWrite(ENC_CS_PIN, HIGH);
  SPI.begin();
  // 读取一次原始角度，初始化 lastRawAngleDeg
  rawAngleDeg = readRawAngleDeg();
  lastRawAngleDeg = rawAngleDeg;
  totalAngleDeg = 0.0f;
  DEBUG_PRINTLN("[INFO] MT6826S encoder initialized.");
}

float readRawAngleDeg() {
  // 参考测试代码，读取 21-bit 原始数据并转换为角度
  uint8_t reg03, reg04, reg05;
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(ENC_CS_PIN, LOW);
  
  // 读寄存器 0x03 (高位)
  uint16_t cmd = 0x3000 | 0x03;
  SPI.transfer(highByte(cmd));
  SPI.transfer(lowByte(cmd));
  reg03 = SPI.transfer(0x00);
  
  // 读寄存器 0x04 (中位)
  cmd = 0x3000 | 0x04;
  SPI.transfer(highByte(cmd));
  SPI.transfer(lowByte(cmd));
  reg04 = SPI.transfer(0x00);
  
  // 读寄存器 0x05 (低位)
  cmd = 0x3000 | 0x05;
  SPI.transfer(highByte(cmd));
  SPI.transfer(lowByte(cmd));
  reg05 = SPI.transfer(0x00);
  
  digitalWrite(ENC_CS_PIN, HIGH);
  SPI.endTransaction();
  
  uint32_t rawValue = ((uint32_t)reg03 << 16) | ((uint32_t)reg04 << 8) | reg05;
  rawValue >>= 3;  // 右移3位，得到21位有效数据
  float angleDeg = (float)rawValue * 360.0f / 2097152.0f;  // 2^21 = 2097152
  return angleDeg;
}

void updateTotalAngle() {
  float currentRaw = readRawAngleDeg();
  float delta = currentRaw - lastRawAngleDeg;
  
  // 处理过零点（角度跳变 > 180度 视为方向性过零）
  if (delta > 180.0f) delta -= 360.0f;
  else if (delta < -180.0f) delta += 360.0f;
  
  totalAngleDeg += delta;
  lastRawAngleDeg = currentRaw;
  rawAngleDeg = currentRaw;
}

void initMotor() {
  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);
  stopMotor();
  DEBUG_PRINTLN("[INFO] Motor driver initialized.");
}

void setMotorDirection(MotorDirection dir) {
  currentDir = dir;
  switch (dir) {
    case FORWARD:
      digitalWrite(MOTOR_IN1_PIN, HIGH);
      digitalWrite(MOTOR_IN2_PIN, LOW);
      break;
    case REVERSE:
      digitalWrite(MOTOR_IN1_PIN, LOW);
      digitalWrite(MOTOR_IN2_PIN, HIGH);
      break;
    default:
      digitalWrite(MOTOR_IN1_PIN, LOW);
      digitalWrite(MOTOR_IN2_PIN, LOW);
      break;
  }
}

void stopMotor() {
  setMotorDirection(STOP);
}

void initLimitSwitch() {
  pinMode(LIMIT_SW_PIN, INPUT_PULLUP);  // 内部上拉，开关接地触发低电平
  DEBUG_PRINTLN("[INFO] Limit switch initialized.");
}

bool readLimitSwitch() {
  return (digitalRead(LIMIT_SW_PIN) == LOW);  // 低电平触发
}

// ======================== 回中控制 (状态机) ========================

void startHoming() {
  homingCompleted = false;
  homingState = HOMING_CHECK_LIMIT;
  DEBUG_PRINTLN("[HOMING] Start.");
}

void processHoming() {
  static unsigned long lastStepTime = 0;
  // 为避免频繁更新角度，每次状态机循环都更新一次累计角度
  updateTotalAngle();
  
  switch (homingState) {
    case HOMING_CHECK_LIMIT:
      if (readLimitSwitch()) {
        DEBUG_PRINTLN("[HOMING] Limit switch already triggered, backoff first.");
        homingState = HOMING_BACKOFF;
        homingStartTotalAngle = totalAngleDeg;
        // 需要反转一定圈数（例如一圈）
        homingTargetDelta = -BACKOFF_TURNS_WHEN_LIMIT_TRIGGERED * 360.0f;
        setMotorDirection(REVERSE);
      } else {
        DEBUG_PRINTLN("[HOMING] Limit not triggered, forward to limit.");
        homingState = HOMING_FORWARD_TO_LIMIT;
        homingStartTotalAngle = totalAngleDeg;
        setMotorDirection(FORWARD);
      }
      break;
      
    case HOMING_BACKOFF:
      // 反转直到转过指定角度
      if ((totalAngleDeg - homingStartTotalAngle) <= homingTargetDelta) {
        stopMotor();
        DEBUG_PRINTLN("[HOMING] Backoff complete, now forward to limit.");
        homingState = HOMING_FORWARD_TO_LIMIT;
        homingStartTotalAngle = totalAngleDeg;
        setMotorDirection(FORWARD);
      }
      break;
      
    case HOMING_FORWARD_TO_LIMIT:
      if (readLimitSwitch()) {
        stopMotor();
        DEBUG_PRINTLN("[HOMING] Limit reached, now reverse two turns to zero.");
        homingState = HOMING_REVERSE_TWO_TURNS;
        homingStartTotalAngle = totalAngleDeg;
        homingTargetDelta = -FULL_STROKE_DEG;  // 反转两圈
        setMotorDirection(REVERSE);
      }
      break;
      
    case HOMING_REVERSE_TWO_TURNS:
    {
      float delta = totalAngleDeg - homingStartTotalAngle;
      DEBUG_PRINT("[HOMING] Reverse: delta=");
      DEBUG_PRINT(delta);
      DEBUG_PRINT(" , target=");
      DEBUG_PRINTLN(homingTargetDelta);
      
      // 允许 5° 误差，达到或超过目标即停止
      if (delta <= homingTargetDelta + 2.0f) {
        stopMotor();
        // 设置零点偏移为当前累计角度，使相对角度 = 0
        zeroOffsetDeg = totalAngleDeg;
        // 不要重置 totalAngleDeg，保留累计值以保持连续性
        lastRawAngleDeg = rawAngleDeg;
        homingCompleted = true;
        homingState = HOMING_DONE;
        
        // 将目标位置设置为当前百分比（应为0%），并清除更新标志
        targetPercent = 0;
        targetAngleDeg = 0.0f;
        targetUpdated = false;   // 重要：避免立即执行位置控制
        
        isStalled = false;
        DEBUG_PRINTLN("[HOMING] Completed. Zero point set.");
      }
    }
    break;
      
    default:
      break;
  }
}

// ======================== 位置控制 ========================

float getCurrentPercent() {
  if (!homingCompleted) return 0.0f;
  float relativeAngle = totalAngleDeg - zeroOffsetDeg;
  // 限制范围 [0, FULL_STROKE_DEG]
  if (relativeAngle < 0.0f) relativeAngle = 0.0f;
  if (relativeAngle > FULL_STROKE_DEG) relativeAngle = FULL_STROKE_DEG;
  return (relativeAngle / FULL_STROKE_DEG) * 100.0f;
}

void updatePositionControl() {
  if (!homingCompleted) return;
  
  // 如果收到新目标，重新计算目标角度并清除堵转标志
  if (targetUpdated) {
    targetAngleDeg = (targetPercent / 100.0f) * FULL_STROKE_DEG;
    targetUpdated = false;
    isStalled = false;          // 新指令清除堵转标志
    DEBUG_PRINT("[CTRL] New target: ");
    DEBUG_PRINT(targetPercent);
    DEBUG_PRINT("%, angle: ");
    DEBUG_PRINTLN(targetAngleDeg);
  }
  
  // 如果当前处于堵转状态，不执行运动（等待新指令）
  if (isStalled) {
    stopMotor();
    return;
  }
  
  float currentAngle = totalAngleDeg - zeroOffsetDeg;
  float error = targetAngleDeg - currentAngle;
  
  if (fabs(error) <= POSITION_DEADBAND) {
    stopMotor();
  } else if (error > 0) {
    setMotorDirection(FORWARD);
  } else {
    setMotorDirection(REVERSE);
  }
}

// ======================== 堵转检测 ========================

bool checkStall() {
  // 读取电流检测 ADC 值 (0~1023)
  int adcValue = analogRead(MOTOR_ISEN_PIN);
  if (adcValue >= STALL_CURRENT_TH) {
    return true;
  }
  return false;
}

// ======================== I2C 通信（优化版） ========================

void initI2C(uint8_t addr) {
  Wire.begin(addr);
  Wire.onReceive(i2cReceiveEvent);
  Wire.onRequest(i2cRequestEvent);
  
  // 启用超时功能（Arduino 1.0+ 支持）
  #if defined(WIRE_HAS_TIMEOUT)
    Wire.setWireTimeout(I2C_TIMEOUT_US, true);  // 超时后自动复位
  #endif
  
  DEBUG_PRINT("[I2C] Slave started at address 0x");
  DEBUG_PRINTLN(addr, HEX);
}

// 轻量化 I2C 接收中断：仅保存命令和数据，设置标志
void i2cReceiveEvent(int howMany) {
  if (howMany < 1) return;
  
  uint8_t cmd = Wire.read();
  i2cPendingCmd = cmd;
  i2cHasData = false;
  
  // 对于带有数据的命令，读取第一个数据字节
  if ((cmd == 0x01 || cmd == 0xF0) && Wire.available()) {
    i2cPendingData = Wire.read();
    i2cHasData = true;
  }
  
  // 清空可能剩余的数据
  while (Wire.available()) Wire.read();
  
  // 设置标志，主循环将处理
  i2cCommandPending = true;
}

// 在主循环中处理 I2C 命令（避免中断中执行耗时操作）
void handleI2CCommand() {
  if (!i2cCommandPending) return;
  i2cCommandPending = false;
  
  uint8_t cmd = i2cPendingCmd;
  lastI2CCmd = cmd;
  
  switch (cmd) {
    case 0x55:  // 心跳/握手
      i2cResponseLen = 1;
      i2cResponseData[0] = 0xAA;
      break;
      
    case 0x10:  // 请求设备类型 ID
      i2cResponseLen = 1;
      i2cResponseData[0] = DEVICE_TYPE_ID;
      break;
      
    case 0x03:  // 请求状态（位置+状态+保留）
      {
        uint8_t posPercent = (uint8_t)constrain(getCurrentPercent(), 0, 100);
        uint8_t status = 0;  // 0=正常运行
        if (isStalled) status = 1;      // 堵转受限
        else if (!homingCompleted) status = 2;  // 归零（回中未完成）
        i2cResponseLen = 3;
        i2cResponseData[0] = posPercent;
        i2cResponseData[1] = status;
        i2cResponseData[2] = 0;   // 保留位
      }
      break;
      
    case 0x01:  // 设置目标位置 (0~100)
      if (i2cHasData) {
        uint8_t newTarget = i2cPendingData;
        if (newTarget > 100) newTarget = 100;
        targetPercent = newTarget;
        targetUpdated = true;
        // 新指令清除堵转标志
        isStalled = false;
        DEBUG_PRINT("[I2C] Set target: ");
        DEBUG_PRINTLN(targetPercent);
      }
      i2cResponseLen = 0;
      break;
      
    case 0x02:  // 触发重置校准（重新回中）
      DEBUG_PRINTLN("[I2C] Recalibration triggered.");
      // 停止当前运动
      stopMotor();
      // 重置状态，重新开始回中
      homingCompleted = false;
      isStalled = false;
      startHoming();
      i2cResponseLen = 0;
      break;
      
    case 0xF0:  // 设置新 I2C 地址
      if (i2cHasData) {
        uint8_t newAddr = i2cPendingData;
        if (newAddr >= 0x08 && newAddr <= 0x7F) {
          saveI2CAddrToEEPROM(newAddr);
          DEBUG_PRINT("[I2C] New address saved: 0x");
          DEBUG_PRINTLN(newAddr, HEX);
          DEBUG_PRINTLN("[I2C] Resetting system...");
          delay(50);
          systemReset();
        }
      }
      i2cResponseLen = 0;
      break;
      
    default:
      i2cResponseLen = 1;
      i2cResponseData[0] = 0xFF;
      break;
  }
}

void i2cRequestEvent() {
  // 复制响应数据到本地缓冲区（避免 volatile 类型不匹配）
  uint8_t localData[4];
  uint8_t localLen = i2cResponseLen;
  
  if (localLen > 0) {
    for (uint8_t i = 0; i < localLen; i++) {
      localData[i] = i2cResponseData[i];
    }
    Wire.write(localData, localLen);
    i2cResponseLen = 0;  // 发送后清空
  } else {
    Wire.write(0x00);    // 默认返回0
  }
}

// I2C 总线健康监控与自动恢复
void checkI2CBusHealth() {
  // 定期检查（避免频繁操作）
  if (millis() - lastI2CHealthCheck < I2C_RESET_INTERVAL) return;
  lastI2CHealthCheck = millis();
  
  #if defined(WIRE_HAS_TIMEOUT)
    // 检查是否发生过超时错误
    if (Wire.getWireTimeoutFlag()) {
      DEBUG_PRINTLN("[I2C] Timeout detected, resetting I2C hardware...");
      Wire.clearWireTimeoutFlag();
      
      // 重新初始化 I2C 硬件
      Wire.end();
      delay(10);
      Wire.begin(myI2CAddr);
      Wire.onReceive(i2cReceiveEvent);
      Wire.onRequest(i2cRequestEvent);
      #if defined(WIRE_HAS_TIMEOUT)
        Wire.setWireTimeout(I2C_TIMEOUT_US, true);
      #endif
      DEBUG_PRINTLN("[I2C] I2C hardware reset completed.");
    }
  #else
    // 对于不支持超时的版本，可通过其他方式检测（此处保留扩展接口）
  #endif
}

// ======================== EEPROM 地址管理（增强健壮性） ========================

void loadI2CAddrFromEEPROM() {
  uint8_t addr = EEPROM.read(0);
  // 严格检查地址有效性（避免 0xFF 或保留地址）
  if (addr >= 0x08 && addr <= 0x77 && addr != 0x7F) {  // 排除一些特殊保留地址
    myI2CAddr = addr;
    DEBUG_PRINT("[EEPROM] Loaded address: 0x");
    DEBUG_PRINTLN(myI2CAddr, HEX);
  } else {
    myI2CAddr = DEFAULT_I2C_ADDR;
    DEBUG_PRINT("[EEPROM] Invalid address (0x");
    DEBUG_PRINT(addr, HEX);
    DEBUG_PRINTLN("), using default.");
    // 将有效默认地址写入 EEPROM，避免下次仍异常
    saveI2CAddrToEEPROM(myI2CAddr);
  }
}

void saveI2CAddrToEEPROM(uint8_t addr) {
  EEPROM.update(0, addr);  // update 仅在值不同时写入，延长寿命
  myI2CAddr = addr;
}

// ======================== 系统软复位 ========================

void systemReset() {
  // 使用看门狗复位
  wdt_enable(WDTO_15MS);
  while (1) {
    // 等待看门狗超时复位
  }
}

#if DEBUG_ENABLE
void handleSerialCommands() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;
    
    // 尝试将输入转换为整数 (0-100)
    int cmdPercent = input.toInt();
    // 检查是否为有效数字且范围正确
    if (cmdPercent >= 0 && cmdPercent <= 100) {
      // 设置目标位置（与 I2C 命令 0x03 逻辑一致）
      targetPercent = cmdPercent;
      targetUpdated = true;
      // 清除堵转标志，重新开始运动
      isStalled = false;
      DEBUG_PRINT("[SERIAL] Set target to ");
      DEBUG_PRINT(cmdPercent);
      DEBUG_PRINTLN("%");
    } else {
      DEBUG_PRINT("[SERIAL] Invalid input: ");
      DEBUG_PRINTLN(input);
      DEBUG_PRINTLN("Please enter a number between 0 and 100.");
    }
    // 清空缓冲区中可能剩余的数据
    while (Serial.available()) Serial.read();
  }
}
#endif