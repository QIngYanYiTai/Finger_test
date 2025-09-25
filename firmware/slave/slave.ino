const int inputPins[8] = {4, 5, 6, 7, 8, 9, 10, 11}; // 编号1-8对应D4-D11
const int outputPinLow = 2;  // D2
const int outputPinHigh = 3; // D3

int targetNumber = 0; // 串口输入的目标编号
unsigned long lastPrintTime = 0; // 上次输出编号的时间

void setup() {
  Serial.begin(9600);

  // 设置输入引脚为INPUT_PULLUP（默认高电平，接地后为低电平）
  for (int i = 0; i < 8; i++) {
    pinMode(inputPins[i], INPUT_PULLUP);
  }

  // 输出引脚
  pinMode(outputPinLow, OUTPUT);
  pinMode(outputPinHigh, OUTPUT);

  digitalWrite(outputPinLow, LOW);
  digitalWrite(outputPinHigh, LOW);
}

void loop() {
  // 读取接地的引脚编号
  int currentNumber = getGroundedPinNumber();

  // 串口读取输入
  if (Serial.available()) {
    char c = Serial.read();
    if (c >= '1' && c <= '8') {
      targetNumber = c - '0'; // 转换为数字1-8
      Serial.print("Target Number Set To: ");
      Serial.println(targetNumber);
    }
  }

  // 只有检测到接地编号时，才更新输出状态
  if (currentNumber != 0 && targetNumber != 0) {
    if (targetNumber < currentNumber) {
      digitalWrite(outputPinLow, HIGH);
      digitalWrite(outputPinHigh, LOW);
    } else if (targetNumber > currentNumber) {
      digitalWrite(outputPinLow, LOW);
      digitalWrite(outputPinHigh, HIGH);
    } else {
      // 相等时都拉低
      digitalWrite(outputPinLow, LOW);
      digitalWrite(outputPinHigh, LOW);
    }
  }
  // else { 不做任何事，保持上一次D2/D3的状态 }

  // 每0.5秒输出一次当前接地编号
  if (millis() - lastPrintTime >= 500) {
    Serial.print("Current Grounded Pin Number: ");
    Serial.println(currentNumber);
    lastPrintTime = millis();
  }

  delay(20); // 稳定检测，短延迟
}

// 返回当前接地的引脚编号（1-8），若无则返回0
int getGroundedPinNumber() {
  for (int i = 0; i < 8; i++) {
    if (digitalRead(inputPins[i]) == LOW) {
      return i + 1; // 编号从1开始
    }
  }
  return 0; // 没有接地
}
