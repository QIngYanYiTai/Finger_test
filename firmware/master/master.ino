#include <WiFi.h>
#include <Wire.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>

const char* ssid = "ESP32-Host";
const char* password = "12345678";

AsyncWebServer server(80);
int foundAddress = -1;          // 当前找到的从机地址
int receivedGroundedNumber = -1; // 从机返回的接地引脚编号
unsigned long lastRequestTime = 0;

// 扫描 I2C 设备
void scanI2CDevices() {
  Serial.println("正在扫描 I2C 总线...");
  foundAddress = -1;
  for (int address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      foundAddress = address;
      Serial.print("找到设备，地址: 0x");
      Serial.println(foundAddress, HEX);
      break;
    }
  }
}

// 向从机发送数字
void sendToSlave(int number) {
  if (foundAddress == -1) return;
  Wire.beginTransmission(foundAddress);
  Wire.write(number);
  Wire.endTransmission();
}

// 从从机读取数据
void requestFromSlave() {
  if (foundAddress == -1) return;
  Wire.requestFrom(foundAddress, 1);
  if (Wire.available()) {
    receivedGroundedNumber = Wire.read();
    Serial.print("从机返回接地编号：");
    Serial.println(receivedGroundedNumber);
  }
}

// 热点模式
void setupWiFiAP() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  Serial.print("热点已启动，IP 地址：");
  Serial.println(WiFi.softAPIP());
}

// 网页内容
const char htmlPage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <title>I2C 控制面板</title>
  <style>
    body { font-family: sans-serif; text-align: center; margin-top: 50px; }
    input { font-size: 20px; width: 60px; }
    button { font-size: 20px; padding: 5px 15px; margin: 5px; }
  </style>
</head>
<body>
  <h1>I2C 控制面板</h1>
  <p>输入 1-8 之间的数字：</p>
  <input id="numInput" type="number" min="1" max="8" />
  <button onclick="send()">发送</button>
  <button onclick="rescan()">重新扫描</button>
  <p id="response"></p>
  <p>当前从机接地编号：<span id="grounded">未知</span></p>
  <p>当前从机地址：<span id="slaveAddr">未连接</span></p>

<script>
function send() {
  const val = document.getElementById("numInput").value;
  fetch("/send?num=" + val)
    .then(res => res.text())
    .then(data => document.getElementById("response").innerText = data);
}

function rescan() {
  fetch("/rescan")
    .then(res => res.text())
    .then(data => document.getElementById("response").innerText = data);
}

setInterval(() => {
  fetch("/grounded")
    .then(res => res.text())
    .then(data => document.getElementById("grounded").innerText = data);
  
  fetch("/slave")
    .then(res => res.text())
    .then(data => document.getElementById("slaveAddr").innerText = data);
}, 1000);
</script>
</body>
</html>
)rawliteral";

void setup() {
  Serial.begin(115200);
  Wire.begin(6, 7);  // SDA=GPIO6, SCL=GPIO7（可根据你的接线修改）
  setupWiFiAP();
  scanI2CDevices();

  // 路由：主页
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", htmlPage);
  });

  // 路由：发送数字
  server.on("/send", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("num")) {
      int num = request->getParam("num")->value().toInt();
      if (num >= 1 && num <= 8) {
        sendToSlave(num);
        request->send(200, "text/plain", "已发送: " + String(num));
      } else {
        request->send(400, "text/plain", "请输入 1-8 之间的数字");
      }
    } else {
      request->send(400, "text/plain", "缺少参数");
    }
  });

  // 路由：读取从机返回的接地编号
  server.on("/grounded", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", String(receivedGroundedNumber));
  });

  // 路由：返回当前从机地址
  server.on("/slave", HTTP_GET, [](AsyncWebServerRequest *request){
    if (foundAddress != -1)
      request->send(200, "text/plain", "0x" + String(foundAddress, HEX));
    else
      request->send(200, "text/plain", "未连接");
  });

  // 路由：重新扫描从机
  server.on("/rescan", HTTP_GET, [](AsyncWebServerRequest *request){
    scanI2CDevices();
    if (foundAddress != -1)
      request->send(200, "text/plain", "重新连接成功：0x" + String(foundAddress, HEX));
    else
      request->send(200, "text/plain", "未找到设备");
  });

  server.begin();
}

void loop() {
  if (millis() - lastRequestTime > 1000) {
    lastRequestTime = millis();
    requestFromSlave();
  }
}
