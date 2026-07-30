#include <Arduino.h>

#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif

/*
  ESP32-C3 + MG90S 单舵机串口角度测试

  接线：
    ESP32-C3 GPIO4 -> MG90S 信号线
    外部 5V       -> MG90S 红线
    外部 GND      -> MG90S 棕色/黑色线
    ESP32-C3 GND  -> 外部电源 GND

  串口：
    波特率：115200

  命令：
    A0
    A30
    A90
    A180

  也兼容：
    A=90

  注意：
    这里的角度是程序按照脉宽换算出的“认为角度”，
    不一定等于舵机实际机械角度。
*/

// ================= 参数区 =================

// MG90S PWM 信号输出引脚
const int SERVO_PWM_PIN = 4;

// Arduino-ESP32 2.x 使用的 LEDC 通道
const int SERVO_LEDC_CHANNEL = 0;

// 舵机 PWM 频率
const int SERVO_PWM_FREQ = 50;

// PWM 分辨率
const int SERVO_PWM_RES_BITS = 14;
const uint32_t SERVO_PWM_MAX =
    (1UL << SERVO_PWM_RES_BITS) - 1;

// 角度映射的脉宽范围
// 后续可通过修改这两个值校准实际角度
const float SERVO_MIN_US = 500.0;
const float SERVO_MAX_US = 2500.0;

// 上电后的初始角度
const float SERVO_BOOT_ANGLE = 30.0;

// 串口没有换行符时，停止接收多久后自动执行
const unsigned long SERIAL_IDLE_TIMEOUT_MS = 120;

// =========================================

String serialBuffer;
unsigned long lastSerialReceiveTime = 0;
float currentAngle = SERVO_BOOT_ANGLE;

/**
 * 向 LEDC 写入 PWM 占空比。
 * 同时兼容 Arduino-ESP32 2.x 和 3.x。
 */
void writeLedcDuty(uint32_t duty)
{
#if defined(ESP_ARDUINO_VERSION_MAJOR) && \
    ESP_ARDUINO_VERSION_MAJOR >= 3

  // Arduino-ESP32 3.x：第一个参数是 GPIO 引脚
  ledcWrite(SERVO_PWM_PIN, duty);

#else

  // Arduino-ESP32 2.x：第一个参数是 LEDC 通道
  ledcWrite(SERVO_LEDC_CHANNEL, duty);

#endif
}

/**
 * 初始化舵机 PWM。
 */
bool beginServoPwm()
{
#if defined(ESP_ARDUINO_VERSION_MAJOR) && \
    ESP_ARDUINO_VERSION_MAJOR >= 3

  return ledcAttach(
      SERVO_PWM_PIN,
      SERVO_PWM_FREQ,
      SERVO_PWM_RES_BITS
  );

#else

  ledcSetup(
      SERVO_LEDC_CHANNEL,
      SERVO_PWM_FREQ,
      SERVO_PWM_RES_BITS
  );

  ledcAttachPin(
      SERVO_PWM_PIN,
      SERVO_LEDC_CHANNEL
  );

  return true;

#endif
}

/**
 * 把角度转换为舵机控制脉宽。
 */
float angleToPulseUs(float angle)
{
  angle = constrain(angle, 0.0f, 180.0f);

  return SERVO_MIN_US +
         angle * (SERVO_MAX_US - SERVO_MIN_US) / 180.0f;
}

/**
 * 输出指定高电平脉宽。
 */
void writeServoPulseUs(float pulseUs)
{
  // 50Hz 对应一个周期 20000us
  const float periodUs = 1000000.0f / SERVO_PWM_FREQ;

  pulseUs = constrain(
      pulseUs,
      SERVO_MIN_US,
      SERVO_MAX_US
  );

  float dutyRatio = pulseUs / periodUs;

  uint32_t duty = static_cast<uint32_t>(
      dutyRatio * SERVO_PWM_MAX
  );

  writeLedcDuty(duty);
}

/**
 * 设置舵机角度。
 */
void setServoAngle(float angle)
{
  angle = constrain(angle, 0.0f, 180.0f);

  float pulseUs = angleToPulseUs(angle);

  writeServoPulseUs(pulseUs);
  currentAngle = angle;

  Serial.print("OK A=");
  Serial.print(angle, 1);
  Serial.print(" DEG, PULSE=");
  Serial.print(pulseUs, 1);
  Serial.println(" us");
}

/**
 * 判断字符串是否是有效数字。
 */
bool isValidNumber(const String &text)
{
  if (text.length() == 0) {
    return false;
  }

  bool hasDigit = false;
  bool hasDecimalPoint = false;

  for (unsigned int i = 0; i < text.length(); i++) {
    char c = text.charAt(i);

    if (c >= '0' && c <= '9') {
      hasDigit = true;
    }
    else if (c == '.' && !hasDecimalPoint) {
      hasDecimalPoint = true;
    }
    else if (c == '-' && i == 0) {
      // 允许负数进入，后续检查范围并报错
    }
    else {
      return false;
    }
  }

  return hasDigit;
}

/**
 * 处理串口命令。
 */
void handleCommand(String command)
{
  command.trim();
  command.toUpperCase();

  if (command.length() == 0) {
    return;
  }

  // 只接受以 A 开头的命令
  if (!command.startsWith("A")) {
    Serial.println("ERR: COMMAND FORMAT SHOULD BE AX");
    return;
  }

  // 删除开头的 A
  String angleText = command.substring(1);
  angleText.trim();

  // 同时兼容 A90 和 A=90
  if (angleText.startsWith("=")) {
    angleText.remove(0, 1);
    angleText.trim();
  }

  if (!isValidNumber(angleText)) {
    Serial.println("ERR: INVALID ANGLE");
    return;
  }

  float angle = angleText.toFloat();

  if (angle < 0.0f || angle > 180.0f) {
    Serial.println("ERR: ANGLE RANGE IS 0-180");
    return;
  }

  setServoAngle(angle);
}

/**
 * 读取串口。
 *
 * 支持：
 * 1. 接收到换行符后立即执行；
 * 2. 没有换行符时，停止输入 120ms 后执行。
 */
void readSerialCommand()
{
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());

    lastSerialReceiveTime = millis();

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      String command = serialBuffer;
      serialBuffer = "";

      handleCommand(command);
      continue;
    }

    serialBuffer += c;

    if (serialBuffer.length() > 32) {
      serialBuffer = "";
      Serial.println("ERR: COMMAND TOO LONG");
    }
  }

  // 兼容串口监视器选择“No line ending”
  if (serialBuffer.length() > 0 &&
      millis() - lastSerialReceiveTime >=
          SERIAL_IDLE_TIMEOUT_MS) {

    String command = serialBuffer;
    serialBuffer = "";

    handleCommand(command);
  }
}

void setup()
{
  Serial.begin(115200);
  delay(800);

  Serial.println();
  Serial.println("ESP32-C3 MG90S TEST");
  Serial.print("PWM GPIO: ");
  Serial.println(SERVO_PWM_PIN);

  if (!beginServoPwm()) {
    Serial.println("ERR: LEDC ATTACH FAILED");

    while (true) {
      delay(1000);
    }
  }

  setServoAngle(SERVO_BOOT_ANGLE);

  Serial.println("READY");
  Serial.println("SEND: A0 ~ A180");
  Serial.println("EXAMPLE: A90");
}

void loop()
{
  readSerialCommand();
}