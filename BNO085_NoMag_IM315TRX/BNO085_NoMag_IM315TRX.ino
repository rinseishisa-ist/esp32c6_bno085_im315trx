#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <math.h>

// BNO085（READMEのI2C配線）
constexpr int PIN_BNO_SDA = D4;
constexpr int PIN_BNO_SCL = D5;
constexpr uint8_t BNO_I2C_ADDRESS = 0x4A;
constexpr uint32_t BNO_I2C_FREQUENCY_HZ = 400000;

// IM315TRX（READMEの配線）
// BUSY（pin 1）とRESET（pin 25）はESP32に接続せず、BUSYの状態に関係なく送受信する。
constexpr int PIN_IM315_TX = D6;     // ESP32 TX -> IM315TRX pin 6 RxD
constexpr int PIN_IM315_RX = D7;     // ESP32 RX <- IM315TRX pin 7 TxD

// 実機のIM315TRX設定と一致していることを確認すること。
constexpr uint32_t IM315_BAUD = 19200;
constexpr uint32_t RADIO_SEND_INTERVAL_MS = 100;  // 10 Hz
constexpr uint32_t IM315_REPLY_TIMEOUT_MS = 500;

// BNO085のRST端子は現在の配線では使用しない。
Adafruit_BNO08x bno085(-1);
sh2_SensorValue_t sensorValue;
HardwareSerial im315(1);

struct QuaternionSample {
  float w;
  float x;
  float y;
  float z;
  uint32_t timestampMs;
  bool valid;
};

QuaternionSample latestQuaternion{};
bool waitingForReply = false;
uint32_t lastRadioSendMs = 0;
uint32_t replyDeadlineMs = 0;
uint32_t sentCount = 0;
uint32_t radioErrorCount = 0;
String im315Line;

bool startGameRotationVector() {
  // 20000 us = 0.02秒 = 50 Hz
  return bno085.enableReport(SH2_GAME_ROTATION_VECTOR, 20000);
}

bool isValidQuaternion(float w, float x, float y, float z) {
  if (!isfinite(w) || !isfinite(x) || !isfinite(y) || !isfinite(z)) {
    return false;
  }

  const float normSquared = w * w + x * x + y * y + z * z;
  return normSquared > 0.5f && normSquared < 1.5f;
}

// 1成分を符号付きQ14へ変換する。範囲は[-1, +1]、分解能は約0.000061。
int16_t toQ14(float value) {
  value = constrain(value, -1.0f, 1.0f);
  return static_cast<int16_t>(lroundf(value * 16384.0f));
}

void appendHexByte(char *output, size_t &position, uint8_t value) {
  static constexpr char HEX_DIGITS[] = "0123456789ABCDEF";
  output[position++] = HEX_DIGITS[value >> 4];
  output[position++] = HEX_DIGITS[value & 0x0F];
}

// ペイロードはw, x, y, zの順で、各成分は符号付きQ14・上位バイト先頭。
// 8バイトを16文字の16進数へ変換する。
void encodeQuaternion(const QuaternionSample &input, char (&hex)[17]) {
  QuaternionSample q = input;

  // qと-qは同じ回転を表す。wを正側へそろえて符号の不連続を抑える。
  if (q.w < 0.0f) {
    q.w = -q.w;
    q.x = -q.x;
    q.y = -q.y;
    q.z = -q.z;
  }

  const int16_t values[] = {
      toQ14(q.w),
      toQ14(q.x),
      toQ14(q.y),
      toQ14(q.z),
  };

  size_t position = 0;
  for (const int16_t value : values) {
    const uint16_t bits = static_cast<uint16_t>(value);
    appendHexByte(hex, position, static_cast<uint8_t>(bits >> 8));
    appendHexByte(hex, position, static_cast<uint8_t>(bits));
  }
  hex[position] = '\0';
}

void handleIm315Line(const String &line) {
  if (line.isEmpty()) {
    return;
  }

  // Python可視化側がQ行以外を無視できるよう、診断行には接頭辞を付ける。
  Serial.print("IM315:");
  Serial.println(line);

  if (!waitingForReply) {
    return;
  }

  if (line == "OK") {
    waitingForReply = false;
    ++sentCount;
  } else if (line == "NG") {
    waitingForReply = false;
    ++radioErrorCount;
    Serial.println("ERROR:IM315_TX_REJECTED");
  }
}

void serviceIm315Uart() {
  while (im315.available() > 0) {
    const char c = static_cast<char>(im315.read());

    if (c == '\n') {
      im315Line.trim();
      handleIm315Line(im315Line);
      im315Line = "";
    } else if (c != '\r') {
      if (im315Line.length() < 95) {
        im315Line += c;
      } else {
        im315Line = "";
        ++radioErrorCount;
        Serial.println("ERROR:IM315_LINE_TOO_LONG");
      }
    }
  }

  if (waitingForReply &&
      static_cast<int32_t>(millis() - replyDeadlineMs) >= 0) {
    waitingForReply = false;
    ++radioErrorCount;
    Serial.println("ERROR:IM315_REPLY_TIMEOUT");
  }
}

void trySendQuaternion() {
  const uint32_t now = millis();

  if (!latestQuaternion.valid ||
      now - lastRadioSendMs < RADIO_SEND_INTERVAL_MS) {
    return;
  }

  // BUSYは見ずに、応答待ちでも送信間隔ごとに必ず送信する。
  // 前回コマンドの応答が来ていなければ、応答なしとして数える。
  if (waitingForReply) {
    waitingForReply = false;
    ++radioErrorCount;
    Serial.println("ERROR:IM315_REPLY_TIMEOUT");
  }

  char payload[17];
  encodeQuaternion(latestQuaternion, payload);

  // IM315TRXへ送信コマンドを渡す。終端はCRLF。
  im315.print("TXDT ");
  im315.print(payload);
  im315.print("\r\n");

  lastRadioSendMs = now;
  waitingForReply = true;
  replyDeadlineMs = now + IM315_REPLY_TIMEOUT_MS;

  Serial.print("TX:");
  Serial.println(payload);
}

void setup() {
  Serial.begin(115200);

  const uint32_t serialWaitStartMs = millis();
  while (!Serial && millis() - serialWaitStartMs < 5000) {
    delay(10);
  }

  im315.begin(IM315_BAUD, SERIAL_8N1, PIN_IM315_RX, PIN_IM315_TX);

  Wire.begin(PIN_BNO_SDA, PIN_BNO_SCL);
  Wire.setClock(BNO_I2C_FREQUENCY_HZ);

  if (!bno085.begin_I2C(BNO_I2C_ADDRESS, &Wire)) {
    Serial.println("ERROR:BNO085_NOT_FOUND");
    while (true) {
      serviceIm315Uart();
      delay(1000);
    }
  }

  if (!startGameRotationVector()) {
    Serial.println("ERROR:REPORT_START_FAILED");
    while (true) {
      serviceIm315Uart();
      delay(1000);
    }
  }

  Serial.println("READY");
}

void loop() {
  serviceIm315Uart();

  if (bno085.wasReset() && !startGameRotationVector()) {
    Serial.println("ERROR:REPORT_RESTART_FAILED");
  }

  if (bno085.getSensorEvent(&sensorValue) &&
      sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
    const float w = sensorValue.un.gameRotationVector.real;
    const float x = sensorValue.un.gameRotationVector.i;
    const float y = sensorValue.un.gameRotationVector.j;
    const float z = sensorValue.un.gameRotationVector.k;

    if (isValidQuaternion(w, x, y, z)) {
      latestQuaternion = {w, x, y, z, millis(), true};

      // 既存のPython可視化プログラム用出力は維持する。
      Serial.print("Q,");
      Serial.print(w, 6);
      Serial.print(",");
      Serial.print(x, 6);
      Serial.print(",");
      Serial.print(y, 6);
      Serial.print(",");
      Serial.println(z, 6);
    } else {
      Serial.println("ERROR:INVALID_QUATERNION");
    }
  }

  trySendQuaternion();
  delay(1);
}
