#include <Wire.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>

// Cấu hình
#define PHONE_NUMBER "+84334385064" // Số điện thoại nhận thông báo
#define EMERGENCY_BUTTON_PIN 5      // Chân nút khẩn cấp
const int MPU_addr = 0x68;          // Địa chỉ I2C của MPU-6050

// Đối tượng GPS và Serial
TinyGPSPlus gps;
HardwareSerial sim(2);      // Serial2 cho module SIM (TX: 19, RX: 18)
HardwareSerial gpsSerial(1); // Serial1 cho module GPS (TX: 17, RX: 16)

// Biến toàn cục
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
boolean fall = false, trigger1 = false, trigger2 = false, trigger3 = false;

byte trigger1count = 0, trigger2count = 0, trigger3count = 0;
boolean emergencyButtonPressed = false;
float lastLatitude = 0.0, lastLongitude = 0.0;
unsigned long lastUpdateTime = 0;
unsigned long lastThingSpeakUpdate = 0;
const unsigned long thingSpeakInterval = 10000;  // Cập nhật ThingSpeak mỗi 10 giây

// Cấu hình ThingSpeak
String apn = "v-internet"; // APN của Viettel
String apiKey = "XZQLSUUZVRMP15S6"; // API Key trên ThingSpeak

// Biến để lưu thời gian đã nhấn nút khẩn cấp
unsigned long lastEmergencyPressTime = 0;
const unsigned long emergencyPressCooldown = 5000; // Thời gian làm mới nút nhấn (10 giây)

void setup() {
  Serial.begin(115200);
  sim.begin(115200, SERIAL_8N1, 18, 19);
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);

  pinMode(EMERGENCY_BUTTON_PIN, INPUT_PULLUP);
  Wire.begin();
  initMPU6050();
  initGPRS();
}

void loop() {
  // Cập nhật GPS
  while (gpsSerial.available()) gps.encode(gpsSerial.read());
  if (gps.location.isValid() && gps.location.isUpdated()) {
    lastLatitude = gps.location.lat();
    lastLongitude = gps.location.lng();
} else {
    Serial.println("Không lấy được tọa độ GPS mới, giữ nguyên giá trị cũ");
}



  // Đọc cảm biến và phát hiện ngã
  readSensor();
  processData();

  // Xử lý phát hiện ngã
  if (fall) {
    sendAlert("PHAT HIEN TE NGA: s.net.vn/KodY");
    fall = false;  // Reset flag ngã sau khi xử lý
    lastThingSpeakUpdate = millis();  // Dừng việc gửi ThingSpeak trong thời gian xử lý khẩn cấp
  }

  // Xử lý nút khẩn cấp
  if (digitalRead(EMERGENCY_BUTTON_PIN) == LOW && !emergencyButtonPressed) {
    emergencyButtonPressed = true;
    sendAlert("NHAN NUT KHAN CAP, CAN GIUP DO: s.net.vn/KodY");
    lastThingSpeakUpdate = millis();  // Dừng gửi ThingSpeak trong thời gian xử lý
    lastEmergencyPressTime = millis(); // Lưu lại thời gian nhấn nút
  }

  // Reset lại trạng thái nút khẩn cấp sau thời gian làm mới (cooldown)
  if (emergencyButtonPressed && millis() - lastEmergencyPressTime >= emergencyPressCooldown) {
    emergencyButtonPressed = false;  // Reset lại sau khi hết thời gian cooldown
  }

  // Sau khi xử lý xong (gọi điện hoặc nhắn tin), tiếp tục gửi dữ liệu lên ThingSpeak
  if (millis() - lastThingSpeakUpdate >= thingSpeakInterval && !fall && !emergencyButtonPressed) {
    lastThingSpeakUpdate = millis();
    sendDataToThingSpeak();
  }

  delay(100);
}

// Hàm khởi tạo
void initMPU6050() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0); // Đánh thức MPU-6050
  Wire.endTransmission(true);
}

void initGPRS() {
  sendATCommand("AT", 300, true);
  sendATCommand("AT+CSQ", 300, true);
  sendATCommand("AT+CREG?", 300, true);
  sendATCommand("AT+CGATT=1", 300, true);
  sendATCommand("AT+CGDCONT=1,\"IP\",\"" + apn + "\"", 300, true);
  sendATCommand("AT+CGACT=1,1", 2000, true);
}

// Hàm đọc cảm biến
void readSensor() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);

  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
}

// Hàm xử lý phát hiện ngã (bổ sung đầy đủ logic)
void processData() {
  ax = (AcX - 2050) / 16384.00;
  ay = (AcY - 77) / 16384.00;
  az = (AcZ - 1947) / 16384.00;
  gx = (GyX + 270) / 131.07;
  gy = (GyY - 351) / 131.07;
  gz = (GyZ + 136) / 131.07;

  float Raw_Amp = pow(pow(ax, 2) + pow(ay, 2) + pow(az, 2), 0.5);
  int Amp = Raw_Amp * 10;
  Serial.println(Amp);

  if (Amp <= 4 && !trigger2) { // Nếu gia tốc dưới ngưỡng (0.4g)
    trigger1 = true;
    Serial.println("Kích hoạt TRIGGER 1");
  }
  if (trigger1) {
    trigger1count++;
    if (Amp >= 6) { // Nếu gia tốc vượt quá ngưỡng (3g)
      trigger2 = true;
      Serial.println("Kích hoạt TRIGGER 2");
      trigger1 = false;
      trigger1count = 0;
    }
  }
  if (trigger2) {
    trigger2count++;
    float angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5);
    Serial.println(angleChange);
    if (angleChange >= 30 && angleChange <= 450) { // Nếu thay đổi hướng nằm trong khoảng 30-450 độ
      trigger3 = true;
      trigger2 = false;
      trigger2count = 0;
      Serial.println(angleChange);
      Serial.println("Kích hoạt TRIGGER 3");
    }
  }
  if (trigger3) {
    trigger3count++;
    if (trigger3count >= 5) {
      float angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5);
      Serial.println(angleChange);
      if ((angleChange >= 0) && (angleChange <= 150)) {
        fall = true;
        trigger3 = false;
        trigger3count = 0;
        Serial.println(angleChange);
      } else { // Người dùng lấy lại hướng bình thường
        trigger3 = false;
        trigger3count = 0;
        Serial.println("Hủy kích hoạt TRIGGER 3");
      }
    }
  }
  if (trigger2count >= 6) { // Cho phép 0.5 giây cho việc thay đổi hướng
    trigger2 = false;
    trigger2count = 0;
    Serial.println("Hủy kích hoạt TRIGGER 2");
  }
  if (trigger1count >= 6) { // Cho phép 0.5 giây cho gia tốc vượt quá ngưỡng trên
    trigger1 = false;
    trigger1count = 0;
    Serial.println("Hủy kích hoạt TRIGGER 1");
  }
}

// Hàm gửi thông báo khẩn cấp
void sendAlert(const char* message) {
  Serial.println(message);
  sendSMS(message);
  makeCall();
}

// Hàm gửi SMS
void sendSMS(const char* message) {
  sim.println("AT+CMGF=1");    // Đặt chế độ văn bản cho mô-đun GSM
  delay(500);
  sim.print("AT+CMGS=\"");
  sim.print(PHONE_NUMBER);
  sim.println("\"");
  delay(500);
  sim.print(message);
  delay(500);
  sim.write(26); // Ctrl+Z để gửi SMS
  delay(500);
}

// Hàm thực hiện cuộc gọi
void makeCall() {
  sim.println("AT+CHUP"); 
  sim.print("ATD");
  sim.print(PHONE_NUMBER);
  sim.println(";"); // Bắt đầu cuộc gọi
  delay(10000); // 10 giây
  sim.println("ATH"); // Kết thúc cuộc gọi
  delay(500);
}

// Hàm gửi dữ liệu lên ThingSpeak
void sendDataToThingSpeak() {
  // Xây dựng URL với API key và các trường cần thiết
  String url = "http://api.thingspeak.com/update?api_key=" + apiKey + "&field1=" + String(lastLatitude, 6) + "&field2=" + String(lastLongitude, 6);

  // Khởi tạo HTTP
  sendATCommand("AT+HTTPINIT", 300, true);  // Khởi tạo HTTP
  sendATCommand("AT+HTTPPARA=\"CID\",1", 300, true); // Cấu hình CID (thường là 1)
  sendATCommand("AT+HTTPPARA=\"URL\",\"" + url + "\"", 300, true);  // Cấu hình URL

  // Gửi GET request (HTTP ACTION 0 là GET)
  String response = sendATCommand("AT+HTTPACTION=0", 10000, true);  // 0 cho GET, 1 cho POST

  // Kiểm tra phản hồi
  if (response.indexOf("OK") != -1) {
    Serial.println("Data sent successfully to ThingSpeak");
  } else {
    Serial.println("Error sending data to ThingSpeak");
  }

  // Đọc phản hồi từ server (nếu cần)
  sendATCommand("AT+HTTPREAD", 300, true); 

  // Kết thúc phiên làm việc HTTP
  sendATCommand("AT+HTTPTERM", 300, true);
}


String sendATCommand(String cmd, int timeout, boolean debug) {
  String response = "";
  sim.println(cmd);
  unsigned long previousMillis = millis();
  
  // Đợi phản hồi từ SIM trong thời gian timeout
  while (millis() - previousMillis < timeout) {
    while (sim.available()) {
      char c = sim.read();
      response += c;
    }
  }
  
  // Nếu cần debug, in ra phản hồi
  if (debug) {
    Serial.println(response);
  }
  
  return response;  // Trả về phản hồi dưới dạng String
}

