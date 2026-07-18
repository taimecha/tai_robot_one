#include <Arduino.h>

// Hầu hết các board ESP32 mặc định dùng chân 2 cho LED tích hợp
#define LED_PIN 2 

void setup() {
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_PIN, HIGH);   // Bật đèn
  delay(500);                    // Đợi 0.5 giây
  digitalWrite(LED_PIN, LOW);    // Tắt đèn
  delay(500);                    // Đợi 0.5 giây
}