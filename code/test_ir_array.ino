#define MUX_SIG 34
#define S0 16
#define S1 17
#define S2 18
#define S3 19

void setup() {
  Serial.begin(115200);
  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);
  pinMode(MUX_SIG, INPUT);  // optional: สำหรับชัดเจน
}

// ฟังก์ชันเลือกช่อง MUX และอ่านค่า analog
int readMUXChannel(uint8_t channel) {
  digitalWrite(S0, bitRead(channel, 0));
  digitalWrite(S1, bitRead(channel, 1));
  digitalWrite(S2, bitRead(channel, 2));
  digitalWrite(S3, bitRead(channel, 3));
  delayMicroseconds(5);
  return analogRead(MUX_SIG);  // GPIO34 รองรับ analogRead()
}

void loop() {
  for (uint8_t ch = 0; ch < 16; ch++) {
    int val = readMUXChannel(ch);
    Serial.print("CH"); Serial.print(ch); Serial.print(": ");
    Serial.println(val);
  }
  Serial.println("----");
  delay(500);
}
