// Khai báo chân điều khiển động cơ
const int AIN1 = 3; // Chân AIN1
const int AIN2 = 4; // Chân AIN2
const int PWMA = 5; // Chân PWMA (PWM)

// Chân đọc tín hiệu từ Hall sensor
const int HallSensor = 2; // Chân 2 dùng ngắt ngoài

// Biến lưu số xung encoder
volatile long pulseCount = 0;

// Tốc độ động cơ (PWM từ 0 đến 255)
int motorSpeed = 250; // Tốc độ ban đầu

void setup() {
  // Cấu hình chân TB6612
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  // Cấu hình chân Hall sensor
  pinMode(HallSensor, INPUT_PULLUP);

  // Gắn ngắt ngoài để đọc xung từ encoder
  attachInterrupt(digitalPinToInterrupt(HallSensor), countPulse, FALLING);

  // Giao tiếp Serial để hiển thị thông tin
  Serial.begin(115200);
}

void loop() {
  // Điều khiển động cơ (quay theo chiều kim đồng hồ)
  driveMotor(motorSpeed);

  // Hiển thị số xung encoder
  Serial.print("Số xung: ");
  Serial.println(pulseCount);

  delay(1000); // Đợi 1 giây trước khi lặp lại
}

// Hàm ngắt để đếm xung encoder
void countPulse() {
  pulseCount++;
}

// Hàm điều khiển động cơ
void driveMotor(int speed) {
  if (speed > 0) {
    // Quay thuận
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else if (speed < 0) {
    // Quay ngược
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    speed = -speed; // Đảo chiều tốc độ
  } else {
    // Dừng động cơ
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  }

  // Điều chỉnh tốc độ bằng PWM
  analogWrite(PWMA, constrain(speed, 0, 255));
}
