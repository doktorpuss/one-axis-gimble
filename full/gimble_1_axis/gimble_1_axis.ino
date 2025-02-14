#include <MPU6050_tockn.h>
#include <Wire.h>
#include <Fuzzy.h>

//-------------------GYRO ZONE------------------------
MPU6050 mpu6050(Wire);
float prev_ang = 0;
float target_ang = 0;

// calculate differential in angle
float current_ang_diff(){
  mpu6050.update();

  float current = mpu6050.getAngleZ();
  float diff = target_ang - current;

  // Serial.print("Recent angle: ");
  // Serial.print(current);
  // Serial.print("\tAngle_diff: ");
  // Serial.print(diff);

  return diff;
}
//------------------END GYRO ZONE---------------------

//---------------MOTOR INITIATE ZONE------------------
#define PULSE_PER_CYCLE 200
#define MAX_SPEED 255.0
#define MAX_PWM 255

#define ANG_TOLRATE 0

// Khai báo chân điều khiển động cơ
const int AIN1 = 3; // Chân AIN1
const int AIN2 = 4; // Chân AIN2
const int PWMA = 5; // Chân PWMA (PWM)

// Chân đọc tín hiệu từ Hall sensor
const int HallSensor = 2; // Chân 2 dùng ngắt ngoài

// Biến lưu số xung encoder
volatile long pulseCount = 0;

//Biến đánh dấu thời gian
unsigned long t_spd_measure; //chu kỳ đo tốc độ
unsigned long t_motor_run;   //chu kỳ điều khiển động cơ

//Biến lưu tốc độ động cơ
float keyboard_motor_speed=0;

float choose_speed(float ang_diff){
  if(ang_diff > ANG_TOLRATE)          return (50);
  if(ang_diff < (-1.0 * ANG_TOLRATE)) return (-50);
  return 0;
}

void motor_init(){
  // Cấu hình chân TB6612
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  // Cấu hình chân Hall sensor
  pinMode(HallSensor, INPUT_PULLUP);

  // Gắn ngắt ngoài để đọc xung từ encoder
  attachInterrupt(digitalPinToInterrupt(HallSensor), countPulse, FALLING);
}

void spd_measure(){
  float spd_w;
  unsigned long time_diff=millis() - t_spd_measure; // calculate time differential

  t_spd_measure=millis(); //reset time marker 
  spd_w= (360*( (float)(pulseCount) / PULSE_PER_CYCLE ) ) / time_diff; // calculate speed by angle
  pulseCount=0;

  // Serial.print("\tSpeed: ");
  // Serial.print(spd_w*1000);  //present speed (1 second)
  // Serial.print("\tTime diff: ");
  // Serial.print(time_diff);   // present time diff
}

void set_motor_speed(float speed){
  unsigned long time_diff = millis() - t_motor_run;
  t_motor_run = millis();

  int pwm = (int) ((speed / MAX_SPEED) * MAX_PWM);
  driveMotor(pwm);
}

// Hàm ngắt để đếm xung encoder
void countPulse() {
  // driveMotor(0);
  pulseCount++;
}

// Hàm điều khiển động cơ
void driveMotor(int speed) {
  // Serial.print("\tControled speed: ");
  // Serial.print(speed);   
  if (speed > 0) {
    // Quay thuận
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);

  // Điều chỉnh tốc độ bằng PWM
  analogWrite(PWMA, constrain(speed, 0, 255));
  } else if (speed < 0) {
    // Quay ngược
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    speed = -speed; // Đảo chiều tốc độ
    // Serial.print("\tRev controled speed: ");
    // Serial.print(speed);   
    
    // Điều chỉnh tốc độ bằng PWM
    analogWrite(PWMA, constrain(speed, 0, 255));
  } else {
    // Dừng động cơ
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    // Điều chỉnh tốc độ bằng PWM
    analogWrite(PWMA, constrain(speed, 0, 255));
  }

}

void motor_proc(){
  set_motor_speed (keyboard_motor_speed);

  // // Hiển thị số xung encoder
  // Serial.print("\tSố xung: ");
  // Serial.print(pulseCount);

  spd_measure();
}
//------------------END MOTOR ZONE------------------

//-----------------FUZZY LOGIC ZONE-------------------
Fuzzy *fuzzy = new Fuzzy();

void setup_fuzzy_logic() {
  // Input: Angle Difference
  FuzzySet *negError = new FuzzySet(-90, -90, -10, 0);
  FuzzySet *zero = new FuzzySet(-5, 0, 0, 5);
  FuzzySet *posError = new FuzzySet(0, 10, 90, 90);
  FuzzyInput *angleDiff = new FuzzyInput(1);
  angleDiff->addFuzzySet(negError);
  angleDiff->addFuzzySet(zero);
  angleDiff->addFuzzySet(posError);
  fuzzy->addFuzzyInput(angleDiff);

  // Output: Motor Speed
  FuzzySet *reverse = new FuzzySet(-225, -225, -50, 0);
  FuzzySet *stop = new FuzzySet(-25, 0, 0, 25);
  FuzzySet *forward = new FuzzySet(0, 50, 225, 225);
  FuzzyOutput *motorSpeed = new FuzzyOutput(1);
  motorSpeed->addFuzzySet(reverse);
  motorSpeed->addFuzzySet(stop);
  motorSpeed->addFuzzySet(forward);
  fuzzy->addFuzzyOutput(motorSpeed);

  // Rules
  FuzzyRuleAntecedent *ifNegError = new FuzzyRuleAntecedent();
  ifNegError->joinSingle(negError);
  FuzzyRuleConsequent *thenReverse = new FuzzyRuleConsequent();
  thenReverse->addOutput(reverse);
  fuzzy->addFuzzyRule(new FuzzyRule(1, ifNegError, thenReverse));

  FuzzyRuleAntecedent *ifZero = new FuzzyRuleAntecedent();
  ifZero->joinSingle(zero);
  FuzzyRuleConsequent *thenStop = new FuzzyRuleConsequent();
  thenStop->addOutput(stop);
  fuzzy->addFuzzyRule(new FuzzyRule(2, ifZero, thenStop));

  FuzzyRuleAntecedent *ifPosError = new FuzzyRuleAntecedent();
  ifPosError->joinSingle(posError);
  FuzzyRuleConsequent *thenForward = new FuzzyRuleConsequent();
  thenForward->addOutput(forward);
  fuzzy->addFuzzyRule(new FuzzyRule(3, ifPosError, thenForward));
}

float fuzzy_speed_control(float ang_diff) {
  fuzzy->setInput(1, ang_diff);
  fuzzy->fuzzify();
  return fuzzy->defuzzify(1);
}
//----------------END FUZZY LOGIC ZONE----------------

//----------------SERIAL PROCESS ZONE-----------------

#define LOG_CYCLE 500
unsigned long log_marked;
float pi=3.14159265358979323846;

float toRad(float num){
  return (num/360)*(2*pi);
}

void status_log(float ang_diff, float speed){
  if (millis() - log_marked > LOG_CYCLE)
  {
    //print status
    Serial.print("Target angle: ");
    Serial.print(toRad(target_ang));
    Serial.print("rad ;\tCurrent angle: ");
    Serial.print(toRad(mpu6050.getAngleZ()));
    Serial.print("rad ;\tAngle difference: ");
    Serial.print(toRad(ang_diff));
    Serial.print("rad ;\tMotor speed: ");
    Serial.print(-1.0*toRad(speed));
    Serial.println(" rad/s ");  

    //update time mark
    log_marked=millis();
  }
}

void command_listen(){
  if(Serial.available()){
    // String str = Serial.readStringUntil('\n'); // To read string

    // Read Serial into char type buffer
    char buffer[5] ;
    uint8_t size = Serial.readBytesUntil('\n',buffer,5);
    buffer[size]='\0';

    // COMMENT/UNCOMMENT for define input as angle or speed

    // keyboard_motor_speed = atof(buffer); 
    // Serial.print("\tSpeed get from buffer: ");
    // Serial.print(keyboard_motor_speed);   

    target_ang = atof(buffer);
    // Serial.print("\tAngle get from buffer: ");
    // Serial.print(target_ang);   
  }
}
//----------------END SERIAL PROCESS ZONE----------------

void setup() {
  // Giao tiếp Serial để hiển thị thông tin
  Serial.begin(115200);

  motor_init();

  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  mpu6050.update();
  prev_ang = mpu6050.getAngleZ();

  setup_fuzzy_logic();

  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  log_marked=millis();
  // t_spd_measure=millis();
}

void loop() {
  // Điều khiển động cơ (quay theo chiều kim đồng hồ)
  float ang_diff= current_ang_diff();

  //simlple test
  // float speed = choose_speed(ang_diff);

  //fuzzy implement
  float speed = fuzzy_speed_control(ang_diff);

  set_motor_speed(speed);

  command_listen();

  status_log(ang_diff,speed);

  // motor_proc();

  // Serial.println();
  // delay(50);
}

