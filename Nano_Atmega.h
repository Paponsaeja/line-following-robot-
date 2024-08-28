#define motor1A  5     // พอร์ตควบคุมมอเตอร์ 1 ขา A
#define motor1B  6     // พอร์ตควบคุมมอเตอร์ 1 ขา B
#define motor2A  9     // พอร์ตควบคุมมอเตอร์ 2 ขา A
#define motor2B  10    // พอร์ตควบคุมมอเตอร์ 2 ขา B

int _sensorPins[20];                                                        // อาร์เรย์เก็บพอร์ตของเซ็นเซอร์
int _NumofSensor = 0;                                                        // จำนวนเซ็นเซอร์
int _min_sensor_values[20];                                                  // อาร์เรย์เก็บค่าต่ำสุดของเซ็นเซอร์
int _max_sensor_values[20];                                                    // อาร์เรย์เก็บค่าสูงสุดของเซ็นเซอร์
int _lastPosition = 0;                                                        // ตำแหน่งล่าสุดที่ตรวจพบเส้น
int _Sensitive  = 20;                                                          // ความไวในการตรวจจับเส้น
int stateOfRunPID = 0;                                                         // สถานะการทำงานของ PID
float  errors = 0, output = 0, integral = 0, derivative = 0, previous_error = 0;  // ตัวแปรที่ใช้ในการคำนวณ PID
uint8_t FrontLineColor = 0;                                                     // สีของเส้นที่ใช้ในการติดตาม (0 = ขาว, 1 = ดำ)
uint8_t BackLineColor = 0;                                                            // สีของพื้นหลัง (ไม่ได้ใช้ในโค้ดนี้)

void Nano_RP2() {   
  pinMode(12,INPUT_PULLUP);                 // ตั้งค่าให้พอร์ต 12 เป็น input แบบ pull-up
  pinMode(13,OUTPUT);                       // ตั้งค่าให้พอร์ต 13 เป็น output
  for (uint8_t i = 0; i < _NumofSensor; i++) {   
    _max_sensor_values[i] =  0;             // รีเซ็ตค่าสูงสุดของเซ็นเซอร์
    _min_sensor_values[i] = 1023;           // รีเซ็ตค่าต่ำสุดของเซ็นเซอร์
  }
}

void wait_SW1(){      // ฟังก์ชันแบบไม่รับค่าภายนอก จะคิดของตัวเองเท่านั้น ง่าย แต่ไม่มีความร่วมมือกับฟังก์ชั่นอื่นๆ จะคำนวณได้แค่ผลลัพเดเียวเท่านั้น เพราะไม่สนใจฟังก์ชันอื่น
  pinMode(12,INPUT_PULLUP);  // ตั้งค่าให้พอร์ต 12 เป็น input แบบ pull-up

  while(digitalRead(12)==1){  // รอจนกว่าปุ่มที่พอร์ต 12 จะถูกกด
    digitalWrite(13,1);       // เปิดไฟ LED ที่พอร์ต 13
    delay(100);             // หน่วงเวลา 100 มิลลิวินาที
    digitalWrite(13,0);     // ปิดไฟ LED ที่พอร์ต 13
    delay(100);              // หน่วงเวลา 100 มิลลิวินาที
  }
}

int ADC_read(int analog_CH) {  
  int val = 0;
  if (analog_CH < 8 ) {                 // ตรวจสอบว่าเป็นช่องอะนาล็อก 0-7 หรือไม่
    pinMode(A1, OUTPUT);          // ตั้งค่าให้พอร์ต A1 เป็น output
    pinMode(A2, OUTPUT);         // ตั้งค่าให้พอร์ต A2 เป็น output
    pinMode(A3, OUTPUT);           // ตั้งค่าให้พอร์ต A3 เป็น output
    int controlPin[] = {27, 28, 29};  // พอร์ตควบคุม MUX
    int muxChannel[8][3] = {{1, 1, 1},{1, 1, 0}, {1, 0, 1}, {1, 0, 0}, {0, 1, 1}, {0, 1, 0}, {0, 0, 1}, {0, 0, 0}};  // ค่าที่ใช้ในการควบคุม MUX
    digitalWrite(A3, muxChannel[analog_CH][0]);  // กำหนดค่าที่พอร์ต A3
    digitalWrite(A2, muxChannel[analog_CH][1]);  // กำหนดค่าที่พอร์ต A2
    digitalWrite(A1, muxChannel[analog_CH][2]);  // กำหนดค่าที่พอร์ต A1
    val = analogRead(A0);                         // อ่านค่าอะนาล็อกที่พอร์ต A0
  }
  return val;  // ส่งค่ากลับ
}

void motor(int pin, int speed_Motor) {  
  if (speed_Motor > 100) speed_Motor = 100;  // จำกัดค่าความเร็วสูงสุดไว้ที่ 100
  if (speed_Motor < -100) speed_Motor = -100;  // จำกัดค่าความเร็วต่ำสุดไว้ที่ -100
  if (pin == 1) {                                    // ควบคุมมอเตอร์ 1
    if (speed_Motor < 0) {  
      speed_Motor = abs(speed_Motor) * 2.55;         // แปลงความเร็วจาก % เป็นค่า PWM (0-255)
      analogWrite(motor1B, 255);                   // หมุนมอเตอร์ถอยหลัง
      analogWrite(motor1A, 255-abs(speed_Motor));   // ปรับความเร็วตามค่า speed_Motor
    }
    else {
      speed_Motor = abs(speed_Motor) * 2.55;         // แปลงความเร็วจาก % เป็นค่า PWM (0-255)
      analogWrite(motor1A, 255);                     // หมุนมอเตอร์เดินหน้า
      analogWrite(motor1B, 255- abs(speed_Motor));  // ปรับความเร็วตามค่า speed_Motor
    }
  }
  else if (pin == 2) {                               // ควบคุมมอเตอร์ 2
    if (speed_Motor < 0) {
      speed_Motor = abs(speed_Motor) * 2.55;        // แปลงความเร็วจาก % เป็นค่า PWM (0-255)
      analogWrite(motor2B, 255);                   // หมุนมอเตอร์ถอยหลัง
      analogWrite(motor2A, 255-abs(speed_Motor));  // ปรับความเร็วตามค่า speed_Motor
    }
    else {
      speed_Motor = abs(speed_Motor) * 2.55;               // แปลงความเร็วจาก % เป็นค่า PWM (0-255)
      analogWrite(motor2A, 255);                           // หมุนมอเตอร์เดินหน้า
      analogWrite(motor2B, 255-abs(speed_Motor));          // ปรับความเร็วตามค่า speed_Motor
    }
  }
}

void setSensorPins(const int * _pins, int _NumofSensor_) {     // ฟังชันแบบรับค่าภายนอกเข้ามาคำนวณ และส่งผลลัพ ออกไป เปลี่ยนแปลงคำตอบได้ตลอดตามค่าที่รับ input เข้ามา _pins , _NumofSensor
  _NumofSensor = _NumofSensor_;            // เก็บจำนวนเซ็นเซอร์
  for (uint8_t i = 0; i < _NumofSensor_; i++) {
    _sensorPins[i] = _pins[i];          // กำหนดพอร์ตของเซ็นเซอร์
    _min_sensor_values[i] = 255;         // รีเซ็ตค่าต่ำสุดของเซ็นเซอร์
    _max_sensor_values[i] = 0;           // รีเซ็ตค่าสูงสุดของเซ็นเซอร์
  }
}

void setSensorMin(const int * _MinSensor) {  
  for (uint8_t i = 0; i < _NumofSensor; i++) {
    _min_sensor_values[i] = _MinSensor[i];  // กำหนดค่าต่ำสุดที่อ่านได้จากเซ็นเซอร์
  }
}

void setSensorMax(const int * _MaxSensor) {  
  for (uint8_t i = 0; i < _NumofSensor; i++) {
    _max_sensor_values[i] = _MaxSensor[i];  // กำหนดค่าสูงสุดที่อ่านได้จากเซ็นเซอร์
  }
}

void setSensitive(const uint16_t  _SensorSensitive) {  
  _Sensitive = _SensorSensitive;  // กำหนดความไวในการตรวจจับเส้น
}

void setFrontLineColor(const uint16_t  _setFrontLineColor) {  
  FrontLineColor = _setFrontLineColor;  // กำหนดสีของเส้นที่หุ่นยนต์ต้องติดตาม (1=ดำ, 0=ขาว)
}

int refSensor(int ch) {  
  return ( _max_sensor_values[ch] + _min_sensor_values[ch] ) / 2 ;  // คำนวณค่ากลางของเซ็นเซอร์
}

int readSensorMinValue(uint8_t _Pin) {  
  return _min_sensor_values[_Pin];  // อ่านค่าต่ำสุดของเซ็นเซอร์ที่พินระบุ
}

int readSensorMaxValue(uint8_t _Pin) {  
  return _max_sensor_values[_Pin];  // อ่านค่าสูงสุดของเซ็นเซอร์ที่พินระบุ
}

int ReadLightSensor(int analog_CH) {  
  int value = 0;
  if(FrontLineColor == 0)value= map(ADC_read(_sensorPins[analog_CH]), _min_sensor_values[analog_CH], _max_sensor_values[analog_CH], 100, 0);          // แปลงค่าเซ็นเซอร์ตามการตั้งค่าสีเส้น (กรณีเส้นสีขาว)
  else if (FrontLineColor == 1) value= map(ADC_read(_sensorPins[analog_CH]), _min_sensor_values[analog_CH], _max_sensor_values[analog_CH], 0, 100);  // แปลงค่าเซ็นเซอร์ตามการตั้งค่าสีเส้น (กรณีเส้นสีดำ)
  if(value < 0)value = 0;          // ถ้าค่าที่อ่านได้น้อยกว่า 0 ให้ปรับเป็น 0
  else if(value >100)value = 100;  // ถ้าค่าที่อ่านได้มากกว่า 100 ให้ปรับเป็น 100
  return value;                    // ส่งค่ากลับให้ฟังชันใช้งาน 
}

void setCalibrate(int cal_round) {  
  for (uint8_t i = 0; i < _NumofSensor; i++){
    _max_sensor_values[i] = 0;                // รีเซ็ตค่าสูงสุดของเซ็นเซอร์
    _min_sensor_values[i] = 1023;             // รีเซ็ตค่าต่ำสุดของเซ็นเซอร์
  }
  
  for (int round_count = 0; round_count < cal_round; round_count ++ ) {
    for (uint8_t i = 0; i < _NumofSensor; i++) {
      if (ADC_read(_sensorPins[i]) > _max_sensor_values[i] ) {
        _max_sensor_values[i]  = ADC_read(_sensorPins[i]);               // อัปเดตค่าสูงสุดของเซ็นเซอร์
        if (_max_sensor_values[i] > 1023 )_max_sensor_values[i] = 1023;  // จำกัดค่าที่ 1023
      }
    }
    for (uint8_t i = 0; i < _NumofSensor; i++) {
      if (ADC_read(_sensorPins[i]) < _min_sensor_values[i] ) {
        _min_sensor_values[i] = ADC_read(_sensorPins[i]);            // อัปเดตค่าต่ำสุดของเซ็นเซอร์
        if (_min_sensor_values[i] < 0) _min_sensor_values[i] = 0;  // จำกัดค่าที่ 0
      }
    }
    delay(5);  // หน่วงเวลา 5 มิลลิวินาทีเพื่อรอการอ่านค่าต่อไป
  }
}

int readline() {  
  bool onLine = false;  // ตรวจสอบว่าหุ่นยนต์อยู่บนเส้นหรือไม่
  long avg = 0;         // เก็บค่าเฉลี่ยตำแหน่งของเส้น
  long sum = 0;         // เก็บผลรวมของค่าที่อ่านได้จากเซ็นเซอร์
  for (uint8_t i = 0; i < _NumofSensor; i++) {
    long value = ReadLightSensor(i);         // อ่านค่าแสงจากเซ็นเซอร์
    if (value > _Sensitive) {  
      onLine = true;                         // ถ้าค่าที่อ่านได้มากกว่าความไวที่ตั้งไว้ แสดงว่าหุ่นยนต์อยู่บนเส้น
    }
    if (value > 5) {
      avg += (long)value * (i * 100);  // คำนวณตำแหน่งของเส้น
      sum += value;                    // เก็บผลรวมของค่าที่อ่านได้
    }
  }
  if (!onLine) {                                     // ถ้าหุ่นยนต์ไม่พบเส้น
    if (_lastPosition < (_NumofSensor - 1) * 100 / 2) {
      return 0;                                     // ส่งค่าตำแหน่งที่เป็นจุดเริ่มต้น
    }
    else {
      return (_NumofSensor - 1) * 100;              // ส่งค่าตำแหน่งที่เป็นจุดสิ้นสุด
    }
  }
  _lastPosition = avg / sum;                       // คำนวณตำแหน่งล่าสุดของเส้น
  return _lastPosition;                            // ส่งค่าตำแหน่งล่าสุดของเส้น
}

void lineFollow_PID(int RUN_PID_speed , float RUN_PID_KP, float RUN_PID_KI, float RUN_PID_KD) {  
  int speed_PID = RUN_PID_speed;                                                       // ความเร็วเริ่มต้นที่ใช้ใน PID
  int present_position = readline();                                                  // อ่านตำแหน่งของเส้นปัจจุบัน
  int setpoint = ((_NumofSensor - 1) * 100) / 2;                                      // กำหนดจุดมุ่งหมาย (จุดกึ่งกลางของเส้น)
  errors = present_position - setpoint;                                                 // คำนวณค่า error จากตำแหน่งปัจจุบัน
  if (errors == 0) integral = 0;                                                      // ถ้า error เป็น 0 ให้รีเซ็ตค่า integral
  integral = integral + errors ;                                                        // คำนวณค่า integral ของ error
  derivative = (errors - previous_error) ;                                              // คำนวณค่า derivative ของ error
  output = RUN_PID_KP * errors  + RUN_PID_KI * integral + RUN_PID_KD * derivative;        // คำนวณ output จากค่า KP, KI, KD
  int motorL = constrain(RUN_PID_speed + output, -100, 100);                             // จำกัดค่าความเร็วของมอเตอร์ซ้ายให้อยู่ในช่วง -100 ถึง 100
  int motorR = constrain(RUN_PID_speed - output, -100, 100);                             // จำกัดค่าความเร็วของมอเตอร์ขวาให้อยู่ในช่วง -100 ถึง 100
  motor(1,motorL);                                        // ควบคุมมอเตอร์ซ้าย
  motor(2,motorR);                                        // ควบคุมมอเตอร์ขวา
  previous_error = errors;                                // อัปเดตค่า error ล่าสุดเพื่อใช้ในการคำนวณ derivative รอบถัดไป
} 
