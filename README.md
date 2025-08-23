# Arduino Nano ATmega328 Line Following Robot

โปรเจคหุ่นยนต์ตามเส้นที่ใช้ Arduino Nano ATmega328 พร้อมระบบ PID Controller และ Multiplexer สำหรับการรองรับเซนเซอร์หลายตัว

## 🤖 คุณสมบัติหลัก (Key Features)

- **Arduino Nano ATmega328** - ใช้ไมโครคอนโทรลเลอร์ที่มีประสิทธิภาพและขนาดกะทัดรัด
- **PID Controller** - ระบบควบคุมแบบ PID (Proportional-Integral-Derivative) สำหรับความแม่นยำสูง
- **Multiplexer Support** - รองรับเซนเซอร์หลายตัวผ่าน MUX IC
- **Auto Calibration** - ระบบปรับเทียบเซนเซอร์อัตโนมัติ
- **Dual Color Line Support** - รองรับเส้นสีดำและสีขาว
- **Motor Control** - ควบคุมมอเตอร์ DC 2 ตัวแบบ PWM
- **LED Status Indicator** - ไฟ LED แสดงสถานะการทำงาน

## 🔧 ฮาร์ดแวร์ที่ใช้ (Hardware Requirements)

### บอร์ดควบคุม
- **Arduino Nano ATmega328P**
- Power Supply 7-12V DC

### เซนเซอร์และอุปกรณ์
- **Line Sensor Array** (รองรับได้ถึง 20 เซนเซอร์)
- **Analog Multiplexer** (สำหรับขยายช่อง analog input)
- **DC Motor** 2 ตัว + Motor Driver
- **Push Button** (SW1) + LED Status
- **Resistor และ Capacitor** ตามวงจร

### การเชื่อมต่อขา (Pin Configuration)

```
Motor Control:
├── Motor 1A: Pin 5 (PWM)
├── Motor 1B: Pin 6 (PWM)
├── Motor 2A: Pin 9 (PWM)
└── Motor 2B: Pin 10 (PWM)

Sensor & Control:
├── Sensor Input: A0 (Analog)
├── MUX Control: A1, A2, A3 (Digital Output)
├── Button SW1: Pin 12 (Input + Pull-up)
└── Status LED: Pin 13 (Output)

Multiplexer Channels:
├── Channel 0-7: Sensor Array
└── Control Logic: Binary Selection
```

## 📁 โครงสร้างไฟล์ (File Structure)

```
Arduino_Nano_Line_Robot/
├── Exam_Nano_ATmega328.ino    # โค้ดหลัก (Main Program)
├── Nano_Atmega.h              # ไลบรารีควบคุม (Control Library)
└── README.md                  # เอกสารนี้
```

## 🚀 การติดตั้งและใช้งาน (Installation & Setup)

### 1. เตรียม Arduino IDE
```bash
# ตรวจสอบ Board Manager
- Arduino Nano
- ATmega328P (Old Bootloader/New Bootloader)
- Port: ระบุพอร์ต COM ที่เชื่อมต่อ
```

### 2. อัพโหลดโค้ด
```cpp
#include "Nano_Atmega.h"

void setup() {
  Nano_RP2();                   // เริ่มต้นระบบ
  setFrontLineColor(1);         // 1=เส้นดำ, 0=เส้นขาว
  Serial.begin(115200);         // Serial Communication
  
  // กำหนดเซนเซอร์ 8 ตัว (ช่อง 0-7)
  setSensorPins((const int[]) {0,1,2,3,4,5,6,7}, 8);
  
  // ปรับเทียบเซนเซอร์ 2000 รอบ
  setCalibrate(2000);
  
  // กำหนดค่าสูงสุด-ต่ำสุดของเซนเซอร์
  setSensorMax((const int[]) {1000,1000,1000,1000,1000,1000,1000,1000});
  setSensorMin((const int[]) {300,400,400,400,450,500,400,350});
  
  wait_SW1();                   // รอกดปุ่มเริ่มต้น
}

void loop() {
  // PID Control: (ความเร็ว, Kp, Ki, Kd)
  lineFollow_PID(20, 0.25, 0, 4);
}
```

### 3. ขั้นตอนการปรับเทียบ

1. **วางหุ่นยนต์บนเส้นทาง** - วางให้เซนเซอร์กลางอยู่บนเส้น
2. **เริ่มโปรแกรม** - LED จะกะพริบรอการกดปุ่ม
3. **กดปุ่ม SW1** - เริ่มกระบวนการปรับเทียบ
4. **เลื่อนหุ่นยนต์** - เลื่อนไปมาข้ามเส้นดำและพื้นขาวช้าๆ
5. **รอการปรับเทียบเสร็จ** - ระบบจะปรับเทียบ 2000 รอบ
6. **กดปุ่มอีกครั้ง** - เริ่มการทำงานจริง

## ⚙️ การปรับตั้งค่า PID (PID Tuning)

### ฟังก์ชัน PID
```cpp
lineFollow_PID(speed, Kp, Ki, Kd);
```

### พารามิเตอร์
- **Speed (0-100)**: ความเร็วพื้นฐาน (% PWM)
- **Kp (0.1-2.0)**: Proportional Gain - การตอบสนองต่อ error
- **Ki (0.0-0.5)**: Integral Gain - การแก้ไข steady-state error  
- **Kd (1-20)**: Derivative Gain - การป้องกันการสั่น

### การปรับค่าแนะนำ

```cpp
// เริ่มต้น - ความเร็วต่ำ
lineFollow_PID(15, 0.2, 0, 2);

// ปานกลาง - สมดุล
lineFollow_PID(25, 0.35, 0, 5);

// ความเร็วสูง - การแข่งขัน
lineFollow_PID(40, 0.5, 0.05, 8);

// เส้นโค้งแหลม
lineFollow_PID(20, 0.15, 0, 3);
```

## 🔍 ฟังก์ชันสำคัญ (Key Functions)

### การควบคุมเซนเซอร์
```cpp
setSensorPins(pins[], numSensors);    // กำหนดพินเซนเซอร์
setSensorMin(minValues[]);            // กำหนดค่าต่ำสุด
setSensorMax(maxValues[]);            // กำหนดค่าสูงสุด
setCalibrate(rounds);                 // ปรับเทียบ
setFrontLineColor(color);             // เลือกสีเส้น (0=ขาว, 1=ดำ)
```

### การอ่านข้อมูล
```cpp
ReadLightSensor(channel);             // อ่านค่าเซนเซอร์ (0-100)
readline();                           // อ่านตำแหน่งเส้น
ADC_read(channel);                    // อ่านค่า ADC ผ่าน MUX
```

### การควบคุมมอเตอร์
```cpp
motor(motorNum, speed);               // ควบคุมมอเตอร์ (-100 ถึง 100)
// motorNum: 1=ซ้าย, 2=ขวา
// speed: -100=ถอยเต็มที่, 0=หยุด, 100=หน้าเต็มที่
```

## 🧮 การทำงานของ Multiplexer

### MUX Channel Selection
```
Channel | A3 | A2 | A1 | Sensor
--------|----|----|----|---------
   0    | 1  | 1  | 1  |   S0
   1    | 1  | 1  | 0  |   S1
   2    | 1  | 0  | 1  |   S2
   3    | 1  | 0  | 0  |   S3
   4    | 0  | 1  | 1  |   S4
   5    | 0  | 1  | 0  |   S5
   6    | 0  | 0  | 1  |   S6
   7    | 0  | 0  | 0  |   S7
```

### การอ่านค่าผ่าน MUX
```cpp
int sensorValue = ADC_read(channel);  // channel 0-7
int processedValue = ReadLightSensor(channel); // ค่าที่ปรับแล้ว 0-100
```

## 🛠️ การแก้ไขปัญหา (Troubleshooting)

### ปัญหา: หุ่นยนต์สั่นไปมา
**สาเหตุ:** Kp สูงเกินไป หรือ Kd ต่ำเกินไป
```cpp
// ลด Kp และ เพิ่ม Kd
lineFollow_PID(20, 0.15, 0, 6);  // แทน 0.4, 0, 3
```

### ปัญหา: ตอบสนองช้า หักเลี้ยวช้า
**สาเหตุ:** Kp ต่ำเกินไป
```cpp
// เพิ่ม Kp
lineFollow_PID(20, 0.4, 0, 4);   // แทน 0.2, 0, 4
```

### ปัญหา: มีการเบี่ยงเบนสะสม (Steady-State Error)
**สาเหตุ:** ต้องใช้ Ki
```cpp
// เพิ่ม Ki เล็กน้อย
lineFollow_PID(20, 0.3, 0.02, 4); // เพิ่ม Ki = 0.02
```

### ปัญหา: เซนเซอร์อ่านค่าผิดปกติ
**การแก้ไข:**
1. ตรวจสอบการเชื่อมต่อ MUX (A1, A2, A3)
2. ตรวจสอบ Power Supply เซนเซอร์
3. ปรับเทียบใหม่ใน setCalibrate()
4. ปรับค่า setSensorMin() และ setSensorMax()

### ปัญหา: มอเตอร์ทำงานผิดปกติ
**การแก้ไข:**
1. ตรวจสอบการเชื่อมต่อ Pin 5,6,9,10
2. ตรวจสอบ Motor Driver และ Power Supply
3. ทดสอบด้วยการควบคุมมอเตอร์โดยตรง:
```cpp
motor(1, 50);  delay(1000);  // ทดสอบมอเตอร์ซ้าย
motor(2, 50);  delay(1000);  // ทดสอบมอเตอร์ขวา
```

## 📊 การติดตาม Performance

### Serial Monitor Output
```cpp
void debugSensor() {
  for(int i = 0; i < 8; i++) {
    Serial.print("S");
    Serial.print(i);
    Serial.print(":");
    Serial.print(ReadLightSensor(i));
    Serial.print(" ");
  }
  Serial.print("| Pos:");
  Serial.println(readline());
}
```

### การวัดประสิทธิภาพ
- **ความเร็วเฉลี่ย**: เวลาที่ใช้ในการทำรอบเสร็จ
- **ความแม่นยำ**: จำนวนครั้งที่หลุดเส้นต่อรอบ
- **ความเสถียร**: การสั่นขณะวิ่งตรง

## 🎯 ตัวอย่างการใช้งาน (Usage Examples)

### 1. การใช้งานพื้นฐาน
```cpp
void loop() {
  lineFollow_PID(25, 0.3, 0, 4);  // ความเร็วปานกลาง
}
```

### 2. เปลี่ยนโหมดตามเงื่อนไข
```cpp
void loop() {
  int position = readline();
  
  if(position < 100 || position > 600) {
    // เจอโค้งแหลม - ลดความเร็ว
    lineFollow_PID(15, 0.2, 0, 3);
  } else {
    // เส้นตรง - ความเร็วปกติ
    lineFollow_PID(30, 0.35, 0, 5);
  }
}
```

### 3. โหมดแบบมีการหยุดชั่วคราว
```cpp
void loop() {
  static long timer = millis();
  
  lineFollow_PID(25, 0.3, 0, 4);
  
  // หยุดทุก 10 วินาที เป็นเวลา 1 วินาที
  if(millis() - timer > 10000) {
    motor(1, 0); motor(2, 0);  // หยุด
    delay(1000);               // รอ 1 วินาที
    timer = millis();          // รีเซ็ตตัวจับเวลา
  }
}
```

## 🏁 การปรับแต่งสำหรับการแข่งขัน (Competition Tuning)

### Mode 1: ความเร็วสูง
```cpp
lineFollow_PID(45, 0.6, 0.03, 8);
```

### Mode 2: ความแม่นยำสูง
```cpp
lineFollow_PID(20, 0.25, 0, 5);
```

### Mode 3: สำหรับเส้นที่ซับซ้อน
```cpp
lineFollow_PID(30, 0.4, 0.01, 6);
```

## 📚 เทคนิคขั้นสูง (Advanced Techniques)

### การปรับ PID แบบ Adaptive
```cpp
int adaptivePID() {
  int pos = readline();
  int error = abs(pos - 350);  // ระยะจากจุดกลาง
  
  if(error > 200) {
    // โค้งแหลม - PID แบบช้าแต่แม่นยำ
    lineFollow_PID(20, 0.2, 0, 3);
  } else if(error < 50) {
    // เส้นตรง - PID แบบเร็ว
    lineFollow_PID(40, 0.5, 0, 6);
  } else {
    // ปกติ
    lineFollow_PID(25, 0.35, 0, 4);
  }
}
```

### การจดจำเส้นทาง
```cpp
int pathMemory[100];  // เก็บตำแหน่งเส้นทางได้ 100 จุด
int memoryIndex = 0;

void recordPath() {
  if(memoryIndex < 100) {
    pathMemory[memoryIndex] = readline();
    memoryIndex++;
  }
}
```

## 🔬 การทดสอบและ Debug

### การทดสอบเซนเซอร์
```cpp
void testSensors() {
  Serial.println("=== Sensor Test ===");
  for(int i = 0; i < 8; i++) {
    Serial.print("Raw S");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(ADC_read(i));
    Serial.print(" | Processed: ");
    Serial.println(ReadLightSensor(i));
    delay(200);
  }
}
```

### การทดสอบมอเตอร์
```cpp
void testMotors() {
  Serial.println("=== Motor Test ===");
  
  // ทดสอบมอเตอร์ซ้าย
  motor(1, 50); delay(1000); motor(1, 0);
  Serial.println("Left Motor: OK");
  
  // ทดสอบมอเตอร์ขวา  
  motor(2, 50); delay(1000); motor(2, 0);
  Serial.println("Right Motor: OK");
  
  // ทดสอบการหมุน
  motor(1, 50); motor(2, -50); delay(1000);
  motor(1, 0); motor(2, 0);
  Serial.println("Turn Test: OK");
}
```

## 👨‍💻 ผู้จัดทำ (Developer Info)

- **GitHub:** [@Paponsaeja](https://github.com/Paponsaeja)
- **Project:** Arduino Nano ATmega328 Line Following Robot  
- **Hardware:** Arduino Nano + Custom Sensor Board
- **Version:** 1.0

## 📄 License

โปรเจคนี้สร้างขึ้นเพื่อการศึกษาและการแข่งขันหุ่นยนต์

---
*Arduino Nano ATmega328 Line Following Robot with PID Control System*

## ⚡ Quick Start Guide

1. **อัพโหลดโค้ด** → Arduino IDE
2. **เชื่อมต่อฮาร์ดแวร์** → ตรวจสอบ Pin Connection  
3. **ปรับเทียบเซนเซอร์** → วางบนเส้นทางและกดปุ่ม SW1
4. **ปรับค่า PID** → เริ่มที่ `lineFollow_PID(20, 0.25, 0, 4)`
5. **ทดสอบและปรับแต่ง** → วัดประสิทธิภาพและปรับปรุง
