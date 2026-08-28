/*
  Bagian untuk Main Program
  Ditulis oleh: Aji dan Jans
  Program Studi Teknologi Rekayasa Instrumentasi dan Kontrol
  Departemen Teknik Elektro dan Informatika
  Sekolah Vokasi
  Universitas Gadjah Mada
  2025
*/

#include <Wire.h>
#include <Servo.h>
#include <math.h>

HardwareSerial Serial1(PA10, PA9);

// === Konfigurasi Pin ===
#define ESC1_PIN PA5
#define ESC2_PIN PB9
#define ESC3_PIN PB1
#define ESC4_PIN PA15
#define I2C_SDA  PB7
#define I2C_SCL  PB6
#define BMI160_ADDR 0x68


Servo esc1, esc2, esc3, esc4;

// === Parameter PWM dan Referensi ===
const float PWM_MIN = 1200;
const float PWM_MAX = 1900;
float pwm_base = 1300;
float roll_ref = 0.0;
float pitch_ref = 0.0;
float yaw_ref = 0.0;
float yaw = 0.0;
float roll_cf = 0, pitch_cf = 0;
float alpha_cf = 0.98;
float roll = 0, pitch = 0;
float gx = 0, gy = 0, gz = 0;
float U_phi = 0, U_theta = 0, U_psi = 0;
float roll_raw = 0, pitch_raw = 0;
float dphi_ref = 0, dtheta_ref = 0;
float tau_phi = 0, tau_theta = 0;

// === Parameter SMC
float k1 = 6.2111;
float k2 = 1.7889;
float eps = 600.0;
float I_y = 2.971e-4;
float I_x = 3.039e-4;
float I_z = 5.854e-4;
float l = 0.105;
float force_to_pwm = 30.0; //30

unsigned long last_loop_time = 0;



float constrainPWM(float val) {
  return constrain(val, PWM_MIN, PWM_MAX);
}

void setup() {
  delay(2000);
  Serial1.begin(115200);
  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.setClock(400000);
  Wire.begin(); delay(100);
  initBMI160();

  esc1.attach(ESC1_PIN, 1000, 2000);
  esc2.attach(ESC2_PIN, 1000, 2000);
  esc3.attach(ESC3_PIN, 1000, 2000);
  esc4.attach(ESC4_PIN, 1000, 2000);


  unsigned long start_time = millis();
  while (millis() - start_time < 2000) {
    esc1.writeMicroseconds(2000);
    esc2.writeMicroseconds(2000);
    esc3.writeMicroseconds(2000);
    esc4.writeMicroseconds(2000);
  }

  start_time = millis();
  while (millis() - start_time < 1000) {
    esc1.writeMicroseconds(1000);
    esc2.writeMicroseconds(1000);
    esc3.writeMicroseconds(1000);
    esc4.writeMicroseconds(1000);
  }

  updateRollPitch();
  roll_ref = roll;
  pitch_ref = pitch;

  last_loop_time = millis();
}



void loop() {
  unsigned long now = millis();
  float dt = (now - last_loop_time) / 1000.0;

  if (dt >= 0.004) {
    last_loop_time = now;

    updateRollPitch();
    computeSMCControl();
    sendPWMtoMotors(pwm_base);
	// Serial1.print(roll_raw, 2);
	// Serial1.print(","); Serial1.print(roll, 2);
	// Serial1.print(","); Serial1.print(pitch_raw);
	// Serial1.print(","); Serial1.print(pitch);
	// Serial1.print(","); Serial1.print(dphi_ref);
	// Serial1.print(","); Serial1.print(tau_phi);
	// Serial1.print(","); Serial1.print(dtheta_ref);
	// Serial1.print(","); Serial1.print(tau_theta);
	// Serial1.print(","); Serial1.print(pwm1);
	// Serial1.print(","); Serial1.print(pwm2);
	// Serial1.print(","); Serial1.print(pwm3);
	// Serial1.print(","); Serial1.println(pwm4);
  }
}
