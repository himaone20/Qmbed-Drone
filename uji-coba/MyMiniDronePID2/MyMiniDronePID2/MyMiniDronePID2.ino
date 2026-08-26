
/*
  Bagian untuk kendali roll dan pitch pada quadcopter (mini/coreless) 
  Ditulis oleh: Ajeng dan Jans
  Program Studi Teknologi Rekayasa Instrumentasi dan Kontrol
  Departemen Teknik Elektro dan Informatika
  Sekolah Vokasi
  Universitas Gadjah Mada
  2024
*/

#include <Wire.h>
HardwareSerial Serial1(PA_10, PA_9);

// Motor -> ORIGINAL
#define motor1 PB5
#define motor2 PB10
#define motor3 PA15
#define motor4 PB3

// LED indicator
#define LED1 PC13
#define LED2 PA6

float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;

uint32_t LoopTimer;
float DesiredRateRoll = 0, DesiredRatePitch = 0, DesiredRateYaw = 0;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputThrottle = 1500, InputPitch, InputYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
float in1, in2, in3, in4;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
float Kalman1DOutput[] = { 0, 0 };
float DesiredAngleRoll, DesiredAnglePitch;
float ErrorAngleRoll, ErrorAnglePitch;
float PrevErrorAngleRoll, PrevErrorAnglePitch;
float PrevItermAngleRoll, PrevItermAnglePitch;
float w1, w2, w3, w4;

float PIDReturn[] = { 0, 0, 0 };
float PAngleRoll = 8;
float PAnglePitch = PAngleRoll;
float PRateRoll = 2.5;
float PRatePitch = PRateRoll;
float PRateYaw = 2;

float IAngleRoll = 0.05;
float IAnglePitch = IAngleRoll;
float IRateRoll = 0.3; 
float IRatePitch = IRateRoll;
float IRateYaw = 12;

float DAngleRoll = 0.5;
float DAnglePitch = DAngleRoll;
float DRateRoll = 0.06;
float DRatePitch = DRateRoll;
float DRateYaw = 0;

float tau = 2;
float bts_pid_min = -400;
float bts_pid_max = 400;
float imin = -400;
float imax = 400;

void setup() {
  Serial1.begin(9600);

  // Seting pin untuk I2C
  Wire.setSDA(PB9);
  Wire.setSCL(PB6);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  analogWriteFrequency(250);
  analogWriteResolution(8);

  // Hidupkan led indikator
  pinMode(LED1, OUTPUT);
  digitalWrite(LED1, LOW);

  // Hidupkan led indikator 2
  pinMode(LED2, OUTPUT);
  digitalWrite(LED2, HIGH);

  delay(5000);

  LoopTimer = micros();
}

void loop() {
  gyro_signals();

  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll = Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

  ErrorAngleRoll = DesiredAngleRoll - KalmanAngleRoll;
  ErrorAnglePitch = DesiredAnglePitch - KalmanAnglePitch;

  PIDtrik(ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, PrevErrorAngleRoll, PrevItermAngleRoll);
  DesiredRateRoll = PIDReturn[0];
  PrevErrorAngleRoll = PIDReturn[1];
  PrevItermAngleRoll = PIDReturn[2];

  PIDtrik(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorAnglePitch, PrevItermAnglePitch);
  DesiredRatePitch = PIDReturn[0];
  PrevErrorAnglePitch = PIDReturn[1];
  PrevItermAnglePitch = PIDReturn[2];

  // Hitung error kecepatan sudut
  ErrorRateRoll = DesiredRateRoll - RateRoll;
  ErrorRatePitch = DesiredRatePitch - RatePitch;
  ErrorRateYaw = DesiredRateYaw - RateYaw;

  PIDtrik(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
  InputRoll = PIDReturn[0];
  PrevErrorRateRoll = PIDReturn[1];
  PrevItermRateRoll = PIDReturn[2];

  PIDtrik(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
  InputPitch = PIDReturn[0];
  PrevErrorRatePitch = PIDReturn[1];
  PrevItermRatePitch = PIDReturn[2];

  PIDtrik(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
  InputYaw = PIDReturn[0];
  PrevErrorRateYaw = PIDReturn[1];
  PrevItermRateYaw = PIDReturn[2];

  driveMotors();

  Serial1.print(KalmanAngleRoll);
  Serial1.print(" | ");
  Serial1.print(KalmanAnglePitch);
  Serial1.print(" | ");

  // Serial1.print(in1);
  // Serial1.print(" | ");
  // Serial1.print(in2);
  // Serial1.print(" | ");
  // Serial1.print(in3);
  // Serial1.print(" | ");
  // Serial1.print(in4);
  // Serial1.print(" | ");

  Serial1.print(InputRoll);
  Serial1.print(" | ");
  Serial1.print(InputPitch);
  Serial1.print(" | ");
  Serial1.print(InputYaw);
  Serial1.print(" | ");

  Serial1.print(w1);
  Serial1.print(" | ");
  Serial1.print(w2);
  Serial1.print(" | ");
  Serial1.print(w3);
  Serial1.print(" | ");
  Serial1.print(w4);

  Serial1.println(" ");

  while (micros() - LoopTimer < 4000)
    ;
  LoopTimer = micros();
}