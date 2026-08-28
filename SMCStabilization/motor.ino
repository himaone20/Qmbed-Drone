/*
  Bagian untuk Alokasi Kontrol Motor
  Ditulis oleh: Aji dan Jans
  Program Studi Teknologi Rekayasa Instrumentasi dan Kontrol
  Departemen Teknik Elektro dan Informatika
  Sekolah Vokasi
  Universitas Gadjah Mada
  2025
*/
void sendPWMtoMotors(float base_pwm) {
  float pwm1 = constrainPWM(base_pwm - U_theta - U_psi); // Motor 1 (atas)
  float pwm2 = constrainPWM(base_pwm + U_phi + U_psi);   // Motor 2 (kiri)
  float pwm3 = constrainPWM(base_pwm + U_theta - U_psi); // Motor 3 (bawah)
  float pwm4 = constrainPWM(base_pwm - U_phi + U_psi);   // Motor 4 (kanan)

  esc1.writeMicroseconds(pwm1);
  esc2.writeMicroseconds(pwm2);
  esc3.writeMicroseconds(pwm3);
  esc4.writeMicroseconds(pwm4);

}