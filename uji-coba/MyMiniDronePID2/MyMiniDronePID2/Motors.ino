
/*
  Bagian untuk driving motor pada quad copter
  Ditulis oleh: Ajeng dan Jans
  Program Studi Teknologi Rekayasa Instrumentasi dan Kontrol
  Departemen Teknik Elektro dan Informatika
  Sekolah Vokasi
  Universitas Gadjah Mada
  2024
*/

void driveMotors() {
  in1 = 1.024 * (InputThrottle + InputRoll + InputPitch );
  in2 = 1.024 * (InputThrottle - InputRoll + InputPitch );
  in3 = 1.024 * (InputThrottle - InputRoll - InputPitch );
  in4 = 1.024 * (InputThrottle + InputRoll - InputPitch );

  w1 = map(in1, 0, 2500, 0, 255);
  w2 = map(in2, 0, 2500, 0, 255);
  w3 = map(in3, 0, 2500, 0, 255);
  w4 = map(in4, 0, 2500, 0, 255);

  if (w1 > 255) w1 = 255;
  if (w2 > 255) w2 = 255;
  if (w3 > 255) w3 = 255;
  if (w4 > 255) w4 = 255;

   analogWrite(motor1, w1);
   analogWrite(motor2, w2);
   analogWrite(motor3, w3);
   analogWrite(motor4, w4);
}