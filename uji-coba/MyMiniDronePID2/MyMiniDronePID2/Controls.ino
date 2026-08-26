
/*
  Bagian untuk kontrol 
  Ditulis oleh: Ajeng dan Jans
  Program Studi Teknologi Rekayasa Instrumentasi dan Kontrol
  Departemen Teknik Elektro dan Informatika
  Sekolah Vokasi
  Universitas Gadjah Mada
  2024
*/

void PIDtrik(float galat, float Kp, float Ki, float Kd, float galatSebelumnya, float Isebelumnya)
{
  float T = 0.004;
  float P = Kp * galat;   // kontrol proporsional
  float I = Isebelumnya + 0.5 * Ki * T * (galat + galatSebelumnya); // kontrol integrator

  // anti wind-up pada kontrol integrator
  if (I > imax)
  {
    I = imax;
  }
  else if (I < imin)
  {
    I = imin;
  }

  float D = -(2 * Kd * (galat - galatSebelumnya) + (2 * tau - T) * D) / (2 * tau + T);

  // output dari PID
  float u = P + I + D;

  // batasi nilai kontrol PID
  if (u > bts_pid_max)
  {
    u = bts_pid_max;
  }
  else if (u < bts_pid_min)
  {
    u = bts_pid_min;
  }

  PIDReturn[0] = u;
  PIDReturn[1] = galat;
  PIDReturn[2] = I;
}

