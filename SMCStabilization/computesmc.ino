/*
  Bagian untuk Perhitungan SMC
  Ditulis oleh: Aji dan Jans
  Program Studi Teknologi Rekayasa Instrumentasi dan Kontrol
  Departemen Teknik Elektro dan Informatika
  Sekolah Vokasi
  Universitas Gadjah Mada
  2025
*/

void computeSMCControl() {
  // Bagian Roll
  float e_phi = roll_ref - roll;
  float de_phi = -gx;
  dphi_ref = k1 * e_phi;
  float S_phi = -gx + dphi_ref;
  tau_phi = I_x * (k1 * (dphi_ref - gx) + k2 * tanh(S_phi / eps));
  U_phi = (tau_phi / l) * force_to_pwm;

  // Bagian Pitch
  float e_theta = pitch_ref - pitch;
  float de_theta = -gy;
  dtheta_ref = k1 * e_theta;
  float S_theta = -gy + dtheta_ref;
  tau_theta = I_y * (k1 * (dtheta_ref - gy) + k2 * tanh(S_theta / eps));
  U_theta = (tau_theta / l) * force_to_pwm;
  
  // Bagian Yaw
  float e_psi = yaw_ref - yaw;
  float de_psi = -gz;
  float dpsi_ref = k1 * e_psi;
  float S_psi = -gz + dpsi_ref;
  float tau_psi = I_z * (k1 * (dpsi_ref - gz) + k2 * tanh(S_psi / eps));
  U_psi = (tau_psi / l) * 10;

  float delta_max_pwm = 300;
  U_phi = constrain(U_phi, -delta_max_pwm, delta_max_pwm);
  U_theta = constrain(U_theta, -delta_max_pwm, delta_max_pwm);
  U_psi = constrain(U_psi, -delta_max_pwm, delta_max_pwm);
}