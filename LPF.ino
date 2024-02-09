float v_raw;
float LPF_v;
float LPF_Beta = 0.01; // 0<ß<1

/// Low-pass filter
float LPF(float s, float prevS, float deltaT) {
  v_raw = (s-prevS)/deltaT;
  
  LPF_v = LPF_v - (LPF_Beta * (LPF_v - v_raw));

  return LPF_v;
}
