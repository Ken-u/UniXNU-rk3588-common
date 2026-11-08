#pragma once
// Used once, in the vdpu38x frame-buffer-compression size code, which the H.264 / vdpu34x path never reaches; kept out of the kernel's way as an integer rounding.
static inline float ceilf(float x) {
  int i = (int)x;
  return (float)i + (x > (float)i ? 1.0f : 0.0f);
}
double log(double);   // KernelLibc.c: the kernel has no math library; used by the rate control of the first frame of a re-encode
double log2(double);   // KernelLibc.c: the H.265 register generator's log2_ctu_num
double ceil(double);
