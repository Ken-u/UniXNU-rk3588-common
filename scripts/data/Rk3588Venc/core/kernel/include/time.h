#pragma once
typedef int clockid_t;
typedef long time_t;
struct timespec {
  time_t tv_sec;
  long tv_nsec;
};
