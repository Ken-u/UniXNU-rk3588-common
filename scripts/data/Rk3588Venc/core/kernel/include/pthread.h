#pragma once
// The core runs in one thread, under the lock of whoever calls it: MPP's mutexes and conditions have nothing to do.
#include <stddef.h>
#include <sys/types.h>
#include <time.h>
typedef struct { int unused; } pthread_mutex_t;
typedef struct { int unused; } pthread_mutexattr_t;
typedef struct { int unused; } pthread_cond_t;
typedef struct { int unused; } pthread_condattr_t;
typedef struct { int unused; } pthread_attr_t;
typedef unsigned long pthread_t;
enum { PTHREAD_MUTEX_NORMAL = 0, PTHREAD_MUTEX_RECURSIVE = 1 };
static inline int pthread_mutexattr_init(pthread_mutexattr_t *a) { (void)a; return 0; }
static inline int pthread_mutexattr_settype(pthread_mutexattr_t *a, int t) { (void)a; (void)t; return 0; }
static inline int pthread_mutexattr_destroy(pthread_mutexattr_t *a) { (void)a; return 0; }
static inline int pthread_mutex_init(pthread_mutex_t *m, const pthread_mutexattr_t *a) { (void)m; (void)a; return 0; }
static inline int pthread_mutex_destroy(pthread_mutex_t *m) { (void)m; return 0; }
static inline int pthread_mutex_lock(pthread_mutex_t *m) { (void)m; return 0; }
static inline int pthread_mutex_unlock(pthread_mutex_t *m) { (void)m; return 0; }
static inline int pthread_mutex_trylock(pthread_mutex_t *m) { (void)m; return 0; }
typedef int pthread_once_t;
#define PTHREAD_ONCE_INIT 0
static inline int pthread_once(pthread_once_t *once, void (*routine)(void)) {
  if (!*once) {
    *once = 1;
    routine();
  }
  return 0;
}
