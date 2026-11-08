#pragma once
// MPP's encoder start-up and command path wait for its own thread. The core has no thread: every one of these waits is on a path that is not taken (the
// host test, which uses real semaphores, would hang on one); a wait that returns at once is what the single-threaded loop of VencCore.c would want anyway.
typedef struct { int unused; } sem_t;
static inline int sem_init(sem_t *s, int shared, unsigned int value) { (void)s; (void)shared; (void)value; return 0; }
static inline int sem_destroy(sem_t *s) { (void)s; return 0; }
static inline int sem_wait(sem_t *s) { (void)s; return 0; }
static inline int sem_post(sem_t *s) { (void)s; return 0; }
