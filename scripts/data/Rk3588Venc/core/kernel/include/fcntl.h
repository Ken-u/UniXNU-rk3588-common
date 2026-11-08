#pragma once
#define F_DUPFD 0
#define F_SETFD 2
#define FD_CLOEXEC 1
#define O_RDONLY 0
#define O_RDWR 2
int fcntl(int, int, ...);
