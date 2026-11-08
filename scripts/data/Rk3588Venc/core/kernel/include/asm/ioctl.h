#pragma once
// mpp_service.h (the request structures of MPP's kernel service, which the H.265 front end includes for the service's capability word) builds MPP_IOC_CFG_V1 with _IOW:
// the encoding is the one of sys/ioctl.h; nothing issues the request (core/EncShim.c is the service).
#include <sys/ioctl.h>
