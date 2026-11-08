// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// Force-included into every file of the encoder core in the kernel build (scripts/build-rk3588-venc.py). MPP sets its modules up from constructors
// (MPP_SINGLETON / MPP_MODULE_ADD, osal/inc/mpp_sgln_base.h): a static function with __attribute__((constructor(priority))) that tells the registry which
// setup function to run. The kernel image runs the constructor section of its own objects when it chooses to, in an order of its own, and a constructor
// priority means nothing on Mach-O. These definitions replace the macros: the function stays, without the attribute and without `static`, so that the
// build script can list them (nm) and write the table venc_core_register_singletons() calls them from, at the moment the embedder wants them to run.
#pragma once
#include "mpp_sgln_base.h"

#undef MPP_SGLN_DEF
#undef MPP_MODULE_DEF
#define MPP_SGLN_DEF(add, id, name, init, deinit) \
    void MPP_SGLN_CTOR(add, name)(void) { \
        MppSglnInfo info = { id, MPP_SGLN_STR(name), init, deinit, }; \
        add(&info, __FUNCTION__); \
    }
#define MPP_MODULE_DEF(add, name, init, deinit) \
    void MPP_SGLN_CTOR(add, name)(void) { \
        MppSglnInfo info = { -1, MPP_SGLN_STR(name), init, deinit, }; \
        add(&info, __FUNCTION__); \
    }
