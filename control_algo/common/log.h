#pragma once

#include "glog/logging.h"
#include "glog/raw_logging.h"

#define FDEBUG VLOG(0) << "[DEBUG] "
#define FINFO LOG(INFO)
#define FWARN LOG(WARNING)
#define FERROR LOG(ERROR)
#define FFATAL LOG(FATAL)

// LOG_IF
#define FINFO_if (cond) LOG_if (INFO, cond)
#define FERROR_if (cond) LOG_if (ERROR, cond)
#define FCHECK(cond) CHECK(cond)

// LOG_EVERY_N
#define FINFO_EVERY(freq) LOG_EVERY_N(INFO, freq)
#define FWARN_EVERY(freq) LOG_EVERY_N(WARNING, freq)
#define FERROR_EVERY(freq) LOG_EVERY_N(ERROR, freq)

#define RETURN_IF_NULL(ptr)               \
    if (ptr == nullptr) {                 \
        FWARN << #ptr << " is nullptr.";  \
        return;                           \
    }

#define RETURN_VAL_IF_NULL(ptr, val)      \
    if (ptr == nullptr) {                 \
        FWARN << #ptr << " is nullptr.";  \
        return val;                       \
    }

#define RETURN_if (condition)                   \
    if (condition) {                           \
        FWARN << #condition << " is not met."; \
        return;                                \
    }

#define RETURN_VAL_if (condition, val)          \
    if (condition) {                           \
        FWARN << #condition << " is not met."; \
        return val;                            \
    }
