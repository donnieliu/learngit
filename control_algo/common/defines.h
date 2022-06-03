#ifndef __DEFINE_H__
#define __DEFINE_H__
#include "package.h"

extern const ApiFuncList *g_api;

#define LOG_INFO(fmt, args...) g_api->log_info(g_api->log, fmt, ##args)
#define LOG_WARN(fmt, args...) g_api->log_warn(g_api->log, fmt, ##args)
#define LOG_ERR(fmt, args...)  g_api->log_err(g_api->log, fmt, ##args)
#define PUB(msg_name, msg_ptr) g_api->publish(g_api->node, msg_name, msg_ptr)
#define SUB(msg_name, msg_ptr) g_api->register_sub(g_api->node, msg_name, msg_ptr)
#define TIMER(ms, callback) g_api->register_timer(g_api->node, ms, callback)

#endif
