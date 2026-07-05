/*
 * The Driver Station Library (LibDS)
 * Copyright (c) Lily Wang and other contributors.
 * Open Source Software; you can modify and/or share it under the terms of
 * the MIT license file in the root directory of this project.
 */

#pragma once

#include "libwebsockets.h"

#ifdef __cplusplus
extern "C" {
#endif

struct wrapper_str;
void client_create();
void client_destroy();

int callbackEx(struct lws *wsi,
               enum lws_callback_reasons reason,
               void *user,
               void *in,
               size_t len);

int system_notify_cb(lws_state_manager_t *mgr,
                     lws_state_notify_link_t *link,
                     int current,
                     int target);
#ifdef __cplusplus
}
#endif
