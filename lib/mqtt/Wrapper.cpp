/*
 * The Driver Station Library (LibDS)
 * Copyright (c) Lily Wang and other contributors.
 * Open Source Software; you can modify and/or share it under the terms of
 * the MIT license file in the root directory of this project.
 */

#include "Wrapper.h"
#include "MqttClient.h"
#include <atomic>
#include <memory>
#include <mutex>

extern std::atomic<MqttClient *> g_mqttClient_ptr;

static std::recursive_mutex wrapperMutex;
static std::unique_ptr<MqttClient> g_mqttInstance;

void client_create() {
    std::lock_guard<std::recursive_mutex> lock(wrapperMutex);
    g_mqttInstance = std::make_unique<MqttClient>();
    g_mqttClient_ptr = g_mqttInstance.get();
}

void client_destroy() {
    std::lock_guard<std::recursive_mutex> lock(wrapperMutex);
    if (!g_mqttInstance) {
        return;
    }

    g_mqttClient_ptr.store(nullptr);
    g_mqttInstance.reset();
}

int callbackEx(struct lws *wsi,
               enum lws_callback_reasons reason,
               void *user,
               void *in,
               size_t len) {

    std::lock_guard<std::recursive_mutex> lock(wrapperMutex);
    if (!g_mqttInstance) {
        return 0;
    }
    return g_mqttInstance->callback(wsi, reason, user, in, len);
}

int system_notify_cb(lws_state_manager_t *mgr,
                     lws_state_notify_link_t *link,
                     int current,
                     int target) {
    std::lock_guard<std::recursive_mutex> lock(wrapperMutex);
    if (!g_mqttInstance) {
        return 0;
    }

    return g_mqttInstance->notifyCallback(mgr, link, current, target);
}
