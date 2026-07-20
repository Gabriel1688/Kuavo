#include "MqttClient.h"
#include "Wrapper.h"
#include "common/Config.h"
#include "spdlog/cfg/env.h"
#include "spdlog/fmt/ranges.h"
#include "spdlog/spdlog.h"
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fstream>
#include <functional>
#include <pthread.h>
#include <sched.h>
#include <thread>

using namespace std::placeholders;

std::atomic<class MqttClient *> g_mqttClient_ptr{nullptr};

enum {
    STATE_SUBSCRIBE,    /* subscribe to the topic */
    STATE_PUBLISH_QOS0, /* Send the message in QoS0 */
    STATE_WAIT_ACK0     /* Wait for the synthetic "ack" */
};

// Route all libwebsockets log output through spdlog
static void lws_spdlog_emit(int level, const char *line) {
    // Strip trailing newline/CR that lws appends
    std::string msg(line);
    while (!msg.empty() && (msg.back() == '\n' || msg.back() == '\r'))
        msg.pop_back();

    if (level & LLL_ERR)
        spdlog::error("[lws] {}", msg);
    else if (level & LLL_WARN)
        spdlog::warn("[lws] {}", msg);
    else if (level & (LLL_NOTICE | LLL_USER | LLL_CLIENT))
        spdlog::info("[lws] {}", msg);
    else if (level & LLL_INFO)
        spdlog::info("[lws] {}", msg);
    else if (level & LLL_DEBUG)
        spdlog::debug("[lws] {}", msg);
    else
        spdlog::trace("[lws] {}", msg);
}

MqttClient::MqttClient() {
    m_shutdown = false;
    m_clientId = "MqttClient";
}

void MqttClient::loadConfig(const std::string &fileName) {
    (void) fileName;// config.yaml is loaded via Config singleton
    const auto &mqtt = Config::instance().mqtt();
    m_clientId = mqtt.clientId;
    m_username = mqtt.username;
    m_password = mqtt.password;
    m_address = mqtt.address;
    m_host = mqtt.host;
    m_port = mqtt.port;
    m_topics = mqtt.topics;

    for (const auto &topic : m_topics) {
        SPDLOG_INFO("topic:[{}]", topic);
    }
    SPDLOG_INFO("[{}]:[{}]:[{}]:[{}]:[{}].",
                m_clientId, m_username, m_password, m_address, m_host);
}

void MqttClient::start() {
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, 512 * 1024);
    m_thrCreated = (pthread_create(&m_thrId, &attr, EntryOfThread, this) == 0);
    pthread_attr_destroy(&attr);
    if (!m_thrCreated) {
        SPDLOG_ERROR("Failed to create thread for mqtt Client.");
    } else {
        struct sched_param param{};
        param.sched_priority = 0;
        if (pthread_setschedparam(m_thrId, SCHED_OTHER, &param) != 0) {
            SPDLOG_WARN("MQTT Logger: failed to set SCHED_OTHER/0: {}",
                        strerror(errno));
        } else {
            SPDLOG_INFO("MQTT Logger: SCHED_OTHER priority 0");
        }
    }
}

void MqttClient::shutdown() {
    m_shutdown = true;
    if (!m_thrCreated) {
        return;
    }
    if (pthread_cancel(m_thrId) != 0) {
    }
    void *res;
    if (pthread_join(m_thrId, &res) != 0) {
        SPDLOG_ERROR("Failed to join mqtt Client thread.");
    }
    m_thrCreated = false;
}

/*static*/
void *MqttClient::EntryOfThread(void *arg) {
    MqttClient *pClient = static_cast<MqttClient *>(arg);
    pClient->run();
    return (void *) (pClient);
}

void MqttClient::run() {
    const struct lws_protocols protocols[] = {
        {.name = "mqtt",
         .callback = &callbackEx,
         .per_session_data_size = sizeof(struct pss)},
        LWS_PROTOCOL_LIST_TERM};
    const lws_retry_bo_t retry = {
        .secs_since_valid_ping = 20,   /* if idle, PINGREQ after secs */
        .secs_since_valid_hangup = 25, /* hangup if still idle secs */
    };

    lws_state_notify_link_t notifier = {{NULL, NULL, NULL}, &system_notify_cb, "app"};
    lws_state_notify_link_t *na[] = {&notifier, NULL};

    struct lws_context_creation_info info;
    memset(&info, 0, sizeof info);

    info.port = CONTEXT_PORT_NO_LISTEN;
    info.protocols = protocols;
    info.register_notifier_list = na;
    info.fd_limit_per_thread = 1 + 1 + 1;
    info.retry_and_idle_policy = &retry;
    lws_set_log_level(LLL_USER | LLL_ERR | LLL_WARN, lws_spdlog_emit);

    m_context = lws_create_context(&info);
    if (!m_context) {
        SPDLOG_ERROR("lws init failed");
        return;
    }

    int n = 0;
    while ((n >= 0) && (m_shutdown == false)) {
        n = lws_service(m_context, 0);
        // lws v4.3 ignores the timeout parameter and manages its own poll()
        // internally, but if it returns without blocking (e.g. no fds, forced
        // service, or library quirk) the tight loop can burn 100% CPU.
        // Guard with a 1 ms floor so worst-case is ~1000 iterations/sec.
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    lws_context_destroy(m_context);
    SPDLOG_INFO("Exit from thread.");
}

int MqttClient::connect(struct lws_context *pcontext) {
    lws_mqtt_client_connect_param_t client_connect_param = {
        .client_id = m_clientId.c_str(),
        .keep_alive = 60,
        .clean_start = 1,
        .client_id_nofree = 1,
        .username_nofree = 1,
        .password_nofree = 1,
        .will_param = {
            .topic = "good/bye",
            .message = "sign-off",
            .qos = static_cast<lws_mqtt_qos_levels_t>(0),
            .retain = 0,
        },
        .username = m_username.c_str(),
        .password = m_password.c_str(),
    };

    struct lws_client_connect_info info;

    memset(&info, 0, sizeof info);

    info.mqtt_cp = &client_connect_param;
    info.address = m_address.c_str();
    info.host = m_host.c_str();
    info.protocol = "mqtt";
    info.context = pcontext;
    info.method = "MQTT";
    info.alpn = "mqtt";
    info.port = atoi(m_port.c_str());

    SPDLOG_INFO("connection [{}]:[{}]:[{}]:[{}]:[{}]",
                client_connect_param.client_id,
                client_connect_param.username,
                info.address,
                info.host,
                info.port);
    if (!lws_client_connect_via_info(&info)) {
        SPDLOG_ERROR("Failed to setup client Connect Failed");
        return 1;
    }
    return 0;
}

int MqttClient::reconnect() {
    return 0;
}

void MqttClient::disconnect() {
}

void MqttClient::publish(std::string &p_topic, const std::shared_ptr<MESSAGE> &p_message) {
    MqttMessage_ msg = std::make_pair(p_topic, p_message);
    {
        std::lock_guard<std::mutex> lock(m_mqttMutex);
        m_messages.emplace_back(msg);
    }

    std::string componentName = "app";
    struct lws *wsi = getWsiInstance(componentName);
    lws_callback_on_writable(wsi);
}

void MqttClient::publish(std::string &p_topic, const std::string &payload) {
    StringMessage_ msg = std::make_pair(p_topic, payload);
    {
        std::lock_guard<std::mutex> lock(m_mqttMutex);
        m_stringMessages.emplace_back(std::move(msg));
    }

    std::string componentName = "app";
    struct lws *wsi = getWsiInstance(componentName);
    if (wsi) {
        lws_callback_on_writable(wsi);
    }
}

//TODO need to decide when to unsubscribe the topic,
int MqttClient::unsubscribe(__attribute__((unused)) const std::string &topicFilter) {
    return 0;
}

int MqttClient::notifyCallback(lws_state_manager_t *mgr,
                               __attribute__((unused)) lws_state_notify_link_t *link,
                               int current,
                               int target) {
    m_context = (struct lws_context *) mgr->parent;

    if (current != LWS_SYSTATE_OPERATIONAL || target != LWS_SYSTATE_OPERATIONAL) {
        return 0;
    }

    /*
      * We delay trying to do the client connection until
      * the protocols have been initialized for each
      * vhost... this happens after we have network and
      * time so we can judge tls cert validity.
      */

    if (connect(m_context)) {
        SPDLOG_ERROR("failed to setup connection");
    }
    return 0;
}

void MqttClient::onClientWriteAble(struct lws *wsi, struct pss *pss) {
    switch (pss->state) {
    case STATE_SUBSCRIBE: {
        SPDLOG_TRACE("Subscribing");
        for (int idx = 0; idx < m_topics.size(); ++idx) {
            lws_mqtt_topic_elem_t elem;
            elem.name = m_topics[idx].c_str();
            elem.qos = QOS0;
            m_mqttTopics.emplace_back(elem);
        }

        m_subParam.num_topics = m_topics.size();
        m_subParam.topic = &m_mqttTopics[0];

        if (lws_mqtt_client_send_subcribe(wsi, &m_subParam)) {
            SPDLOG_ERROR("Failed to subscribe ");
        }
        pss->state = STATE_PUBLISH_QOS0;
    } break;

    case STATE_PUBLISH_QOS0: {
        // First try to drain a binary MESSAGE from the legacy queue.
        MqttMessage_ elem;
        bool hasBinary = false;
        {
            std::lock_guard<std::mutex> lock(m_mqttMutex);
            if (!m_messages.empty()) {
                elem = m_messages.front();
                m_messages.erase(m_messages.begin());
                hasBinary = true;
            }
        }
        if (hasBinary) {
            m_pubParam.topic = const_cast<char *>(elem.first.c_str());
            m_pubParam.topic_len = elem.first.length();
            m_pubParam.qos = QOS0;
            m_pubParam.payload_len = elem.second->length;
            SPDLOG_INFO("Publish topic [{}], len [{}]", m_pubParam.topic, m_pubParam.payload_len);
            if (lws_mqtt_client_send_publish(wsi, &m_pubParam,
                                             elem.second->Union.content,
                                             m_pubParam.payload_len, 1)) {
                SPDLOG_ERROR("Failed to send data");
            }
            break;
        }

        // Then try to drain a string message (telemetry JSON etc.)
        StringMessage_ strElem;
        bool hasString = false;
        {
            std::lock_guard<std::mutex> lock(m_mqttMutex);
            if (!m_stringMessages.empty()) {
                strElem = std::move(m_stringMessages.front());
                m_stringMessages.erase(m_stringMessages.begin());
                hasString = true;
            }
        }
        if (hasString) {
            m_pubParam.topic = const_cast<char *>(strElem.first.c_str());
            m_pubParam.topic_len = strElem.first.length();
            m_pubParam.qos = QOS0;
            m_pubParam.payload_len = strElem.second.size();
            SPDLOG_DEBUG("Publish string topic [{}], len [{}]", m_pubParam.topic, m_pubParam.payload_len);
            if (lws_mqtt_client_send_publish(wsi, &m_pubParam,
                                             strElem.second.data(),
                                             m_pubParam.payload_len, 1)) {
                SPDLOG_ERROR("Failed to send string data");
            }
        }
    }

    default:
        break;
    }
}

int MqttClient::callback(struct lws *wsi, enum lws_callback_reasons reason, void *user, void *in, size_t len) {
    struct pss *pss = (struct pss *) user;

    switch (reason) {
    case LWS_CALLBACK_CLIENT_CONNECTION_ERROR:
        SPDLOG_ERROR("CLIENT_CONNECTION_ERROR: {}", in ? (const char *) in : "(null)");
        m_isConnected = false;
        break;
    case LWS_CALLBACK_MQTT_CLIENT_CLOSED: {
        SPDLOG_TRACE("CLIENT_CLOSED");
        m_isConnected = false;
        std::string componentName = "app";
        removeWsiInstance(componentName);
        break;
    }
    case LWS_CALLBACK_MQTT_CLIENT_ESTABLISHED: {
        SPDLOG_TRACE("MQTT_CLIENT_ESTABLISHED");
        m_isConnected = true;
        std::string componentName = "app";
        addWsiInstance(componentName, wsi);
        lws_callback_on_writable(wsi);
        break;
    }

    case LWS_CALLBACK_MQTT_SUBSCRIBED:
        SPDLOG_TRACE("MQTT_SUBSCRIBED");
        break;

    case LWS_CALLBACK_MQTT_CLIENT_WRITEABLE:
        onClientWriteAble(wsi, pss);
        break;
    case LWS_CALLBACK_MQTT_ACK:
        SPDLOG_TRACE("MQTT_ACK");
        if (pss->state == STATE_PUBLISH_QOS0) {
            std::lock_guard<std::mutex> lock(m_mqttMutex);
            if (!m_messages.empty() || !m_stringMessages.empty()) {
                lws_callback_on_writable(wsi);
            }
        }
        break;
    case LWS_CALLBACK_MQTT_RESEND:
        SPDLOG_TRACE("MQTT_RESEND");
        if (++pss->retries == 3) {
            break;
        }
        pss->state--;
        pss->pos = 0;
        break;

    case LWS_CALLBACK_MQTT_CLIENT_RX:
        SPDLOG_TRACE("MQTT_CLIENT_RX");
        processMessage(in, len, wsi);
        return 0;
    default:
        break;
    }
    return 0;
}

/*
 * Generates a packet that the DS will send to the robot, it contains the
 * following information:
 *    - Packet index / ID
 *    - Control code (control modes, e-stop state, etc)
 *    - Request code (robot reboot, restart code, normal operation, etc)
 *    - Team station (alliance & position)
 *    - Date and time data (if robot requests it)
 *    - Joystick information (if the robot does not want date/time)
 */
void MqttClient::processMessage(void *in, __attribute__((unused)) size_t len, __attribute__((unused)) struct lws *wsi) {
    lws_mqtt_publish_param_t *pub_param = (lws_mqtt_publish_param_t *) in;
    assert(pub_param);

    auto pp = reinterpret_cast<const uint8_t *>(pub_param->payload);
    SPDLOG_INFO("[{}]: {:#04x}", pub_param->topic, fmt::join(pp, pp + pub_param->payload_len, " "));

    //TODO:: dispatch message according to the topic type.
    //if("FRC_ROBOT" == pub_param->topic)
    {
        m_newDataOccurHandler(pub_param->payload, pub_param->payload_len);
    }
    //else { "Command" ==  pub_param->topic )

    //}
    /*
	 {
	     Client * const client = (Client *)user;
	     const size_t remaining = lws_remaining_packet_payload(wsi);

         if (!remaining && lws_is_final_fragment(wsi)) {
            if (client->HasFragments()) {
                client->AppendMessageFragment(in, len, 0);
                in = (void *)client->GetMessage();
                len = client->GetMessageLength();
            }

            client->ProcessMessage((char *)in, len, wsi);
            client->ResetMessage();
        } else {
            client->AppendMessageFragment(in, len, remaining);
        }
    }
    */
}

void MqttClient::addWsiInstance(std::string &componentName, struct lws *wsi) {
    const std::lock_guard<std::mutex> lock(m_wsiMapMutex);
    if (m_wsiMap.find(componentName) == m_wsiMap.end()) {
        m_wsiMap.insert(std::make_pair(componentName, wsi));
        SPDLOG_TRACE("wsi instance for [{}] is added.", componentName);
    } else {
        SPDLOG_WARN("wsi instance for [{}] exist.", componentName);
    }
}

void MqttClient::removeWsiInstance(std::string &componentName) {
    const std::lock_guard<std::mutex> lock(m_wsiMapMutex);
    wsi_map_type_::iterator itmap = m_wsiMap.find(componentName);
    if (itmap == m_wsiMap.end()) {
        SPDLOG_WARN("wsi instance for [{}] does not exist.", componentName);
    } else {
        m_wsiMap.erase(itmap);
        SPDLOG_TRACE("wsi instance for [{}] is removed.", componentName);
    }
}

struct lws *MqttClient::getWsiInstance(std::string &componentName) {
    struct lws *instance = NULL;
    const std::lock_guard<std::mutex> lock(m_wsiMapMutex);
    wsi_map_type_::iterator itmap = m_wsiMap.find(componentName);
    if (itmap == m_wsiMap.end()) {
        SPDLOG_WARN("wsi instance for [{}] does NOT exist.", componentName);
    } else {
        instance = itmap->second;
    }
    return instance;
}