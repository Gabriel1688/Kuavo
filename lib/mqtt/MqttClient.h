#pragma once

#include "libwebsockets.h"
#include "libwebsockets/lws-mqtt.h"
#include "message.h"
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <utility>
#include <vector>

extern std::atomic<class MqttClient *> g_mqttClient_ptr;

struct pss {
    int state;
    size_t pos;
    int retries;
};

class MqttClient {
public:
    /**
	 * Create a client that can be used to communicate with an MQTT server.
	 * This allows the caller to specify a user-defined persistence object,
	 * or use no persistence.
	 * @param serverURI the address of the server to connect to, specified
	 *  				  as a URI.
	 * @param clientId a client identifier that is unique on the server
	 * 				   being connected to
	 * @param persistence The user persistence structure. If this is null,
	 * 					then no persistence is used.
	 */
    MqttClient() {
        m_shutdown = false;
        m_clientId = "MqttClient";
        m_context = nullptr;
        m_newDataOccurHandler = nullptr;
        memset(&m_subParam, 0, sizeof(m_subParam));
        memset(&m_pubParam, 0, sizeof(m_pubParam));
    }

    /**
	 * Virtual destructor
	 */
    ~MqttClient(){};

    /**
	 * Connects to an MQTT server using the default options.
	 */
    int connect(struct lws_context *context);

    void loadConfig(const std::string &fileName);

    /**
	 * Reconnects the client using options from the previous connect.
	 * The client must have previously called connect() for this to work.
	 */
    int reconnect();

    /**
	 * Disconnects from the server.
	 */
    void disconnect();

    /**
	 * Disconnects from the server.
	 * @param timeoutMS the amount of time in milliseconds to allow for
	 *  				  existing work to finish before disconnecting. A value
	 *  				  of zero or less means the client will not quiesce.
	 */

    /**
	 * Gets the client ID used by this client.
	 * @return The client ID used by this client.
	 */
    std::string getClientId() const {
        return m_clientId;
    }

    /**
	 * Gets the address of the server used by this client.
	 * @return The address of the server used by this client, as a URI.
	 */
    std::string getServerUri() const {
        return m_serverUri;
    }

    /**
	 * Return the maximum time to wait for an action to complete.
	 * @return int
	 */
    std::vector<std::string> getTopic();

    /**
	 * Return the maximum time to wait for an action to complete.
	 * @return int
     */
    void setClientId(const std::string &p_clientId) {
        m_clientId = p_clientId;
    }

    /**
	 * Determines if this client is currently connected to the server.
	 * @return @em true if this client is currently connected to the server, @em false if
	 * 	  	 not.
	 */
    bool isConnected() const {
        return m_isConnected;
    }

    /**
	 * Publishes an opaque binary payload to a topic on the server.
	 * @param topic The topic to publish (must remain valid until sent)
	 * @param data  The binary payload bytes
	 * @param len   The number of bytes to publish
	 * @param qos   MQTT QoS level (0, 1, or 2)
	 * @param retain Whether to publish as a retained message
	 * @return true if queued successfully, false if the send queue is full
	 */
    bool publish_binary(const char *topic, const uint8_t *data, size_t len, int qos, bool retain = false);

    void processMessage(void *in, size_t len, struct lws *wsi);

    /**
	 * Requests the server unsubscribe the client from a topic.
	 * @param topicFilter A single topic to unsubscribe.
	 * @param props The MQTT v5 properties.
	 * @return The "unsubscribe" response from the server.
	 */
    int unsubscribe(const std::string &topicFilter);
    int callback(struct lws *wsi,
                 enum lws_callback_reasons reason,
                 void *user,
                 void *in,
                 size_t len);

    int notifyCallback(lws_state_manager_t *mgr,
                        lws_state_notify_link_t *link,
                        int current,
                        int target);

    void start();
    void shutdown();
    void run();
    static void *EntryOfThread(void *arg);
    void addWsiInstance(std::string &componentName, struct lws *wsi);
    void removeWsiInstance(std::string &componentName);
    void setOccurFuncPointer(void (*dataOccurHandler)(const void *payload, uint32_t payload_len)) {
        m_newDataOccurHandler = dataOccurHandler;
    }

    struct lws *getWsiInstance(std::string &componentName);

    void asyncResult(std::string &result);

    void onClientWriteAble(struct lws *wsi, struct pss *pss);
    void publishStatusOnline();
    void publishStatusOffline();
    void scheduleReconnect();

    static constexpr const char *TOPIC_COMMAND_BIN = "robot/command/bin";
    static constexpr const char *TOPIC_SENSOR_BIN = "robot/sensor/bin";
    static constexpr const char *TOPIC_STATUS = "robot/status";

    bool m_shutdown;
    bool m_thrCreated = false;
    pthread_t m_thrId;

    std::atomic<bool> m_isConnected{false};
    std::chrono::steady_clock::time_point m_reconnectAt{std::chrono::steady_clock::time_point::max()};
    std::chrono::milliseconds m_reconnectDelay{1000};
    bool m_reconnectPending{false};
    std::vector<lws_mqtt_topic_elem_t> m_mqttTopics;
    std::vector<std::string> m_topics;
    std::string m_clientId;
    std::string m_username;
    std::string m_password;
    std::string m_serverUri;
    std::string m_address;
    std::string m_port;
    std::string m_host;
    int m_qos = 0;
    int m_robotId = 1;

    struct lws_context *m_context;

    lws_mqtt_subscribe_param_t m_subParam;
    lws_mqtt_publish_param_t m_pubParam;

    struct BinaryMessage {
        const char *topic;
        std::vector<uint8_t> payload;
        int qos;
        bool retain;
    };
    std::queue<BinaryMessage> m_binaryMessages;
    size_t m_highWater = 1000;
    size_t m_lowWater = 500;
    std::mutex m_mqttMutex;
    bool m_highWaterWarning = false;

    /*
        If you want to keep a list of live wsi, 
        you need to use lifecycle callbacks on the protocol in the service 
        thread to manage the list, with your own locking. 
        Typically you use ESTABLISHED callback to add ws wsi to your list and 
        a CLOSED callback to remove them.*/
    using wsi_map_type_ = std::map<std::string, struct lws *>;
    wsi_map_type_ m_wsiMap;
    std::mutex m_wsiMapMutex;
    void (*m_newDataOccurHandler)(const void *payload, uint32_t payload_len);
};
