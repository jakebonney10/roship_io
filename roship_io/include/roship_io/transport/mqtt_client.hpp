#pragma once

#include "transport_interface.hpp"
#include <mosquitto.h>
#include <string>
#include <vector>
#include <functional>
#include <memory>
#include <iostream>
#include <stdexcept>

TRANSPORT_NS_HEAD

class MqttClient : public TransportInterface {
public:
    struct Params {
        std::string host;
        int port;
        std::vector<std::string> topics;
        int keep_alive;
    };

    MqttClient(const Params& params);
    ~MqttClient();
    void send(const std::vector<byte>& message) override;
    void send(const std::string& topic, const std::vector<byte>& message); // Overloaded function with topic
    void spinOnce() override;
    void addCallback(const MessageCallback& callback) override;

private:
    static void on_connect(struct mosquitto* mosq, void* userdata, int result);
    static void on_message(struct mosquitto* mosq, void* userdata, const struct mosquitto_message* message);
    static void on_disconnect(struct mosquitto* mosq, void* userdata, int reason);

    Params params_;
    struct mosquitto* mosq_;
    std::vector<MessageCallback> callbacks_;
};

TRANSPORT_NS_FOOT
