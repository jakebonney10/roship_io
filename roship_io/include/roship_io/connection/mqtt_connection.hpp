#pragma once

#include "connection_defs.hpp"
#include <rclcpp/rclcpp.hpp>
#include "io_connection.hpp"
#include "transport/mqtt_client.hpp"
#include <io_interfaces/msg/raw_packet.hpp>

CONNECTION_NS_HEAD

class MqttConnection : public IoConnection<transport::MqttClient>
{
public:
    struct Params
    {
        Params();
        void declare(rclcpp::Node::SharedPtr node);
        void update(rclcpp::Node::SharedPtr node);
        transport::MqttClient::Params client;
    };

    MqttConnection(rclcpp::Node::SharedPtr node);

    void mqttCallback(const std::vector<byte>& message);

    void sendToDevice(const io_interfaces::msg::RawPacket& msg);

    void spin_once();

protected:
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<transport::MqttClient> client_ptr_;
    rclcpp::Publisher<io_interfaces::msg::RawPacket>::SharedPtr raw_pub_;
    rclcpp::Subscription<io_interfaces::msg::RawPacket>::SharedPtr raw_sub_;
    Params params_;
};

CONNECTION_NS_FOOT
