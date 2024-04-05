#include "connection/mqtt_connection.hpp"

CONNECTION_NS_HEAD

using namespace std::chrono_literals;

MqttConnection::Params::Params()
{
    client.host = "localhost";
    client.port = 1883;
    client.topics = {"test/topic"};
    client.keep_alive = 60;
}

void MqttConnection::Params::declare(rclcpp::Node::SharedPtr node)
{
    node->declare_parameter("client.host", client.host);
    node->declare_parameter("client.port", client.port);
    node->declare_parameter("client.topics", client.topics);
    node->declare_parameter("client.keep_alive", client.keep_alive);
}

void MqttConnection::Params::update(rclcpp::Node::SharedPtr node)
{
    node->get_parameter("client.host", client.host);
    node->get_parameter("client.port", client.port);
    node->get_parameter("client.topics", client.topics);
    node->get_parameter("client.keep_alive", client.keep_alive);
}

MqttConnection::MqttConnection(rclcpp::Node::SharedPtr node) : 
    IoConnection<transport::MqttClient>(node)
{
    params_.declare(node_ptr_);
    params_.update(node_ptr_);

    client_ptr_.reset(new transport::MqttClient(params_.client));
    client_ptr_->addCallback(std::bind(&MqttConnection::mqttCallback, this, std::placeholders::_1));

    raw_pub_ = node_ptr_->create_publisher<io_interfaces::msg::RawPacket>("~/from_device", 10);
    raw_sub_ = node_ptr_->create_subscription<io_interfaces::msg::RawPacket>(
        "~/to_device", 1, std::bind(&MqttConnection::sendToDevice, this, std::placeholders::_1));

    timer_ = node_ptr_->create_wall_timer(
        1ms, std::bind(&MqttConnection::spin_once, this));

    RCLCPP_INFO(node_ptr_->get_logger(), "MQTT client connected to %s:%d", 
                params_.client.host.c_str(), params_.client.port);
}

void MqttConnection::mqttCallback(const std::vector<byte>& message)
{
    auto rx_time = node_ptr_->now();
    io_interfaces::msg::RawPacket::SharedPtr msg(new io_interfaces::msg::RawPacket);
    msg->header.stamp = rx_time;
    msg->data = message;
    raw_pub_->publish(*msg);
}

void MqttConnection::sendToDevice(const io_interfaces::msg::RawPacket& msg)
{
    // Publish to an MQTT topic, might need to specify the topic here or manage it some other way
    client_ptr_->send(msg.data);
}

void MqttConnection::spin_once()
{
    client_ptr_->spinOnce();
}

CONNECTION_NS_FOOT
