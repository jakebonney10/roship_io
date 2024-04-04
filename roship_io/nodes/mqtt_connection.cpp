#include "connection/mqtt_connection.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node(new rclcpp::Node("mqtt_connection"));
  roship_io::connection::MqttConnection::SharedPtr connection(new roship_io::connection::MqttConnection(node));
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;

}
