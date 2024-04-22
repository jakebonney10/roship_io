#include "modbus/modbus_node.hpp"

using namespace std::chrono_literals;

MODBUS_NS_HEAD

ModbusNode::ModbusNode(std::string name)
    : Node(name)
{
  timer_ = this->create_wall_timer(
      500ms, std::bind(&ModbusNode::timer_callback, this));
  connect_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(params_.timers.connect_ms), std::bind(&ModbusNode::connect, this));
  connect();
}

void ModbusNode::readInputRegisters(Block &block)
{
  try{
    modbus_.read_input_registers(block);
  }catch(std::runtime_error err){
    std::string error_msg = err.what();
    RCLCPP_WARN(this->get_logger(),"modbus read failed: ", error_msg.c_str());
  }
}

void ModbusNode::writeRegisters(Block &block)
{
  try{
    modbus_.write_registers(block);
  }catch(std::runtime_error err){
    std::string error_msg = err.what();
    RCLCPP_WARN(this->get_logger(),"modbus write failed: ", error_msg.c_str());
  }
}

void ModbusNode::connect()
{
  if(connected_){
    return;
  }
  try{
    if(params_.connection.type == "tcp"){
      RCLCPP_INFO_ONCE(this->get_logger(),"Connecting to %s:%i",params_.connection.ip.c_str(), params_.connection.network_port);
      modbus_.connect_tcp(params_.connection.ip.c_str(), params_.connection.network_port);
    }else{
      RCLCPP_ERROR(this->get_logger(),"Invalid connection type specified %s",params_.connection.type.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(),"Connectd to device");
    connect_timer_->reset();
    connected_ = true;
    RCLCPP_INFO(this->get_logger(),"Connected!");
  }catch(std::runtime_error err){
    connected_ = false;
    std::string error_msg = err.what();
    RCLCPP_WARN_ONCE(this->get_logger(),"Unable to connect, will continue trying. Got Error\n ---%s", error_msg.c_str());
  }
}

void ModbusNode::timer_callback()
{
  if(connected_){
    onPoll();
  }
}

MODBUS_NS_FOOT
