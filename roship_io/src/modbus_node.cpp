#include "modbus/modbus_node.hpp"

using namespace std::chrono_literals;

MODBUS_NS_HEAD

ModbusNode::ModbusNode(std::string name)
    : Node(name)
{
  params_.declare(this);
  params_.update(this);
  timer_ = this->create_wall_timer(
      500ms, std::bind(&ModbusNode::timer_callback, this));
  connect_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(params_.timers.connect_ms), std::bind(&ModbusNode::connect, this));
  connect();
}

bool ModbusNode::readInputRegisters(Block &block)
{
  try{
    modbus_.read_input_registers(block);
    return true;
  }catch(std::runtime_error err){
    std::string error_msg = err.what();
    RCLCPP_WARN(this->get_logger(),"modbus read failed: %s", error_msg.c_str());
    return false;
  }
}

bool ModbusNode::writeRegisters(Block &block)
{
  try{
    modbus_.write_registers(block);
    return true;
  }catch(std::runtime_error err){
    std::string error_msg = err.what();
    RCLCPP_WARN(this->get_logger(),"modbus write failed: %s", error_msg.c_str());
    return false;
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

    //modbus_.set_response_timout(1,params_.connection.response_timout_ms*1000);


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

void ModbusNode::Params::declare(rclcpp::Node *node)
{
  // Declare parameters for the 'timers' struct
  node->declare_parameter("timers.poll_ms", timers.poll_ms);
  node->declare_parameter("timers.connect_ms", timers.connect_ms);
  node->declare_parameter("timers.write_ms", timers.write_ms);

  // Declare parameters for the 'connection' struct
  node->declare_parameter("connection.type", connection.type);
  node->declare_parameter("connection.ip", connection.ip);
  node->declare_parameter("connection.network_port", connection.network_port);
}

void ModbusNode::Params::update(rclcpp::Node *node)
{
  // Update parameters for the 'timers' struct
  node->get_parameter("timers.poll_ms", timers.poll_ms);
  node->get_parameter("timers.connect_ms", timers.connect_ms);
  node->get_parameter("timers.write_ms", timers.write_ms);

  // Update parameters for the 'connection' struct
  node->get_parameter("connection.type", connection.type);
  node->get_parameter("connection.ip", connection.ip);
  node->get_parameter("connection.network_port", connection.network_port);
}

MODBUS_NS_FOOT
