#include <libserial/SerialStream.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <thread>

class SerialPort : public rclcpp::Node {
 public:
  SerialPort() : Node("serial_port") {
    this->declare_parameter("device", "/dev/ttyACM0");
    this->device_ = this->get_parameter("device").as_string();

    this->declare_parameter("device_name", "serial_port");
    this->device_name_ = this->get_parameter("device_name").as_string();

    this->declare_parameter("carriage_return", false);
    this->add_carriage_return_ = this->get_parameter("carriage_return").as_bool();

    this->declare_parameter("raw_in", false);
    this->raw_in_ = this->get_parameter("raw_in").as_bool();

    this->serial_port_pub_ = this->create_publisher<std_msgs::msg::String>(
        this->device_name_ + "/out", 10);

    if (this->raw_in_) {
      this->serial_port_raw_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        this->device_name_ + "/in/raw", 10,
        std::bind(&SerialPort::send_data_raw_cb, this, std::placeholders::_1));
    } else {
      this->serial_port_sub_ = this->create_subscription<std_msgs::msg::String>(
        this->device_name_ + "/in", 10,
        std::bind(&SerialPort::send_data_cb, this, std::placeholders::_1));
    }
  }

  void start() {
    this->reader_thread_ = std::thread(&SerialPort::read_data_thread, this);
  }

  bool open_port() {
    try {
      this->serial_port_.Open(this->device_,
                              std::ios_base::in | std::ios_base::out);
    } catch (const LibSerial::OpenFailed&) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open serial port");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "serial port opened");

    // Set serial port settings
    this->serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    this->serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    this->serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
    this->serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    this->serial_port_.SetFlowControl(
        LibSerial::FlowControl::FLOW_CONTROL_NONE);

    return true;
  }

  void close_port() {
    this->serial_port_.Close();
    RCLCPP_INFO(this->get_logger(), "serial port closed");
  }

  void read_data_thread() {
    char ch;
    std::string line;
    while (rclcpp::ok()) {
      if (this->serial_port_.GetNumberOfBytesAvailable() > 0) {
        this->serial_port_.read(&ch, 1);
        if (this->serial_port_.gcount() > 0) {
          if (ch == '\n') {
            /* RCLCPP_INFO(this->get_logger(), "Received: %s", line.c_str()); //
             * DEBUG */
            auto msg = std_msgs::msg::String();
            msg.data = line;
            this->serial_port_pub_->publish(msg);
            line.clear();
          } else {
            line += ch;
          }
        }
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
  }

  void send_data_cb(const std_msgs::msg::String& msg) {
    std::string data = std::string(msg.data);
    if(this->add_carriage_return_) {
      data += "\r";
    }
    data += "\n";
    this->serial_port_ << data.c_str();
    this->serial_port_.DrainWriteBuffer();

    /* RCLCPP_INFO(this->get_logger(), "Sent: %s", msg.data.c_str()); // DEBUG
     */
  }

  void send_data_raw_cb(const std_msgs::msg::UInt8MultiArray& msg) {
    std::vector<uint8_t> data = msg.data;
    this->serial_port_.write(reinterpret_cast<const char*>(data.data()), data.size());
    this->serial_port_.DrainWriteBuffer();
  }

  void wait() { this->reader_thread_.join(); }

 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr serial_port_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr serial_port_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr serial_port_raw_sub_;

  LibSerial::SerialStream serial_port_;
  std::thread reader_thread_;
  std::string device_;
  std::string device_name_;
  bool add_carriage_return_, raw_in_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto serial_port = std::make_shared<SerialPort>();
  if (!serial_port->open_port()) {
    return -1;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(serial_port);

  std::this_thread::sleep_for(std::chrono::milliseconds(5000));  // Why??
  serial_port->start();
  executor.spin();

  serial_port->wait();
  rclcpp::shutdown();
  return 0;
}
