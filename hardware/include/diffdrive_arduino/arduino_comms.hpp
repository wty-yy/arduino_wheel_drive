#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP

// #include <cstring>
#include <sstream>
// #include <cstdlib>
#include <libserial/SerialPort.h>
#include <iostream>
#include <vector>
#include <unistd.h>


LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

class ArduinoComms
{

public:

  ArduinoComms() = default;

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {  
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
    timeout_ms_ = timeout_ms;
    for (int i = 0; i < 3; i++) {
      std::ostringstream oss;
      oss << "sleep " << i+1 << "/3 seconds";
      RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), oss.str().c_str());
      sleep(1);
    }
  }

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }


  std::string send_msg(const std::string &msg_to_send, bool print_output = false)
  {
    serial_conn_.FlushIOBuffers(); // Just in case
    serial_conn_.Write(msg_to_send+"\r");
    std::ostringstream oss;
    oss << "Sent: " << msg_to_send;
    std::string result = oss.str();
    // RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), result.c_str());

    std::string response = "";
    try
    {
      // Responses end with \r\n so we will read up to (and including) the \n.
      serial_conn_.ReadLine(response, '\n', timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout&)
    {
        std::cerr << "The ReadByte() call has timed out." << std::endl ;
    }

    if (print_output)
    {
      std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
    }

    return response;
  }


  void send_empty_msg()
  {
    std::string response = send_msg("\r");
  }

  void read_encoder_values(int &val_1, int &val_2, int &val_3, int &val_4)
  {
    std::string response = send_msg("e\r");

    std::istringstream iss(response);
    std::vector<int> vals;
    int val;
    while (iss >> val) vals.push_back(val);

    val_1 = vals[0];
    val_2 = vals[1];
    val_3 = vals[2];
    val_4 = vals[3];
  }
  
  void set_motor_rpm(int rpm_1, int rpm_2, int rpm_3, int rpm_4)
  {
    std::stringstream ss;
    ss << "s " << rpm_1 << " " << rpm_2 << " " << rpm_3 << " " << rpm_4 << "\r";
    send_msg(ss.str());
  }

private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP