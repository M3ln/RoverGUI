#include "rclcpp/rclcpp.hpp"
#include "rover_interface/msg/ackermann.hpp"
#include "fcntl.h"      // Для open()
#include "unistd.h"     // Для close(), write()
#include <string>
#include <cmath>
#include <stdexcept>    // Для исключений
#include "termios.h"

const double DISTANCE_BETWEEN_AXES = 0.25; 

class UARTBridge : public rclcpp::Node {
public:
    UARTBridge() : Node("uart_bridge") {
        // Открытие UART-порта
        uart_fd_ = open("/dev/ttyTHS1", O_WRONLY | O_NOCTTY);  // Открываем порт только для записи
        if (uart_fd_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open UART port: %s", strerror(errno));
            throw std::runtime_error("UART port opening failed.");
        }

        
        // Настройка параметров последовательного порта
        struct termios tty;
        tcgetattr(uart_fd_, &tty);

        // Устанавливаем скорость передачи данных (9600 бод)
        cfsetospeed(&tty, B9600);
        cfsetispeed(&tty, B9600);

        // Настройка других параметров
        tty.c_cflag &= ~PARENB; // Без контроля четности
        tty.c_cflag &= ~CSTOPB; // 1 стоп-бит
        tty.c_cflag &= ~CSIZE;  // Очищаем биты размера данных
        tty.c_cflag |= CS8;     // 8 бит данных
        tty.c_cflag &= ~CRTSCTS; // Без аппаратного управления потоком
        tty.c_cflag |= CREAD | CLOCAL; // Включаем приемник

        tty.c_lflag &= ~ICANON; // Неканонический режим
        tty.c_lflag &= ~ECHO;   // Отключаем эхо
        tty.c_lflag &= ~ECHOE;
        tty.c_lflag &= ~ECHONL;
        tty.c_lflag &= ~ISIG;   // Отключаем обработку сигналов

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Отключаем программное управл>
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICR | ICRNL)

        tty.c_oflag &= ~OPOST; // Отключаем обработку вывода
        tty.c_oflag &= ~ONLCR;
        
        // Применяем настройки
        tcsetattr(uart_fd_, TCSANOW, &tty);
        RCLCPP_INFO(this->get_logger(), "UART port opened successfully.");

        // Подписка на топик 
        subscription_ = this->create_subscription<rover_interface::msg::Ackermann>(
            "/robot/cmd_control", 10, std::bind(&UARTBridge::listener_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "UART bridge node started. Listening to topic '/robot/cmd_control'.");
    }

    ~UARTBridge() {
        // Закрытие UART порта при уничтожении объекта
        if (uart_fd_ != -1) {
            close(uart_fd_);
            RCLCPP_INFO(this->get_logger(), "UART port closed.");
        }
    }

private:
    void listener_callback(const rover_interface::msg::Ackermann::SharedPtr msg) {
        // Логируем полученное сообщение
        RCLCPP_INFO(this->get_logger(), "Received from operator: linear velocity: %i; angular velocity %i", 
                    msg->linear_vel, msg->angle);

        // Отправляем сообщение на микроконтроллер по UART
        if (uart_fd_ != -1) {
            const std::string command = ConvertToStr(msg); 
            ssize_t bytes_written = write(uart_fd_, command.data(), command.size());
            if (bytes_written == -1) {
                RCLCPP_ERROR(this->get_logger(), "Failed to send data to UART");
            }
            else {
                RCLCPP_INFO(this->get_logger(), "Sent to UART: %s", command.c_str());
            }
        }
    }

    static const std::string ConvertToStr(const rover_interface::msg::Ackermann::SharedPtr msg) {
        int angle = msg->angle;
        std::string angle_str = std::to_string(angle);

        int velocity = msg->linear_vel;
        std::string velocity_str = std::to_string(velocity);

        std::string result = 'e' + velocity_str + angle_str;
        return result;
    } 

    int uart_fd_;  // Файловый дескриптор UART
    rclcpp::Subscription<rover_interface::msg::Ackermann>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UARTBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}