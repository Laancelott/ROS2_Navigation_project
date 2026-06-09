#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <algorithm>
#include <cmath>

class RescuePlanner : public rclcpp::Node
{
public:
    RescuePlanner() : Node("rescue_planner")
    {
        // Подписываемся на топик лидара
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&RescuePlanner::lidar_callback, this, std::placeholders::_1));

        // Будем публиковать команды управления моторами
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "Автономный спасательный планер запущен!");
    }

private:
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Наш лидар выдает 360 лучей. Индекс 180 — это строго вперед.
        // Давай проверим сектор перед роботом: от 150° (чуть правее) до 210° (чуть левее)
        int sector_start = 150;
        int sector_end = 210;
        
        float min_distance_to_obstacle = 999.0;

        for (int i = sector_start; i <= sector_end; ++i) {
            // Защита от некорректных значений (иногда лидар возвращает nan или inf)
            if (std::isnan(msg->ranges[i]) || std::isinf(msg->ranges[i])) {
                continue;
            }
            
            // Ищем самое близкое препятствие в нашем секторе видимости
            if (msg->ranges[i] < min_distance_to_obstacle) {
                min_distance_to_obstacle = msg->ranges[i];
            }
        }

        auto motor_command = geometry_msgs::msg::Twist();

        // ЛОГИКА ИЗБЕГАНИЯ СТЕН:
        if (min_distance_to_obstacle < 0.7) {
            // Если до обломка стены меньше 70 см — даем задний ход и резко разворачиваемся
            RCLCPP_WARN(this->get_logger(), "Опасность! Препятствие слишком близко: %.2f м! Сдаю назад и разворачиваюсь.", min_distance_to_obstacle);
            motor_command.linear.x = -0.1;  // Медленно отъезжаем назад
            motor_command.angular.z = 0.8; // Быстро крутимся на месте
        } 
        else if (min_distance_to_obstacle < 1.5) {
            // Если до стены меньше 1.5 метров — плавно притормаживаем и начинаем подруливать в сторону
            RCLCPP_INFO(this->get_logger(), "Вижу препятствие впереди (%.2f м). Снижаю скорость и уклоняюсь.", min_distance_to_obstacle);
            motor_command.linear.x = 0.1;   // Снижаем скорость хода вперед
            motor_command.angular.z = 0.4;  // Мягко отворачиваем траекторию
        } 
        else {
            // Путь чист! Едем вперед на полной спасательной скорости
            motor_command.linear.x = 0.3;   // Скорость 0.3 м/с
            motor_command.angular.z = 0.0;  // Едем строго прямо
        }

        // Отправляем команду в общую DDS сеть контейнеров
        cmd_pub_->publish(motor_command);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RescuePlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}