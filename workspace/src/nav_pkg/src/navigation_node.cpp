#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>

#include "nav_pkg/d_star_lite.hpp"

using std::placeholders::_1;

class NavigationNode : public rclcpp::Node
{
public:
    // Конструктор по умолчанию: инициализируем карту и ROS2
    NavigationNode() : Node("navigation_node"), planner_(200, 200) // Создаем карту 200х200 клеток
    {
        // НАСТРОЙКИ КАРТЫ И РОБОТА
        map_resolution_ = 0.1;

        // Чтобы робот не выехал за границы массива, сместим нулевую координату (0,0) метров
        // в центр нашей матрицы (клетка 100, 100).
        map_offset_ = 100;

        // ЗАДАЕМ ХАРДКОД ЦЕЛИ
        goal_x_meters_ = 8.543366;
        goal_y_meters_ = 8.263840;

        // 3. ИНИЦИАЛИЗАЦИЯ D* LITE
        GridCoordinate start_grid = metersToGrid(0.0, 0.0);
        GridCoordinate goal_grid = metersToGrid(goal_x_meters_, goal_y_meters_);
        planner_.setStartAndGoal(start_grid, goal_grid);
        planner_.calculatePathCosts();

        // 4. ПОДПИСКИ И ПУБЛИКАЦИИ
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&NavigationNode::odomCallback, this, _1));

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&NavigationNode::scanCallback, this, _1));

        // 5. ТАЙМЕР УПРАВЛЕНИЯ (Вызывается 10 раз в секунду)
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&NavigationNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Узел навигации запущен. Цель установлена.");
    }

private:
    // --- ОСНОВНЫЕ ПЕРЕМЕННЫЕ ---
    DStarLite planner_;
    double map_resolution_;
    int map_offset_;

    // КООРДИНАТЫ ЦЕЛИ ПОКА ЗАХАРДКОЖЕНЫ
    double goal_x_meters_;
    double goal_y_meters_;

    // Текущее состояние робота
    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_yaw_ = 0.0; // Угол поворота робота в радианах
    bool odom_received_ = false;

    // Объекты ROS2
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // --- ФУНКЦИИ-ПОМОЩНИКИ ---

    // Перевод реальных метров в индексы массива
    GridCoordinate metersToGrid(double x, double y)
    {
        int grid_x = static_cast<int>(std::round(x / map_resolution_)) + map_offset_;
        int grid_y = static_cast<int>(std::round(y / map_resolution_)) + map_offset_;
        return GridCoordinate{grid_x, grid_y};
    }

    // Перевод индексов массива обратно в реальные метры
    std::pair<double, double> gridToMeters(GridCoordinate cell)
    {
        double x = (cell.x - map_offset_) * map_resolution_;
        double y = (cell.y - map_offset_) * map_resolution_;
        return {x, y};
    }

    // --- КОЛЛБЭКИ ROS2 ---

    // 1. Одометрия: Узнаем, где мы находимся сейчас
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        // Извлекаем угол Yaw из кватерниона (математическая формула Эйлера)
        // Это избавляет нас от подключения тяжелой библиотеки tf2
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        current_yaw_ = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

        odom_received_ = true;
    }

    // 2. Лидар: Ищем препятствия и наносим на карту D*
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (!odom_received_)
            return;

        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            double distance = msg->ranges[i];

            if (distance > msg->range_min && distance < 2.5)
            { // Ограничим лидар 2.5 метрами для стабильности
                double ray_angle = msg->angle_min + i * msg->angle_increment;
                double total_angle = current_yaw_ + ray_angle;

                double obs_x = current_x_ + distance * std::cos(total_angle);
                double obs_y = current_y_ + distance * std::sin(total_angle);

                GridCoordinate obs_grid = metersToGrid(obs_x, obs_y);

                // ИНФЛЯЦИЯ ПРЕПЯТСТВИЙ:
                // Возьмем радиус в 2 клетки во все стороны (матрица 5х5 вокруг точки удара лидара).
                // Так как шаг сетки 10 см, мы создадим "буферную зону безопасности" в 20 см вокруг кубика.
                for (int dx = -2; dx <= 2; ++dx)
                {
                    for (int dy = -2; dy <= 2; ++dy)
                    {
                        GridCoordinate inflated_cell{obs_grid.x + dx, obs_grid.y + dy};

                        // Проверяем, что не выходим за границы карты 200х200
                        if (inflated_cell.x >= 0 && inflated_cell.x < 200 &&
                            inflated_cell.y >= 0 && inflated_cell.y < 200)
                        {

                            planner_.updateMapObstacle(inflated_cell, true);
                        }
                    }
                }
            }
        }
    }

    // 3. Главный цикл управления: Вычисляем куда ехать и крутим колеса
    void controlLoop()
    {
        if (!odom_received_)
            return;

        // Считаем общую дистанцию до финальной цели
        double dist_to_goal = std::hypot(goal_x_meters_ - current_x_, goal_y_meters_ - current_y_);
        if (dist_to_goal < 0.2)
        {
            stopRobot();
            RCLCPP_INFO_ONCE(this->get_logger(), "Финальная цель достигнута!");
            return;
        }

        // Получаем следующую клетку от D* Lite
        GridCoordinate current_grid = metersToGrid(current_x_, current_y_);
        GridCoordinate next_step = planner_.getNextBestStep(current_grid);
        auto [target_x, target_y] = gridToMeters(next_step);

        // Считаем дистанцию до центра СЛЕДУЮЩЕЙ промежуточной клетки
        double dist_to_next_cell = std::hypot(target_x - current_x_, target_y - current_y_);

        // Вычисляем целевой угол
        double angle_to_target = std::atan2(target_y - current_y_, target_x - current_x_);
        double angle_error = angle_to_target - current_yaw_;

        // Нормализуем угол в диапазон [-PI, PI]
        while (angle_error > M_PI)
            angle_error -= 2.0 * M_PI;
        while (angle_error < -M_PI)
            angle_error += 2.0 * M_PI;

        geometry_msgs::msg::Twist cmd;

        // ЗАЩИТА ОТ СИНГУЛЯРНОСТИ: Если мы уже ближе чем на 7 см к центру клетки,
        // не нужно пытаться идеально её выцеливать, иначе atan2 начнет скакать из-за шума.
        if (dist_to_next_cell < 0.07)
        {
            angle_error = 0.0; // Обнуляем ошибку, заставляя робота просто ехать в следующую клетку
        }

        // ГЛАДКИЙ РЕГУЛЯТОР (Вместо bang-bang машины состояний):
        // Линейная скорость плавно уменьшается, если робот сильно отвернут от цели.
        // Если смотрит прямо (angle_error = 0) -> cos(0) = 1 -> скорость максимальна (0.25 м/с).
        // Если отвернут на 90 градусов -> cos(PI/2) = 0 -> робот плавно останавливается и крутится.
        double max_linear_speed = 0.25;
        cmd.linear.x = max_linear_speed * std::cos(angle_error);

        // Запрещаем роботу ехать назад, если cos ушел в минус
        if (cmd.linear.x < 0.0)
            cmd.linear.x = 0.0;

        // Угловая скорость пропорциональна ошибке (коэффициент 2.5 сделает поворот отзывчивым)
        cmd.angular.z = 2.5 * angle_error;

        // Жёсткие лимиты, чтобы робота не унесло в стратосферу
        if (cmd.angular.z > 0.8)
            cmd.angular.z = 0.8;
        if (cmd.angular.z < -0.8)
            cmd.angular.z = -0.8;

        cmd_pub_->publish(cmd);
    }

    void stopRobot()
    {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_pub_->publish(cmd);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}