#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>

#include "nav_pkg/d_star_lite.hpp"

using placeholders::_1;
using namespace std;

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

        // ИНИЦИАЛИЗАЦИЯ D* LITE
        GridPoint start_grid = metersToGrid(0.0, 0.0);
        GridPoint goal_grid = metersToGrid(goal_x_meters_, goal_y_meters_);
        planner_.setStartAndGoal(start_grid, goal_grid);
        
        // ВАЖНО: Добавил первоначальный расчет пути
        planner_.calculatePath();

        // ОДПИСКИ И ПУБЛИКАЦИИ НА РОС КОМПОНЕНТЫ 

        // подписка на управление скоростью робота 
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // подписка на одометрию, чтобы знать, где мы сейчас
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, bind(&NavigationNode::odomCallback, this, _1));

        // подписка на лидар, чтобы обновлять карту препятствий
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, bind(&NavigationNode::scanCallback, this, _1));

        // ТАЙМЕР УПРАВЛЕНИЯ вызывается каждые 100 милисек
        control_timer_ = this->create_wall_timer(
            chrono::milliseconds(100),
            bind(&NavigationNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Узел навигации запущен. Цель установлена.");
    }

private:
    // объект нашего алгоритма Д старлайт (из пацанов дура блондинка)
    DStarLite planner_;

    // TODO: подобрать разрешени!!!!
    // разрешение карты в метраха на клетку можно в целом увеличить хз
    double map_resolution_;
    int map_offset_;

    // КООРДИНАТЫ ЦЕЛИ ПОКА ЗАХАРДКОЖЕНЫ !!!!!!!!!!!!!!!!!!!!!!!!!!!
    double goal_x_meters_;
    double goal_y_meters_;

    // Текущее состояние робота
    // вообще наверное можно было бы юзать структуру, но так как это всего 3 переменные
    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_yaw_ = 0.0; // Угол поворота робота в РАДИАНАХ НЕ В ГРАДУСАХ!!!!
    bool odom_received_ = false;

    // ++++ Объекты ROS2

    // публикатор для управления скоростью робота
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    // паписка на одометрию и лидар
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // паписка на лидар (ну сканер)
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    // Таймыр 
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Перевод реальных метров в индексы массива
    GridPoint metersToGrid(double x, double y)
    {
        int grid_x = static_cast<int>(round(x / map_resolution_)) + map_offset_;
        int grid_y = static_cast<int>(round(y / map_resolution_)) + map_offset_;
        return GridPoint{grid_x, grid_y};
    }

    // Перевод индексов массива обратно в реальные метры
    pair<double, double> gridToMeters(GridPoint cell)
    {
        double x = (cell.x - map_offset_) * map_resolution_;
        double y = (cell.y - map_offset_) * map_resolution_;
        return {x, y};
    }

    // --- КОЛЛБЭКИ ROS2 ---

    // зунаем, где мы находимся сейчас по одометрии
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        // берем угол поворота из положения в виде кватернион
        // конвертируем его в угол в радианах!!!!!!!!!!

        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        current_yaw_ = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

        odom_received_ = true;
    }

    // ну это просто функция которая реализует реакцию на данные с лидара. 
    //  она будет вызываться каждый раз, когда придет новый скан.
    // в ней обновляется карта препятствий д стар лайт и запускается пересчет пути.
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (!odom_received_)
            return;

        // Запоминаем текущую клетку робота
        GridPoint robot_grid = metersToGrid(current_x_, current_y_);
        
        bool map_updated = false;

        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            double distance = msg->ranges[i];

            // Сужаем слепую зону до 2 м, чтобы снизить дальние 
            if (distance > msg->range_min && distance < 2.0)
            {
                double ray_angle = msg->angle_min + i * msg->angle_increment;
                double total_angle = current_yaw_ + ray_angle;

                double obs_x = current_x_ + distance * cos(total_angle);
                double obs_y = current_y_ + distance * sin(total_angle);
                GridPoint obs_grid = metersToGrid(obs_x, obs_y);

                // ТУТ КРЧ ВООБЩЕ ПРИКОЛ (КАРИМ ГЕНИЙ КОМПЬЮТЕРА)
                // Мы делаем увеличение препятствия
                // Типа робот видит его больше, чтобы не пытаться ехать вплотную и не застревать
                for (int dx = -2; dx <= 2; ++dx)
                {
                    for (int dy = -2; dy <= 2; ++dy)
                    {
                        // Обрезаем углы квадрата 5на5, делая круглую зону опасности
                        if (abs(dx) + abs(dy) > 3)
                            continue;

                        GridPoint inflated_cell{obs_grid.x + dx, obs_grid.y + dy};

                        // Не ставим препятствие под колеса роботу!
                        if (inflated_cell == robot_grid)
                            continue;

                        if (inflated_cell.x >= 0 && inflated_cell.x < 200 &&
                            inflated_cell.y >= 0 && inflated_cell.y < 200)
                        {
                            // Если клетка еще не стена, ставим стену и запоминаем, что карта обновилась
                            if (!planner_.isWall(inflated_cell)) {
                                planner_.setObstacle(inflated_cell, true);
                                map_updated = true;
                            }
                        }
                    }
                }
            }
        }
        
        // ВАЖНО: Пересчитываем путь ОДИН РАЗ за весь скан лидара, если появились новые препятствия!
        if (map_updated) {
            planner_.calculatePath();
        }
    }

    // ЭТО ГЛАВНЫЙ ЦИКЛ УПРАВЛЕНИЯ, который вызывается каждые 100 миллисекунд
    // В нем прогоняются все функции которые раньше был
    void controlLoop()
    {
        if (!odom_received_)
            return;

        double dist_to_goal = hypot(goal_x_meters_ - current_x_, goal_y_meters_ - current_y_);
        if (dist_to_goal < 0.25)
        {
            stopRobot();
            RCLCPP_INFO_ONCE(this->get_logger(), "Финальная цель достигнута!");
            return;
        }

        GridPoint current_grid = metersToGrid(current_x_, current_y_);

        // ВАЖНО: Убран лишний цикл for, так как метод getSmoothTarget УЖЕ
        // ищет точку на 4 шага вперед внутри DStarLite!
        GridPoint lookahead_target = planner_.getSmoothTarget(current_grid);

        auto [target_x, target_y] = gridToMeters(lookahead_target);

        // Целится в эту точку впереди 
        double angle_to_target = atan2(target_y - current_y_, target_x - current_x_);
        double angle_error = angle_to_target - current_yaw_;

        while (angle_error > M_PI)
            angle_error -= 2.0 * M_PI;
        while (angle_error < -M_PI)
            angle_error += 2.0 * M_PI;

        geometry_msgs::msg::Twist cmd;

        // ДЕЛАЕМ РЕГУЛЯТОР (КАРИМ ОТЕЦ КОМПЬЮТЕРА И ГЕНИЙ МОЗГА 2)

        // Скорость зависит от необходимого угла поворота
        // Чем больше нужно повернуть, тем медленнее едем
        double max_linear_speed = 0.3;

        // куб зав-ть для плавного торможения
        // TODO: потыкаться в кэфы и функции хз может быть не куб а квадрат или экспонента 
        cmd.linear.x = max_linear_speed * pow(cos(angle_error / 2.0), 3);
        if (cmd.linear.x < 0.0)
            cmd.linear.x = 0.0;

        // пворачиваем пропорционально ошибке
        cmd.angular.z = 2.0 * angle_error;

        // пределы для 
        if (cmd.angular.z > 1.0)
            cmd.angular.z = 1.0;
        if (cmd.angular.z < -1.0)
            cmd.angular.z = -1.0;

        cmd_pub_->publish(cmd);
    }

    // Функция для остановки робота
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
    auto node = make_shared<NavigationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}