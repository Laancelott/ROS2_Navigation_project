#pragma once

#include <vector>
#include <cmath>
#include <set>
#include <utility>
#include <limits>
#include <tuple>

using namespace std;

// Константа для обозначения бесконечности (непроходимая стена или неизвестный путь)
const double INFINITY_COST = numeric_limits<double>::infinity();

// Структура координат клетки на нашей карте
struct GridCoordinate {
    int x;
    int y;

    // Базовые операторы сравнения
    bool operator==(const GridCoordinate& other) const {
        return (x == other.x) && (y == other.y);
    }
    bool operator!=(const GridCoordinate& other) const {
        return !(*this == other);
    }

    // Оператор '<' нужен для использования координат в качестве ключа в set
    bool operator<(const GridCoordinate& other) const {
        if (x != other.x) return x < other.x;
        return y < other.y;
    }
};

// Память робота о конкретной клетке. 
// Вместо трех разных матриц мы храним все данные в одном месте.
struct CellMemory {
    double g;       // Точная (уже подтвержденная) стоимость пути от этой клетки до цели
    double rhs;     // Оценочная (предсказанная) стоимость, основанная на соседях
    bool is_wall;   // Физическое препятствие (true - стена, false - свободно)

    // По умолчанию клетка пустая, а путь через неё неизвестен (бесконечность)
    CellMemory() : g(INFINITY_COST), rhs(INFINITY_COST), is_wall(false) {}
};

// Ключ приоритета для сортировки клеток в очереди на обработку
struct PriorityKey {
    double primary_priority;   // Главный приоритет (учитывает эвристику до робота)
    double secondary_priority; // Второстепенный приоритет (просто стоимость пути)

    // Сравнение ключей: сначала смотрим на главный приоритет, если равны — на второстепенный
    bool operator<(const PriorityKey& other) const {
        if (primary_priority != other.primary_priority) {
            return primary_priority < other.primary_priority;
        }
        return secondary_priority < other.secondary_priority;
    }
};

// Элемент очереди: хранит координаты клетки и её текущий приоритет
struct QueueItem {
    PriorityKey key;
    GridCoordinate coord;

    bool operator<(const QueueItem& other) const {
        if (key < other.key) return true;
        if (other.key < key) return false;
        return coord < other.coord; // Защита от одинаковых ключей: сортируем по координатам
    }
};

class DStarLite {
public:
    // Инициализация карты заданного размера
    DStarLite(int width, int height);

    // Главный интерфейс для управления снаружи (из ROS2 ноды)
    void setStartAndGoal(GridCoordinate start, GridCoordinate goal);
    bool calculatePathCosts();
    void updateMapObstacle(GridCoordinate target_cell, bool has_obstacle);
    
    // Получение результата: куда ехать дальше
    GridCoordinate getNextBestStep(GridCoordinate current_robot_pos);

private:
    // Внутренние методы алгоритма D* Lite
    PriorityKey calculatePriorityKey(GridCoordinate cell);
    void updateCellState(GridCoordinate cell);
    
    double getHeuristicDistance(GridCoordinate a, GridCoordinate b);
    double getTransitionCost(GridCoordinate from, GridCoordinate to);
    vector<GridCoordinate> getValidNeighbors(GridCoordinate cell);

    // Параметры карты
    int map_width_;
    int map_height_;

    // Единая карта мира, где каждая ячейка хранит свои g, rhs и статус стены
    vector<vector<CellMemory>> map_;

    // Важные координаты
    GridCoordinate robot_start_;
    GridCoordinate goal_target_;
    GridCoordinate last_robot_pos_; // Позиция робота при последнем пересчете карты

    // Накопленное смещение робота (нужно для математики D*, чтобы не пересчитывать всю очередь)
    double accumulated_robot_movement_;  

    // Очередь клеток, которые требуют пересчета (несогласованные клетки)
    set<QueueItem> processing_queue_;
};