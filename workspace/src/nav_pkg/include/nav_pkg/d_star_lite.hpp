#pragma once

#include <vector>
#include <set>
#include <cmath>
#include <limits>

using namespace std;


// сразу объявим бескончн как константу

const double INF = numeric_limits<double>::infinity();

// структура для координат на сетке
struct GridPoint {
    int x;
    int y;

    // Операторы для сравнения точек (нужны для хранения в set)
    bool operator==(const GridPoint& other) const { return x == other.x && y == other.y; }
    bool operator!=(const GridPoint& other) const { return !(*this == other); }
    bool operator<(const GridPoint& other) const {
        if (x != other.x) return x < other.x;
        return y < other.y;
    }
};


// !!!!!!!!!!!
// робот запоминает данные о каждой клетке в виде этой стурктуры
struct CellData {
    double g = INF;       // Точная стоимость до цели
    double rhs = INF;     // Оценочная стоимость по соседям
    bool is_wall = false; // Это стена?
};

// Ключ приоритета для очереди расчетов
// тупо две цифры сначала чекаем одну потом другую для сортировки
struct SortKey {
    double primary;
    double secondary;

    bool operator<(const SortKey& other) const {
        if (primary != other.primary) return primary < other.primary;
        return secondary < other.secondary;
    }
};

// Элемент очереди на пересчет
// Просто по сути картеж из ключа и координатыы
struct QueueItem {
    SortKey key;
    GridPoint point;

    bool operator<(const QueueItem& other) const {
        if (key < other.key) return true;
        if (other.key < key) return false;
        return point < other.point;
    }
};


// КЛАСС D* ЛАЙТ
class DStarLite {
public:

    DStarLite(int width, int height);

    void setStartAndGoal(GridPoint start, GridPoint goal);
    bool calculatePath();
    
    // Важно: эта функция теперь только ставит стену, но не запускает долгий пересчет!
    void setObstacle(GridPoint p, bool is_wall);
    
    // Возвращает точку чуть впереди, чтобы робот ехал плавно
    GridPoint getSmoothTarget(GridPoint robot_pos);

    // Проверка, стена ли это (нужно для лидара)
    bool isWall(GridPoint p) const;

private:
    // ширнина и высота карты в клетках
    int width_;
    int height_;

    // КАРТА ЭТО ВЕКТОР ИЩ ВЕКТОРА ИЗ ДАННЫХ О КЛЕТЕКА
    // МАТРИЦА ДАННЫХ О КЛЕТКАХ
    vector<vector<CellData>> map_;


    GridPoint start_pos_;
    GridPoint goal_pos_;
    GridPoint last_calc_pos_;

    double movement_offset_; // компенсация смещения робота для алгоритма
    // ХЗ вообще это костыль наверное
    //
    // Очередь для обработки клеток при пересчете пути
    set<QueueItem> queue_;

    // Внутренние функции алгоритма

    // написаны в d_star_lite.cpp

    
    SortKey calcKey(GridPoint p);
    void updateVertex(GridPoint p);
    double heuristic(GridPoint a, GridPoint b);
    double getCost(GridPoint a, GridPoint b);
    vector<GridPoint> getNeighbors(GridPoint p);
};