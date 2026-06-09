#pragma once

#include <vector>
#include <cmath>
#include <set>
#include <utility>
#include <limits>
#include <tuple>

// Структура для представления координат клетки на сетке
struct State {
    int x;
    int y;

    bool operator==(const State& other) const {
        return x == other.x && y == other.y;
    }
    bool operator!=(const State& other) const {
        return !(*this == other);
    }

    // Оператор меньше нужен чтобы использовать sstate как ключ в ассоциативных контейнерах
    // Классическое лексикографическое сравнение
    bool operator<(const State& other) const {
        if (x != other.x) {
            return x < other.x;
        }
        return y < other.y;
    }
};

// Двойной ключ сортировки в очереди D* Lite
struct Key {
    double k1;
    double k2;

    // Сравнение ключей по правилам оригинальной статьи (лексикографический порядок)
    bool operator<(const Key& other) const {
        if (k1 != other.k1) return k1 < other.k1;
        return k2 < other.k2;
    }
};

// Элемент нашей очереди: пара из Ключа и Клетки, к которой он относится
struct QueueItem {
    Key key;
    State state;

    bool operator<(const QueueItem& other) const {
        if (key < other.key) return true;
        if (other.key < key) return false;
        return state < other.state; // Если ключи равны, сортируем по координатам
    }
};

class DStarLite {
public:
    // Конструктор: принимает размеры сетки (ширина, высота)
    DStarLite(int width, int height);

    // Главные функции управления
    void init(State start, State goal);
    bool computeShortestPath();
    void updateObstacle(State state, bool is_obstacle);
    
    // Функция для получения следующего шага робота
    State getNextBestState(State current_start);
    std::vector<State> getPath(State current_start);

private:
    // Математические помощники алгоритма
    Key calculateKey(State s);
    void updateVertex(State s);
    double heuristic(State s1, State s2);
    double getCost(State s1, State s2);
    std::vector<State> getNeighbors(State s);

    // Размеры нашей карты
    int width_;
    int height_;

    // Физическое представление карты (true - стена, false - свободно)
    std::vector<std::vector<bool>> grid_;

    // Матрицы стоимостей g и rhs
    std::vector<std::vector<double>> g_;
    std::vector<std::vector<double>> rhs_;

    // Текущие позиции
    State start_;
    State goal_;
    State last_; // Точка, где робот последний раз обновлял карту

    double km_;  // Модификатор ключа (Key Modifier) для экономии пересчетов при движении
    const double INF = std::numeric_limits<double>::infinity();

    // Наша приоритетная очередь (используем set для легкого удаления/обновления элементов)
    std::set<QueueItem> queue_;
};