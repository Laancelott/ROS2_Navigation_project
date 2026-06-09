#include "d_star_lite.hpp"
#include <algorithm>
#include <iostream>

DStarLite::DStarLite(int width, int height) 
    : width_(width), height_(height), km_(0.0) 
{
    grid_.assign(width, std::vector<bool>(height, false));
    g_.assign(width, std::vector<double>(height, INF));
    rhs_.assign(width, std::vector<double>(height, INF));
}

// 1. Инициализация (Пункт "Initialize()" из статьи)
void DStarLite::init(State start, State goal) {
    start_ = start;
    goal_ = goal;
    last_ = start_;
    km_ = 0.0;

    queue_.clear();
    
    // Заполняем матрицы бесконечностями
    for (int x = 0; x < width_; ++x) {
        for (int y = 0; y < height_; ++y) {
            g_[x][y] = INF;
            rhs_[x][y] = INF;
        }
    }

    // Так как мы ищем путь от Goal к Start, базовое условие ставится на цель
    rhs_[goal_.x][goal_.y] = 0.0;
    
    // Кладем целевую точку в очередь
    QueueItem item;
    item.key = calculateKey(goal_);
    item.state = goal_;
    queue_.insert(item);
}

// 2. Расчет ключа сортировки (Пункт "CalculateKey(s)")
Key DStarLite::calculateKey(State s) {
    double min_g_rhs = std::min(g_[s.x][s.y], rhs_[s.x][s.y]);
    
    Key k;
    // k1 = минимальная цена + эвристика до робота + накопленное смещение km
    k.k1 = min_g_rhs + heuristic(s, start_) + km_;
    // k2 = просто минимальное знание о цене клетки
    k.k2 = min_g_rhs;
    
    return k;
}

// 3. Эвристика (В нашем случае — обычное расстояние Чебышёва для сетки с диагоналями)
double DStarLite::heuristic(State s1, State s2) {
    return std::max(std::abs(s1.x - s2.x), std::abs(s1.y - s2.y));
}

// 4. Стоимость шага между соседними клетками c(s, s')
double DStarLite::getCost(State s1, State s2) {
    // Если любая из клеток заблокирована — прохода нет (цена бесконечна)
    if (grid_[s1.x][s1.y] || grid_[s2.x][s2.y]) {
        return INF;
    }
    // Стоимость шага по диагонали или прямой примем за 1.0 ради простоты сетки
    return 1.0;
}

// 5. Функция обновления состояния клетки (Пункт "UpdateVertex(u)")
void DStarLite::updateVertex(State s) {
    // Если клетка не является целью, пересчитываем её rhs на основе соседей
    if (s != goal_) {
        double min_rhs = INF;
        for (const auto& next : getNeighbors(s)) {
            double cost = getCost(s, next);
            if (cost != INF && g_[next.x][next.y] != INF) {
                min_rhs = std::min(min_rhs, cost + g_[next.x][next.y]);
            }
        }
        rhs_[s.x][s.y] = min_rhs;
    }

    // Удаляем клетку из очереди, если она там уже была (чтобы обновить её позицию)
    for (auto it = queue_.begin(); it != queue_.end(); ) {
        if (it->state == s) {
            it = queue_.erase(it);
        } else {
            ++it;
        }
    }

    // Если клетка стала несогласованной (g != rhs), возвращаем её в очередь с новым ключом
    if (g_[s.x][s.y] != rhs_[s.x][s.y]) {
        QueueItem item;
        item.key = calculateKey(s);
        item.state = s;
        queue_.insert(item);
    }
}

// 6. Основной цикл планирования пути (Пункт "ComputeShortestPath()")
bool DStarLite::computeShortestPath() {
    if (queue_.empty()) return false;

    // Крутим цикл, пока минимальный ключ в очереди меньше ключа нашего старта,
    // ИЛИ пока наш старт не станет полностью согласованным (rhs == g)
    while (!queue_.empty() && 
          (queue_.begin()->key < calculateKey(start_) || rhs_[start_.x][start_.y] != g_[start_.x][start_.y])) 
    {
        // Берем самый приоритетный элемент и удаляем его из очереди
        State u = queue_.begin()->state;
        Key k_old = queue_.begin()->key;
        queue_.erase(queue_.begin());

        Key k_new = calculateKey(u);
        
        // Если за время лежания в очереди ключ устарел — просто обновляем элемент
        if (k_old < k_new) {
            QueueItem item;
            item.key = k_new;
            item.state = u;
            queue_.insert(item);
        }
        // Случай А: Клетка избыточно согласована (путь стал короче)
        else if (g_[u.x][u.y] > rhs_[u.x][u.y]) {
            g_[u.x][u.y] = rhs_[u.x][u.y];
            for (const auto& s : getNeighbors(u)) {
                updateVertex(s);
            }
        }
        // Случай Б: Клетка недостаточно согласована (появилось препятствие)
        else {
            g_[u.x][u.y] = INF;
            updateVertex(u);
            for (const auto& s : getNeighbors(u)) {
                updateVertex(s);
            }
        }
    }
    return rhs_[start_.x][start_.y] != INF;
}

// 7. Функция вызова при обнаружении препятствия лидаром
void DStarLite::updateObstacle(State state, bool is_obstacle) {
    if (grid_[state.x][state.y] == is_obstacle) return; // Ничего не изменилось

    grid_[state.x][state.y] = is_obstacle;

    // Рассчитываем km: добавляем пройденное роботом расстояние с момента последнего завала
    km_ += heuristic(last_, start_);
    last_ = start_;

    // Обновляем саму измененную клетку и всех её соседей вокруг
    updateVertex(state);
    for (const auto& s : getNeighbors(state)) {
        updateVertex(s);
    }

    // Запускаем локальный пересчет графа
    computeShortestPath();
}

// 8. Выбор лучшего соседа для совершения физического шага робота
State DStarLite::getNextBestState(State current_start) {
    start_ = current_start;
    
    double min_cost = INF;
    State best_state = start_;

    for (const auto& next : getNeighbors(start_)) {
        double cost = getCost(start_, next) + g_[next.x][next.y];
        if (cost < min_cost) {
            min_cost = cost;
            best_state = next;
        }
    }
    return best_state;
}

// Вспомогательная функция получения 8-связных соседей (с учетом диагоналей)
std::vector<State> DStarLite::getNeighbors(State s) {
    std::vector<State> neighbors;
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) continue;
            
            int nx = s.x + dx;
            int ny = s.y + dy;

            if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
                neighbors.push_back(State{nx, ny});
            }
        }
    }
    return neighbors;
}