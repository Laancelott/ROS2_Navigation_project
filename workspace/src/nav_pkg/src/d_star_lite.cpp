#include "nav_pkg/d_star_lite.hpp"
#include <algorithm>

using namespace std;

DStarLite::DStarLite(int width, int height)
    : width_(width), height_(height), movement_offset_(0.0)
{
    // Заполняем карту пустыми клетками
    // базовая инициализация карты просто пусытими данными


    map_.assign(width_, vector<CellData>(height_));
}

void DStarLite::setStartAndGoal(GridPoint start, GridPoint goal)
{

    // инит стартовой позиции и цели алгоритма
    start_pos_ = start;
    goal_pos_ = goal;
    last_calc_pos_ = start;
    movement_offset_ = 0.0;
    queue_.clear();

    // сбрасываем карту для нового расчета
    for (int x = 0; x < width_; ++x)
    {
        for (int y = 0; y < height_; ++y)
        {
            map_[x][y].g = INF;
            map_[x][y].rhs = INF;
        }
    }

    // нициализируем целевую клетку
    map_[goal.x][goal.y].rhs = 0.0;
    queue_.insert({calcKey(goal), goal});
}

// === ИСПРАВЛЕНИЕ: Разделение установки препятствия и пересчета пути ===
SortKey DStarLite::calcKey(GridPoint p)
{
    double min_cost = min(map_[p.x][p.y].g, map_[p.x][p.y].rhs);
    return {
        min_cost + heuristic(p, start_pos_) + movement_offset_,
        min_cost};

    // heuristic(p, start_pos_) - это оценка расстояния от p до стартовой позиции.
    // movement_offset_ - это компенсация для учета смещения робота, чтобы алгоритм
}

double DStarLite::heuristic(GridPoint a, GridPoint b)
{
    // расстояние чебышева, потому что робот может двигаться по диагонали
    return max(abs(a.x - b.x), abs(a.y - b.y));
}

// берет стоимость между соседними клетками клетками с уч стен
// серега лох)
double DStarLite::getCost(GridPoint a, GridPoint b)
{
    if (map_[a.x][a.y].is_wall || map_[b.x][b.y].is_wall)
        return INF;

    int dx = abs(a.x - b.x);
    int dy = abs(a.y - b.y);

    if (dx == 1 && dy == 1)
    {
        // запрет срезания углов через стену
        if (map_[a.x][b.y].is_wall || map_[b.x][a.y].is_wall)
            return INF;
        return 1.4142; // sqrt(2) впадлу прописать было да
    }
    return 1.0;
}

void DStarLite::updateVertex(GridPoint p)
{
    if (p != goal_pos_)
    {
        double min_rhs = INF;
        for (const auto &n : getNeighbors(p))
        {
            double cost = getCost(p, n);
            if (cost != INF && map_[n.x][n.y].g != INF)
            {
                min_rhs = min(min_rhs, cost + map_[n.x][n.y].g);
            }
        }
        map_[p.x][p.y].rhs = min_rhs;
    }

    // чистим очередь от старых записей для клетки p
    for (auto it = queue_.begin(); it != queue_.end();)
    {
        if (it->point == p)
            it = queue_.erase(it);
        else
            ++it;
    }

    // если клетка стала неконсистентной то 
    // эта дура идет обратно в очоредь на пересчет

    if (map_[p.x][p.y].g != map_[p.x][p.y].rhs)
    {
        queue_.insert({calcKey(p), p});
    }
}

// эта функция выполняет основной один цикл дстара
// возвращает true если путь найден, false если нет пути

// Она выхывается в двух местах:
// 1) после установки начальной и конечной точки, чтобы найти первый путь
// 2) после изменения карты (установка стены), чтобы обновить путь

bool DStarLite::calculatePath()
{

    if (queue_.empty())
        return false;

    while (!queue_.empty())
    {
        // смотрим на элемент с наивысшим приоритетом в очереди
        SortKey top_key = queue_.begin()->key;

        // сравниваем его с ключом стартовой позиции
        SortKey start_key = calcKey(start_pos_);

        bool need_update = (top_key < start_key);
        bool is_inconsistent = (map_[start_pos_.x][start_pos_.y].rhs != map_[start_pos_.x][start_pos_.y].g);

        if (!need_update && !is_inconsistent)
            break; // путь найден радуемся и идем дальше
            // можно пивка зарядить на радостях

        // извлекаем элемент с наивысшим приоритетом для обработки
        GridPoint p = queue_.begin()->point;

        // ИСПРАВЛЕНИЕ: Сравнение ключей должно быть до извлечения элемента из очереди
        SortKey old_key = queue_.begin()->key;


        // удаляем его из очереди чтоб потом ес надо обновить если нужно
        // серега лох
        queue_.erase(queue_.begin());

        SortKey new_key = calcKey(p);

        if (old_key < new_key)
        {
            queue_.insert({new_key, p});
        }
        else if (map_[p.x][p.y].g > map_[p.x][p.y].rhs)
        {
            map_[p.x][p.y].g = map_[p.x][p.y].rhs;
            for (const auto &n : getNeighbors(p))
                updateVertex(n);
        }
        else
        {
            map_[p.x][p.y].g = INF;
            updateVertex(p);
            for (const auto &n : getNeighbors(p))
                updateVertex(n);
        }
    }
    return map_[start_pos_.x][start_pos_.y].rhs != INF;
}

// Обновляем стену. ВАЖНО!!!!! больше не вызываем тут calculatePath() потому что это может быть очень долго и мы не хотим тормозить робота в момент установки стены.
void DStarLite::setObstacle(GridPoint p, bool is_wall)
{
    if (map_[p.x][p.y].is_wall == is_wall)
        return;

    map_[p.x][p.y].is_wall = is_wall;
    movement_offset_ += heuristic(last_calc_pos_, start_pos_);
    last_calc_pos_ = start_pos_;

    updateVertex(p);
    for (const auto &n : getNeighbors(p))
        updateVertex(n);
}

bool DStarLite::isWall(GridPoint p) const
{
    if (p.x < 0 || p.x >= width_ || p.y < 0 || p.y >= height_)
        return true;
    return map_[p.x][p.y].is_wall;
}

// ввыбор целевой точки с напасом
GridPoint DStarLite::getSmoothTarget(GridPoint robot_pos)
{
    start_pos_ = robot_pos;
    GridPoint current = robot_pos;
    GridPoint target = current;

    // Ищем точку на 30-40 см впереди по опт пути
    //
    // Робот перестанет вилять задом и начнет резать углы плавно.
    for (int i = 0; i < 4; ++i)
    {
        double min_cost = INF;
        GridPoint best_next = current;

        for (const auto &n : getNeighbors(current))
        {
            double step_cost = getCost(current, n);
            if (step_cost == INF)
                continue;

            double total_cost = step_cost + map_[n.x][n.y].g;
            if (total_cost < min_cost)
            {
                min_cost = total_cost;
                best_next = n;
            }
        }

        if (best_next == current)
            break; // Уперлись
        current = best_next;
        target = current;
    }
    return target;
}

// Это функция тупл возвращает все соседние клетки для данной клетки

vector<GridPoint> DStarLite::getNeighbors(GridPoint p)
{
    vector<GridPoint> neighbors;
    for (int dx = -1; dx <= 1; ++dx)
    {
        for (int dy = -1; dy <= 1; ++dy)
        {
            if (dx == 0 && dy == 0)
                continue;
            int nx = p.x + dx;
            int ny = p.y + dy;
            if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_)
            {
                neighbors.push_back({nx, ny});
            }
        }
    }
    return neighbors;
}