#include "nav_pkg/d_star_lite.hpp"
#include <algorithm>
#include <iostream>

// Конструктор: создаем карту и заполняем её "пустыми" клетками
DStarLite::DStarLite(int width, int height) 
    : map_width_(width), map_height_(height), accumulated_robot_movement_(0.0) 
{
    map_.assign(map_width_, std::vector<CellMemory>(map_height_));
}

// Первоначальная настройка перед началом движения
void DStarLite::setStartAndGoal(GridCoordinate start, GridCoordinate goal) {
    robot_start_ = start;
    goal_target_ = goal;
    last_robot_pos_ = robot_start_;
    accumulated_robot_movement_ = 0.0;

    processing_queue_.clear();
    
    // Сбрасываем всю память карты
    for (int x = 0; x < map_width_; ++x) {
        for (int y = 0; y < map_height_; ++y) {
            map_[x][y].g = INFINITY_COST;
            map_[x][y].rhs = INFINITY_COST;
        }
    }

    // Инициализация от цели: расстояние от цели до самой цели равно нулю
    map_[goal_target_.x][goal_target_.y].rhs = 0.0;
    
    // Добавляем цель в очередь, чтобы волна расчетов пошла от неё
    QueueItem item;
    item.key = calculatePriorityKey(goal_target_);
    item.coord = goal_target_;
    processing_queue_.insert(item);
}

// Расчет ключа сортировки (насколько срочно нужно обработать эту клетку)
PriorityKey DStarLite::calculatePriorityKey(GridCoordinate cell) {
    double min_cost = std::min(map_[cell.x][cell.y].g, map_[cell.x][cell.y].rhs);
    
    PriorityKey key;
    // Главный приоритет = минимальная стоимость + эвристика (расстояние до робота) + поправка на движение
    key.primary_priority = min_cost + getHeuristicDistance(cell, robot_start_) + accumulated_robot_movement_;
    
    // Второстепенный приоритет = просто стоимость клетки
    key.secondary_priority = min_cost;
    
    return key;
}

// Расстояние Чебышёва: позволяет роботу двигаться по диагоналям
double DStarLite::getHeuristicDistance(GridCoordinate a, GridCoordinate b) {
    return std::max(std::abs(a.x - b.x), std::abs(a.y - b.y));
}

// Стоимость перехода между двумя соседними клетками
double DStarLite::getTransitionCost(GridCoordinate from, GridCoordinate to) {
    // Если пытаемся шагнуть в стену или из стены - прохода нет
    if (map_[from.x][from.y].is_wall || map_[to.x][to.y].is_wall) {
        return INFINITY_COST;
    }
    // Обычный шаг стоит 1.0
    return 1.0;
}

// Обновление состояния конкретной клетки (проверка соседей)
void DStarLite::updateCellState(GridCoordinate cell) {
    // Если это не цель, пересчитываем её предсказанную стоимость (rhs)
    if (cell != goal_target_) {
        double best_rhs = INFINITY_COST;
        
        // Ищем самого "дешевого" соседа для прохода к цели
        for (const auto& neighbor : getValidNeighbors(cell)) {
            double cost_to_neighbor = getTransitionCost(cell, neighbor);
            if (cost_to_neighbor != INFINITY_COST && map_[neighbor.x][neighbor.y].g != INFINITY_COST) {
                double total_cost = cost_to_neighbor + map_[neighbor.x][neighbor.y].g;
                best_rhs = std::min(best_rhs, total_cost);
            }
        }
        map_[cell.x][cell.y].rhs = best_rhs;
    }

    // Удаляем клетку из очереди, если она там зависла со старыми приоритетами
    for (auto it = processing_queue_.begin(); it != processing_queue_.end(); ) {
        if (it->coord == cell) {
            it = processing_queue_.erase(it);
        } else {
            ++it;
        }
    }

    // Если клетка "несогласована" (g != rhs), возвращаем её в очередь на пересчет с новым ключом
    if (map_[cell.x][cell.y].g != map_[cell.x][cell.y].rhs) {
        QueueItem item;
        item.key = calculatePriorityKey(cell);
        item.coord = cell;
        processing_queue_.insert(item);
    }
}

// Главный математический двигатель: пересчет стоимостей на карте
bool DStarLite::calculatePathCosts() {
    if (processing_queue_.empty()) return false;

    // Цикл разбит на понятные переменные для легкого чтения
    while (true) {
        if (processing_queue_.empty()) break;

        PriorityKey best_key_in_queue = processing_queue_.begin()->key;
        PriorityKey robot_pos_key = calculatePriorityKey(robot_start_);
        
        bool robot_needs_better_path = (best_key_in_queue < robot_pos_key);
        bool robot_pos_inconsistent = (map_[robot_start_.x][robot_start_.y].rhs != map_[robot_start_.x][robot_start_.y].g);

        // Если путь к роботу оптимален и позиция согласована - выходим из цикла, путь готов!
        if (!robot_needs_better_path && !robot_pos_inconsistent) {
            break;
        }

        // Извлекаем самую приоритетную клетку
        GridCoordinate current_cell = processing_queue_.begin()->coord;
        PriorityKey old_key = processing_queue_.begin()->key;
        processing_queue_.erase(processing_queue_.begin());

        PriorityKey new_key = calculatePriorityKey(current_cell);
        
        // Ситуация 1: Ключ устарел, пока лежал в очереди. Просто обновляем его.
        if (old_key < new_key) {
            QueueItem item;
            item.key = new_key;
            item.coord = current_cell;
            processing_queue_.insert(item);
        }
        // Ситуация 2: Путь стал дешевле (клетка избыточно согласована)
        else if (map_[current_cell.x][current_cell.y].g > map_[current_cell.x][current_cell.y].rhs) {
            // Фиксируем новую, более дешевую цену
            map_[current_cell.x][current_cell.y].g = map_[current_cell.x][current_cell.y].rhs;
            
            // Сообщаем всем соседям, что через нас теперь идти выгодно
            for (const auto& neighbor : getValidNeighbors(current_cell)) {
                updateCellState(neighbor);
            }
        }
        // Ситуация 3: Путь стал дороже, например появилась стена (недостаточно согласована)
        else {
            // Сбрасываем старую стоимость
            map_[current_cell.x][current_cell.y].g = INFINITY_COST;
            
            // Пересчитываем саму клетку и всех её соседей
            updateCellState(current_cell);
            for (const auto& neighbor : getValidNeighbors(current_cell)) {
                updateCellState(neighbor);
            }
        }
    }
    
    // Возвращаем true, если к роботу можно проложить путь (путь не бесконечен)
    return map_[robot_start_.x][robot_start_.y].rhs != INFINITY_COST;
}

// Реакция на данные лидара: появилось или исчезло препятствие
void DStarLite::updateMapObstacle(GridCoordinate target_cell, bool has_obstacle) {
    // Если статус стены не изменился, ничего не делаем
    if (map_[target_cell.x][target_cell.y].is_wall == has_obstacle) {
        return; 
    }

    // Записываем новое состояние стены
    map_[target_cell.x][target_cell.y].is_wall = has_obstacle;

    // Учитываем расстояние, которое робот проехал с прошлого обновления
    accumulated_robot_movement_ += getHeuristicDistance(last_robot_pos_, robot_start_);
    last_robot_pos_ = robot_start_;

    // Обновляем саму клетку и сообщаем её соседям об изменении
    updateCellState(target_cell);
    for (const auto& neighbor : getValidNeighbors(target_cell)) {
        updateCellState(neighbor);
    }

    // Запускаем пересчет только измененных участков карты
    calculatePathCosts();
}

// Запрос от ROS2: в какую соседнюю клетку роботу поехать прямо сейчас?
GridCoordinate DStarLite::getNextBestStep(GridCoordinate current_robot_pos) {
    robot_start_ = current_robot_pos;
    
    double min_cost = INFINITY_COST;
    GridCoordinate best_step = robot_start_;

    // Перебираем всех соседей вокруг робота и выбираем того, от которого путь до цели самый дешевый
    for (const auto& neighbor : getValidNeighbors(robot_start_)) {
        double cost_to_move = getTransitionCost(robot_start_, neighbor);
        double total_path_cost = cost_to_move + map_[neighbor.x][neighbor.y].g;
        
        if (total_path_cost < min_cost) {
            min_cost = total_path_cost;
            best_step = neighbor;
        }
    }
    return best_step;
}

// Вспомогательная функция: получить список соседей клетки (до 8 штук)
std::vector<GridCoordinate> DStarLite::getValidNeighbors(GridCoordinate cell) {
    std::vector<GridCoordinate> neighbors;
    
    // Перебираем смещения от -1 до 1 по обеим осям (сетка 3х3 вокруг клетки)
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) continue; // Пропускаем саму себя
            
            int nx = cell.x + dx;
            int ny = cell.y + dy;

            // Проверяем, что не вышли за границы нашей карты
            if (nx >= 0 && nx < map_width_ && ny >= 0 && ny < map_height_) {
                neighbors.push_back(GridCoordinate{nx, ny});
            }
        }
    }
    return neighbors;
}