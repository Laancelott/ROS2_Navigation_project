#!/bin/bash

# Путь к папке с compose относительно корня проекта
COMPOSE_DIR="compose"

echo "Остановка навигационного проекта..."

# Проверяем, существует ли папка
if [ -d "$COMPOSE_DIR" ]; then
    cd "$COMPOSE_DIR" || exit
    docker compose down
    echo "Проект остановлен."
else
    echo "Ошибка: Папка $COMPOSE_DIR не найдена в $(pwd)"
    exit 1
fi