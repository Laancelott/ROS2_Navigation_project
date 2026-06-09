#!/bin/bash

# 1. Разрешаем локальные подключения к X-серверу
xhost +local:root > /dev/null

# 2. Создаем изолированный и универсальный токен авторизации для Docker
XKEYS=$(xauth list "$DISPLAY" 2>/dev/null)
export XCOMPOSE_AUTH=/tmp/.docker.xauth
rm -f "$XCOMPOSE_AUTH"

if [ -n "$XKEYS" ]; then
    xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f "$XCOMPOSE_AUTH" nmerge -
else
    touch "$XCOMPOSE_AUTH"
fi
chmod 644 "$XCOMPOSE_AUTH"

# 3. Запускаем docker compose, передавая правильный путь к созданному файлу
cd compose
XCOMPOSE_AUTH=$XCOMPOSE_AUTH docker compose up -d

echo "=== Контейнеры запущены! Gazebo открывается... ==="