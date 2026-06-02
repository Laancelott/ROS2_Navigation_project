# Навигация робота

## Структура проекта:
```bash
❯ cd ros2_pouk_project
❯ tree

 .
├──  compose
│   └──  docker-compose.yml
├──  docker
│   ├──  gazebo
│   │   └──  Dockerfile
│   └──  navigation
│       └──  Dockerfile
├── 󰂺 README.md
└──  workspace
    └── 󰣞 src
        └──  nav_pkg
            ├──  CMakeLists.txt
            ├──  include
            │   └──  nav_pkg
            ├── 󰗀 package.xml
            └── 󰣞 src

```

МЫ РАБОТАЕМ ТОЛЬКО В workspace/src
ТАМ БУДУТ ВСЕ ПАКЕТЫ ТИПА:

``` bash
workspace/
└── src/
    ├── nav_pkg/            # пакет навигации (C++)
    ├── gazebo_description/ # Пакет с 3D-моделью робота и URDF-файлами
    ├── robot_interfaces/   # Пакет, где хранятся только кастомные топики и сообщения
    └── sensor_driver/      # Пакет для работы с каким-нибудь реальным лидаром
```

КАЖДЫЙ ПАКЕТ СОСТОИТ ИЗ УЗЛОВ (NODE)

```bash
nav_pkg/                     # ОДИН ПАКЕТ (папка на диске)
├── CMakeLists.txt           # Здесь ты пропишешь сборку трех разных бинарников
├── package.xml
└── src/
    ├── planner.cpp          # Код для Node 1
    ├── safety_brake.cpp     # Код для Node 2
    └── follower.cpp         # Код для Node 3
```




## Структура контейнеров

Проект состоит из 2 docker контейнеров

- navigation
	- Содержит:
		- ROS2 Humble
		- C++ пакет для Ros который обрабатывает данные с датчиков
- gazebo
	- Содержит:
		- ROS2
		- Gazebo
		- Датчики хз
        


Они работают параллельно и обмениваются информацией через шину DDS.

# Запуск

#cTODO: Добавить нормальный универсальный launch файл который будет поднимать докер и запускать симуляцию

```bash
cd ros2_pouk_project
docker compose up

```


