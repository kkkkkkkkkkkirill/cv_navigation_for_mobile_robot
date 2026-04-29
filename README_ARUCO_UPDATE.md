# amr_stage4_cv_nav

## Что в этой версии

1. **Фикс `GZ_SIM_RESOURCE_PATH`** в `gazebo.launch.py` — теперь URI вида `model://kolestel_rover_description/meshes/...` корректно резолвится. Меши робота больше не теряются.
2. **30 ArUco маркеров расставлены параллельно осям** — по одному на каждом из 30 пересечений графа (5 aisles × 6 cross-rows). Маркер стоит в 0.8 м к востоку от линии и в 1.0 м к северу от узла. **yaw = 0**, плоскость маркера перпендикулярна aisle, лицо смотрит на юг и на север одновременно — виден робото`м едущем в любую сторону вдоль aisle.
3. **`cv_navigator.py` чистый** — без blind drive. POST `/robot/route` сохранён.
4. **`aruco_node.py`** — детекция + solvePnP, центрированные углы, RGB-оси, рамка, ID, расстояние, сдвиги. Интринсики из `/camera/camera_info` D435.
5. **Frontend snap-on-route** — позиция робота на карте проецируется на текущий маршрут (компенсация дрифта одометрии).

## Как запустить

```bash
cd ~/Desktop
rm -rf amr_stage4_cv_nav
unzip ~/Downloads/amr_stage4_cv_nav.zip
cd amr_stage4_cv_nav

# полная очистка кешей
pkill -9 -f gz; pkill -9 -f ros; pkill -9 -f rviz; pkill -9 -f uvicorn
rm -rf ~/.gz/sim ~/.gz/fuel ~/.gazebo /tmp/.gazebo* /tmp/gz_*
rm -rf ros2_ws/build ros2_ws/install ros2_ws/log

# сборка
bash build_ros2.sh

# запуск (в 4 терминалах)
bash run_gazebo_cv_nav.sh
bash web_app/app/backend/run_external_backend.sh
bash web_app/app/ros2_bridge/run_robot_status_bridge.sh
bash web_app/app/ros2_bridge/run_task_nav2_bridge.sh
```

В браузере открой `http://127.0.0.1:8010` и нажми **Ctrl+Shift+R** (жёсткий релоад).

## Если робота не видно в Gazebo

Спавн робота происходит через 5-7 секунд **после открытия окна Gazebo**, не сразу. В логе ты должен увидеть:

```
[ros_gz_sim]: Entity creation successful.
```

После этого:
1. В правой панели Gazebo раскрой **Entity Tree**
2. Раскрой `default` (стрелочка слева)
3. Прокрути список вниз — `kolestel_rover` будет в самом низу (после всех `aws_robomaker_*` и `aruco_marker_*`)
4. Кликни по нему правой кнопкой → **Move to**

Если в Entity Tree пусто или нет `kolestel_rover` — значит spawn не прошёл, пришли логи и я посмотрю.

## Если бэшрук жалуется на старый workspace

Если при открытии каждого терминала пишет:
```
bash: /home/rover/ros2_ws/install/setup.bash: No such file or directory
```

Это в `~/.bashrc` старая строка от другого workspace. Закомментируй её:
```bash
sed -i 's|^source /home/rover/ros2_ws/install/setup.bash|# &|' ~/.bashrc
```

## Как увидеть детекцию ArUco в RViz

`rviz2` → Add → Image → Topic `/aruco/debug_image`

Когда робот подъезжает к перекрёстку, в кадре появляются:
- зелёная рамка вокруг каждого маркера
- `id:N  d=X.XXm` (id и евклидово расстояние)
- `0.45m left/right`, `0.05m above/below`
- `fwd=X.XXm` (forward distance)
- RGB-оси из центра маркера

В углу всегда `markers detected: N`.
