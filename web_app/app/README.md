# Autonomous Mobile Robot Navigation — Diploma Project

Система навигации автономного мобильного робота для карты производственного предприятия.
Дипломная работа, ФПМ (Факультет прикладного анализа данных).

## Архитектура

```
[PWA on phone]  ←→  [FastAPI server on NucBox K10]  ←→  [ROS 2 topic]  ←→  [Robot]
     HTTP/WS              task queue (SQLite)              rclpy node
     
     ↕  (later)
  [LoRa module]
```

## Структура монорепо

| Каталог         | Назначение                                                          |
|-----------------|---------------------------------------------------------------------|
| `backend/`      | FastAPI сервер + очередь задач SQLite + WebSocket                   |
| `navigation/`   | Планировщики пути: A*, Dijkstra, Theta*, JPS, RRT/RRT*, D* Lite     |
| `pwa/`          | Progressive Web App для вызова робота со смартфона                  |
| `ros2_bridge/`  | Тонкий rclpy-узел, перекладывающий задачи в топик навигации         |
| `experiments/`  | Бенчмарки алгоритмов, статистические сравнения                      |
| `docs/`         | Техническая документация, архитектурные решения                     |

## Быстрый старт (backend)

```bash
cd backend
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

Документация API: <http://localhost:8000/docs>

## Порядок реализации

1. **Backend + task queue** — ядро системы, тестируется без робота (текущий этап).
2. **PWA frontend** — SVG-карта, выбор зоны, статус в реальном времени.
3. **ROS 2 bridge** — публикация в топик навигации.
4. **Navigation core** — сравнение алгоритмов планирования на реальной карте.
5. **Multi-robot + fault tolerance** — Hungarian algorithm, heartbeat.
6. **LoRa transport** — замена HTTP на LoRa без изменения остальных слоёв.
