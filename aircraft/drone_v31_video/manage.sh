# Создай файл `manage.sh` в корне проекта (`chmod +x manage.sh`):
#!/bin/bash
# manage.sh - Надёжное управление проектом Drone Video Pipeline
# Автоматически обрабатывает "зависшие" контейнеры и ошибки Docker-демона.

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER_NAME="rpi-zero2w-dev"
COMPOSE_FILE="${PROJECT_DIR}/docker-compose.yml"

# 🔍 Получение ID контейнера
get_cid() {
    docker ps -a -q -f name=^${CONTAINER_NAME}$ 2>/dev/null
}

# 🏃 Проверка, действительно ли контейнер запущен
is_running() {
    [[ "$(docker inspect -f '{{.State.Running}}' "$1" 2>/dev/null)" == "true" ]]
}

# 🛑 ОСТАНОВКА (каскадный fallback)
stop_container() {
    echo "🛑 Остановка проекта..."
    local cid=$(get_cid)

    if [[ -z "$cid" ]]; then
        echo "ℹ️ Контейнер не найден. Нечего останавливать."
        return 0
    fi

    # 1. Graceful stop (корректное завершение)
    echo "🔄 Попытка корректной остановки..."
    if sudo docker compose -f "$COMPOSE_FILE" down -v --remove-orphans >/dev/null 2>&1; then
        echo "✅ Контейнер остановлен корректно."
        return 0
    fi

    # 2. SIGKILL через Docker API
    echo "⚡ Graceful не сработал. Пробуем SIGKILL..."
    if sudo docker kill -s SIGKILL "$cid" >/dev/null 2>&1 && sudo docker rm -f "$cid" >/dev/null 2>&1; then
        echo "✅ Контейнер принудительно удалён через Docker."
        return 0
    fi

    # 3. Аварийный сброс на уровне ОС (Docker заблокирован)
    echo "🚨 Демон Docker заблокирован. Выполняем аварийную очистку..."
    sudo systemctl stop docker containerd >/dev/null 2>&1
    sleep 2

    # Удаляем метаданные зависшего контейнера (безопасно при остановленных демонах)
    sudo rm -rf /var/lib/docker/containers/${cid}* 2>/dev/null
    sudo rm -rf /var/run/docker/containerd/daemon/io.containerd.runtime.v2.task/moby/${cid}* 2>/dev/null

    sudo systemctl start containerd docker >/dev/null 2>&1
    sleep 4

    # 4. Финальная проверка
    if [[ -z "$(get_cid)" ]]; then
        echo "✅ Аварийная очистка успешна. Контейнер удалён."
    else
        echo "❌ КРИТИЧЕСКАЯ ОШИБКА: Контейнер всё ещё активен. Выполните вручную: sudo docker system prune -af"
        return 1
    fi
}

# 🚀 ЗАПУСК (чистый старт)
start_container() {
    echo "🚀 Запуск проекта..."

    # Гарантируем отсутствие конфликтов
    stop_container || true

    echo "📦 Сборка образа (без кэша, применяются правки Dockerfile)..."
    if ! sudo docker compose -f "$COMPOSE_FILE" build --no-cache; then
        echo "❌ Ошибка сборки. Проверьте синтаксис Dockerfile."
        return 1
    fi

    #----------------------------------------------------------------------
    # В функции start_container, перед up -d:
    if is_running "$(get_cid)"; then
        echo "ℹ️ Контейнер уже запущен и работает. Пропускаю перезапуск."
        echo "🌐 IDE: http://localhost:8080"
        return 0
    fi

    echo "⏳ Запуск контейнера..."
    if ! sudo docker compose -f "$COMPOSE_FILE" up -d; then
        echo "❌ Ошибка запуска. Проверьте логи: sudo docker compose logs"
        return 1
    fi

    sleep 3
    if is_running "$(get_cid)"; then
        echo "✅ Проект успешно запущен!"
        echo "🌐 IDE: http://localhost:8080"
        echo "💡 В терминале IDE выполните: cd /workspace/src && python3 camera_pipeline.py --mock --telemetry --auto-hq 10"
    else
        echo "❌ Контейнер не запустился или упал. Логи: sudo docker compose logs"
        return 1
    fi
}

# 🎛 CLI-обработчик
case "${1:-}" in
    start)    start_container ;;
    stop)     stop_container ;;
    restart)  stop_container; start_container ;;
    status)   sudo docker compose -f "$COMPOSE_FILE" ps ;;
    logs)     sudo docker compose -f "$COMPOSE_FILE" logs -f --tail=50 ;;
    clean)    stop_container; sudo docker system prune -af --volumes ;;
    *)        echo "Использование: $0 {start|stop|restart|status|logs|clean}"; exit 1 ;;
esac
