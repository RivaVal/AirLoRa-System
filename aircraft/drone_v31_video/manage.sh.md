# Создай файл `manage.sh` в корне проекта (`chmod +x manage.sh`):
#
#!/bin/bash
case "$1" in
  start)
    echo "🚀 Запуск проекта..."
    sudo docker compose up -d
    ;;
  stop)
    echo "🛑 Остановка проекта..."
    sudo docker compose down -v --remove-orphans
    ;;
  logs)
    sudo docker compose logs -f --tail=50
    ;;
  clean)
    echo "🧹 Полная очистка..."
    sudo docker compose down -v --remove-orphans
    sudo docker system prune -af --volumes
    ;;
  *)
    echo "Использование: $0 {start|stop|logs|clean}"
    exit 1
    ;;
esac

