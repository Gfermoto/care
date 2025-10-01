#!/bin/bash
set -euo pipefail

PORT=${1:-/dev/ttyUSB0}
BAUD=${2:-115200}
CMD=${3:-}

if ! command -v stty >/dev/null; then
  echo "stty not found" >&2
  exit 1
fi

if [ -z "$CMD" ]; then
  echo "Usage: $0 <port> [baud] <command>" >&2
  echo "Example: $0 /dev/ttyUSB0 115200 AT+VERSION" >&2
  exit 1
fi

stty -F "$PORT" $BAUD cs8 -cstopb -parenb -ixon -ixoff -crtscts -echo
# Добавляем CRLF
printf "%s\r\n" "$CMD" > "$PORT"
# Ждем и читаем ответ (1 сек)
sleep 1
if timeout 2 cat "$PORT" | head -n 20; then
  exit 0
else
  exit 0
fi
