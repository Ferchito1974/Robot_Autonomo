#!/bin/bash
# Stops the web interface (the app sends a motor stop on exit).
cd "$(dirname "$0")"
if [ -f app.pid ] && kill -0 "$(cat app.pid)" 2>/dev/null; then
    kill "$(cat app.pid)"; sleep 1; echo "App detenida."
else
    echo "La app no estaba corriendo."
fi
rm -f app.pid
