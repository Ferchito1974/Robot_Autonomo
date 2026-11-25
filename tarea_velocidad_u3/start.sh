#!/bin/bash
# Starts the web interface in the background (survives closing the SSH session).
#   ./start.sh        real robot      ./start.sh demo   virtual robot (no hardware)
# Log: app.log   PID: app.pid
cd "$(dirname "$0")"
PY=../tarea_odometria/venv/bin/python
if [ -f app.pid ] && kill -0 "$(cat app.pid)" 2>/dev/null; then
    echo "La app ya esta corriendo (PID $(cat app.pid)). Usa ./stop.sh primero."; exit 0
fi
if [ "$1" == "demo" ]; then export ROBOT_DEMO=1; echo "Modo DEMO: robot virtual"; fi
nohup $PY -u app.py > app.log 2>&1 &
echo $! > app.pid
sleep 6
if kill -0 "$(cat app.pid)" 2>/dev/null; then
    echo "App iniciada (PID $(cat app.pid))."
else
    echo "La app no arranco. Ultimas lineas de app.log:"; tail -15 app.log; exit 1
fi
IP=$(hostname -I | awk '{print $1}')
echo "Abre en la laptop:  http://$(hostname).local:5002   o   http://$IP:5002"
grep -m1 "error de conexion" app.log
