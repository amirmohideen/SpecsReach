#!/usr/bin/env bash
# Start the arm bridge (the program that drives the AUBO) in the background.
# It keeps running, and comes back after a reboot, until you stop it.
#
#   station/start_bridge.sh                              # the AUBO: station/arm_bridge.yaml
#   CONFIG=station/mock.yaml SIGNALING_PORT=8454 station/start_bridge.sh   # test, no hardware
#
#   logs:  docker logs -f specsreach-arm      (the test one: specsreach-arm-mock)
#   stop:  docker rm -f specsreach-arm
set -euo pipefail
cd "$(dirname "$0")/.."

CONFIG="${CONFIG:-station/arm_bridge.yaml}"
PORT="${SIGNALING_PORT:-8444}"
IMAGE="${IMAGE:-specsreach-station}"
# The real bridge is specsreach-arm and restarts with the PC. Any other config
# (a test) gets its own name and no restart, so it can never replace the real one.
if [ "$CONFIG" = "station/arm_bridge.yaml" ]; then
    NAME="${NAME:-specsreach-arm}"; RESTART=unless-stopped
else
    NAME="${NAME:-specsreach-arm-$(basename "$CONFIG" .yaml)}"; RESTART=no
fi

[ -f "$CONFIG" ] || { echo "no config at $CONFIG" >&2; exit 1; }
case "$CONFIG" in /*) echo "CONFIG must be a path inside the repo (got $CONFIG)" >&2; exit 1;; esac
if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "image $IMAGE not found; build it first: docker build -t $IMAGE station/" >&2
    exit 1
fi

# Replace our own earlier container, then make sure nothing else holds the
# port: two bridges on one arm would fight over the controller.
docker rm -f "$NAME" >/dev/null 2>&1 || true
if ss -Hltn "sport = :$PORT" | grep -q .; then
    echo "port $PORT is already in use: another arm bridge is running (see docker ps)." >&2
    echo "Use that bridge (the relay works with it), or stop it first." >&2
    exit 1
fi

mkdir -p logs/journal
docker run -d --name "$NAME" --network host --restart "$RESTART" \
    --user "$(id -u):$(id -g)" -e HOME=/tmp \
    -v "$PWD":/workspace -w /workspace/arm_bridge \
    "$IMAGE" python3 -m arm_bridge.main --signaling-port "$PORT" --config "/workspace/$CONFIG" \
    >/dev/null

printf 'waiting for the bridge on :%s ' "$PORT"
for _ in $(seq 1 30); do
    if ss -Hltn "sport = :$PORT" | grep -q .; then
        printf 'up\n\n'
        docker logs --tail 5 "$NAME" 2>&1
        exit 0
    fi
    if [ "$(docker inspect -f '{{.State.Running}}' "$NAME" 2>/dev/null)" != "true" ]; then
        break
    fi
    printf '.'; sleep 1
done
printf 'not up\n\n' >&2
docker logs --tail 20 "$NAME" >&2 || true
echo >&2
echo "The bridge did not start; the lines above say why. With arm_backend: aubo" >&2
echo "it exits until it can reach the controller, and Docker keeps retrying." >&2
exit 1
