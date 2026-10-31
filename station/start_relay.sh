#!/usr/bin/env bash
# Start the Spectacles relay in this terminal. Keep the terminal open: the
# relay takes keyboard commands (e + Enter enables the arm, q quits).
#
#   station/start_relay.sh                    # arm bridge on this machine, Lens port 8766
#   station/start_relay.sh --bridge-port 8454 # extra arguments go to specs_relay.py
set -euo pipefail
cd "$(dirname "$0")/.."

IMAGE="${IMAGE:-specsreach-station}"
NAME="${NAME:-specsreach-relay}"

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "image $IMAGE not found; build it first: docker build -t $IMAGE station/" >&2
    exit 1
fi
if [ -n "$(docker ps -q --filter "name=^${NAME}$")" ]; then
    echo "a relay is already running ($NAME). Type q in its terminal, or: docker rm -f $NAME" >&2
    exit 1
fi

tty=(-i)
[ -t 0 ] && tty=(-it)
exec docker run --rm "${tty[@]}" --name "$NAME" --network host \
    --user "$(id -u):$(id -g)" -e HOME=/tmp \
    -v "$PWD":/workspace -w /workspace \
    "$IMAGE" python3 specs_relay/specs_relay.py --bridge-host 127.0.0.1 --port 8766 "$@"
