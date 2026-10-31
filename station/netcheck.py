#!/usr/bin/env python3
"""SpecsReach station preflight: what the Lens needs and what will block it.

    python3 station/netcheck.py

Read-only: it lists this PC's addresses, pings the AUBO, and checks which
ports are listening. It never connects to the arm bridge or moves anything.
Standard library only.
"""
import ipaddress
import os
import re
import shutil
import socket
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, "arm_bridge.yaml")
RELAY_PORT = 8766
BRIDGE_PORT = 8444
AUBO_RPC_PORT = 30004
IMAGE = "specsreach-station"
SKIP_IFACES = ("lo", "docker", "br-", "veth", "virbr", "cni", "flannel", "tailscale", "zt")

problems = 0


def say(status, text):
    global problems
    if status == "FIX":
        problems += 1
    print(f"  [{status:4}] {text}")


def run(cmd):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        return ""


def interfaces():
    """[(name, IPv4Interface)] for the global addresses on real interfaces."""
    out = []
    for line in run(["ip", "-4", "-o", "addr", "show", "scope", "global"]).splitlines():
        parts = line.split()
        if len(parts) >= 4 and not parts[1].startswith(SKIP_IFACES):
            out.append((parts[1], ipaddress.ip_interface(parts[3])))
    return out


def default_iface():
    m = re.search(r"\bdev (\S+)", run(["ip", "-4", "route", "show", "default"]))
    return m.group(1) if m else None


def aubo_host():
    try:
        text = open(CONFIG, encoding="utf-8").read()
    except OSError:
        return None
    block = re.search(r"^aubo:\s*\n((?:[ \t]+.*\n?)+)", text, re.M)
    m = re.search(r"^\s+host:\s*\"?([^\"\s#]+)", block.group(1), re.M) if block else None
    return m.group(1) if m else None


def listening(port):
    return bool(run(["ss", "-Hltn", f"sport = :{port}"]).strip())


def main():
    print("SpecsReach station check\n")

    host = aubo_host()
    arm_net = None
    ifaces = interfaces()
    if host:
        try:
            arm_ip = ipaddress.ip_address(host)
            arm_net = next((i.network for _, i in ifaces if arm_ip in i.network), None)
        except ValueError:
            pass

    print("Lens Server Url (ArmRelayClient):")
    dflt = default_iface()
    lens_ifaces = [(n, i) for n, i in ifaces if i.network != arm_net]
    for name, iface in sorted(lens_ifaces, key=lambda x: x[0] != dflt):
        hint = "  <- use this one" if name == dflt else ""
        say("OK", f"ws://{iface.ip}:{RELAY_PORT}   ({name}){hint}")
    if not lens_ifaces:
        say("FIX", "no network the glasses could use: join this PC to the same Wi-Fi as the Spectacles")
    for name, iface in ifaces:
        if iface.network == arm_net:
            print(f"         {iface.ip} ({name}) is the arm's cable network: the glasses can't reach it")

    print("\nAUBO controller:")
    if not host:
        say("FIX", f"no aubo.host in {CONFIG}")
    else:
        if arm_net is None:
            say("WARN", f"{host} is on no subnet of this PC: set a static address on the arm's cable")
        ping = subprocess.run(["ping", "-c", "1", "-W", "1", host], capture_output=True)
        if ping.returncode == 0:
            try:
                socket.create_connection((host, AUBO_RPC_PORT), timeout=1.0).close()
                say("OK", f"{host} answers, RPC port {AUBO_RPC_PORT} open (an ARCS controller)")
            except OSError:
                say("FIX", f"{host} answers ping but port {AUBO_RPC_PORT} is closed: "
                           "not an ARCS controller, or its software is still starting")
        else:
            say("FIX", f"{host} does not answer: check the cable, the controller power and its IP")

    print("\nPrograms:")
    if listening(BRIDGE_PORT):
        say("OK", f"an arm bridge is listening on :{BRIDGE_PORT}")
    else:
        say("FIX", f"no arm bridge on :{BRIDGE_PORT}: station/start_bridge.sh")
    if listening(RELAY_PORT):
        say("OK", f"the relay is listening on :{RELAY_PORT}")
    else:
        say("INFO", f"the relay is not running yet: station/start_relay.sh")
    if shutil.which("docker"):
        if subprocess.run(["docker", "image", "inspect", IMAGE], capture_output=True).returncode == 0:
            say("OK", f"Docker image {IMAGE} is built")
        else:
            say("FIX", f"Docker image {IMAGE} missing: docker build -t {IMAGE} station/")
    else:
        say("FIX", "Docker is not installed")

    print("\nFirewall:")
    if shutil.which("ufw") and run(["systemctl", "is-active", "ufw"]).strip() == "active":
        nets = sorted({str(i.network) for n, i in lens_ifaces if n == dflt}) or ["<wifi-subnet>"]
        say("INFO", "ufw is on. Allow the relay once (this script can't read the rules without sudo):")
        for net in nets:
            print(f"         sudo ufw allow from {net} to any port {RELAY_PORT} proto tcp")
    else:
        say("OK", "no active ufw firewall")

    print("\n" + ("All set." if problems == 0 else f"{problems} thing(s) to fix above."))
    return 0 if problems == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
