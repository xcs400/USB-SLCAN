#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simulateur CANopen SLCAN pour les nœuds :
G=10, D=11, X=12, Y=13, Z=14, W/broche=15.

ATTENTION :
- À utiliser uniquement sur un banc isolé, sans variateurs réels ayant les mêmes Node-ID.
- Ce script simule des réponses SDO et des Heartbeats ; il ne commande pas réellement les moteurs.
- Les types et sémantiques des objets constructeur restent à confirmer avec l'EDS/la documentation.
- Le homing G/D et les limites ne sont PAS exécutés physiquement : leurs paramètres sont seulement mémorisés.
"""

import random
import threading
import time
from dataclasses import dataclass, field

import can

PORT = "/dev/ttyACM0"
SERIAL_BAUDRATE = 115200
CAN_BITRATE = 250000
HEARTBEAT_PERIOD = 1.0
SIMULATION_PERIOD = 0.2

NODES = {
    10: "G",
    11: "D",
    12: "X",
    13: "Y",
    14: "Z",
    15: "W",
}

# Mode de variation : "incremental" ou "random"
PROFILE_MODE = "incremental"

# Valeurs de simulation ; adaptez les bornes à vos unités réelles.
POSITION_MIN = -100000
POSITION_MAX = 100000
VELOCITY_MAX = 1500
POWER_MAX = 1200


@dataclass
class NodeState:
    name: str
    statusword: int = 0x0040       # switch-on disabled simulé
    controlword: int = 0
    target_velocity: int = 0       # 0x60FF
    actual_position: int = 0       # 0x6064
    actual_velocity: int = 0       # 0x301E:08
    actual_power: int = 0          # 0x301C:0D
    mode: int = 3                  # 0x6060
    jog_active: int = 0            # 0x301B:09
    jog_slow: int = 100            # 0x3029:04
    jog_fast: int = 500            # 0x3029:05
    profile_velocity: int = 4000   # 0x607F
    accel: int = 2000              # 0x6083
    decel: int = 7000              # 0x6084
    homing_method: int = 35         # 0x6098
    software_zero: int = 0
    limit_min: int = -100000        # 0x607D:01
    limit_max: int = 100000         # 0x607D:02
    limit_enable: int = 0           # 0x3006:03, bit 0=min, bit 1=max
    values: dict = field(default_factory=dict)
    last_update: float = field(default_factory=time.monotonic)
    power_samples: list = field(default_factory=list)


states = {nid: NodeState(name) for nid, name in NODES.items()}
stop_event = threading.Event()

# Index/subindex -> taille en octets. Les tailles vendor-specific sont à vérifier.
OBJECT_SIZES = {
    (0x6041, 0x00): 2,
    (0x6040, 0x00): 2,
    (0x6064, 0x00): 4,
    (0x301E, 0x08): 2,
    (0x301C, 0x0D): 4,
    (0x60FF, 0x00): 4,
    (0x6060, 0x00): 1,
    (0x607F, 0x00): 4,
    (0x6083, 0x00): 4,
    (0x6084, 0x00): 4,
    (0x6098, 0x00): 1,
    (0x301B, 0x09): 1,
    (0x301B, 0x16): 1,  # taille supposée, à vérifier
    (0x3029, 0x03): 1,
    (0x3029, 0x04): 4,  # taille supposée, à vérifier
    (0x3029, 0x05): 4,  # taille supposée, à vérifier
    (0x607D, 0x01): 4,
    (0x607D, 0x02): 4,
    (0x3006, 0x03): 2,  # taille supposée, à vérifier
    (0x3006, 0x0C): 2,  # taille supposée, à vérifier
    (0x3004, 0x01): 1,
    (0x3023, 0x07): 1,  # taille supposée, à vérifier
}

# Abort codes CANopen
ABORT_NO_OBJECT = 0x06020000
ABORT_NO_SUBINDEX = 0x06090011
ABORT_TYPE_MISMATCH = 0x06070010


def signed_value(node_id, index, subindex):
    """Retourne la valeur entière signée lorsqu'un objet est signé."""
    s = states[node_id]
    key = (index, subindex)
    if key == (0x6064, 0x00):
        return s.actual_position
    if key == (0x301E, 0x08):
        return s.actual_velocity
    if key == (0x6060, 0x00):
        return s.mode
    return None


def get_value(node_id, index, subindex):
    s = states[node_id]
    key = (index, subindex)

    known = {
        (0x6041, 0x00): s.statusword,
        (0x6040, 0x00): s.controlword,
        (0x6064, 0x00): s.actual_position,
        (0x301E, 0x08): s.actual_velocity,
        (0x301C, 0x0D): s.actual_power,
        (0x60FF, 0x00): s.target_velocity,
        (0x6060, 0x00): s.mode,
        (0x607F, 0x00): s.profile_velocity,
        (0x6083, 0x00): s.accel,
        (0x6084, 0x00): s.decel,
        (0x6098, 0x00): s.homing_method,
        (0x301B, 0x09): s.jog_active,
        (0x3029, 0x04): s.jog_slow,
        (0x3029, 0x05): s.jog_fast,
        (0x607D, 0x01): s.limit_min,
        (0x607D, 0x02): s.limit_max,
        (0x3006, 0x03): s.limit_enable,
    }
    if key in known:
        return known[key]
    if key in s.values:
        return s.values[key]
    return None


def to_unsigned_bytes(value, size):
    return (int(value) & ((1 << (8 * size)) - 1)).to_bytes(size, "little")


def send_sdo_abort(bus, node_id, index, subindex, code):
    data = bytes([0x80, index & 0xFF, (index >> 8) & 0xFF, subindex,
                  code & 0xFF, (code >> 8) & 0xFF,
                  (code >> 16) & 0xFF, (code >> 24) & 0xFF])
    send_frame(bus, 0x580 + node_id, data, f"SDO ABORT {NODES[node_id]} 0x{index:04X}:{subindex:02X}")


def send_frame(bus, cob_id, data, label):
    try:
        bus.send(can.Message(arbitration_id=cob_id, data=data, is_extended_id=False))
        print(f"TX {label:<24} ID=0x{cob_id:03X} DATA={' '.join(f'{b:02X}' for b in data)}")
    except can.CanError as exc:
        print(f"ERREUR émission ID=0x{cob_id:03X}: {exc}")


def sdo_response(bus, node_id, index, subindex, value, size):
    # Réponse upload expedited : 0x4F (1), 0x4B (2), 0x43 (4).
    command = {1: 0x4F, 2: 0x4B, 3: 0x47, 4: 0x43}[size]
    raw = to_unsigned_bytes(value, size)
    data = bytes([command, index & 0xFF, (index >> 8) & 0xFF, subindex]) + raw.ljust(4, b"\x00")
    send_frame(bus, 0x580 + node_id, data, f"SDO REP {NODES[node_id]} 0x{index:04X}:{subindex:02X}")


def apply_write(node_id, index, subindex, value):
    s = states[node_id]
    key = (index, subindex)
    if key == (0x6040, 0x00):
        s.controlword = value
        # État de profil CiA-402 simplifié, destiné aux tests d'interface seulement.
        if value & 0x0080:       # fault reset
            s.statusword = 0x0040
        elif value == 0x0006:
            s.statusword = 0x0021
        elif value == 0x0007:
            s.statusword = 0x0023
        elif value == 0x000F:
            s.statusword = 0x0027
        elif value == 0x0100 or value == 0:
            s.statusword = 0x0040
            s.target_velocity = 0
            s.actual_velocity = 0
        return True
    if key == (0x60FF, 0x00):
        s.target_velocity = value
        return True
    if key == (0x6060, 0x00):
        s.mode = value if value < 128 else value - 256
        return True
    if key == (0x607F, 0x00):
        s.profile_velocity = value
        return True
    if key == (0x6083, 0x00):
        s.accel = value
        return True
    if key == (0x6084, 0x00):
        s.decel = value
        return True
    if key == (0x6098, 0x00):
        s.homing_method = value
        return True
    if key == (0x301B, 0x09):
        s.jog_active = value
        if value == 0:
            s.target_velocity = 0
        return True
    if key == (0x3029, 0x04):
        s.jog_slow = value
        return True
    if key == (0x3029, 0x05):
        s.jog_fast = value
        return True
    if key == (0x607D, 0x01):
        s.limit_min = value
        return True
    if key == (0x607D, 0x02):
        s.limit_max = value
        return True
    if key == (0x3006, 0x03):
        s.limit_enable = value
        return True
    # Autres objets explicitement listés : valeur conservée, sémantique non interprétée.
    if key in OBJECT_SIZES:
        s.values[key] = value
        return True
    return False


def handle_sdo(bus, msg):
    node_id = msg.arbitration_id - 0x600
    if node_id not in states or len(msg.data) != 8:
        return
    d = bytes(msg.data)
    cmd = d[0]
    index = d[1] | (d[2] << 8)
    sub = d[3]
    key = (index, sub)

    if cmd == 0x40:  # upload/read request
        size = OBJECT_SIZES.get(key)
        value = get_value(node_id, index, sub)
        if size is None:
            send_sdo_abort(bus, node_id, index, sub,
                           ABORT_NO_OBJECT if not any(k[0] == index for k in OBJECT_SIZES) else ABORT_NO_SUBINDEX)
            return
        if value is None:
            value = 0
        # Les valeurs signées sont encodées en complément à deux.
        send_sdo_upload_value = int(value)
        sdo_response(bus, node_id, index, sub, send_sdo_upload_value, size)
        return

    # SDO download expedited, tailles déterminées par le command specifier.
    size_by_cmd = {0x2F: 1, 0x2B: 2, 0x27: 3, 0x23: 4}
    if cmd in size_by_cmd:
        size = size_by_cmd[cmd]
        expected = OBJECT_SIZES.get(key)
        if expected is None:
            send_sdo_abort(bus, node_id, index, sub,
                           ABORT_NO_OBJECT if not any(k[0] == index for k in OBJECT_SIZES) else ABORT_NO_SUBINDEX)
            return
        if size != expected:
            send_sdo_abort(bus, node_id, index, sub, ABORT_TYPE_MISMATCH)
            return
        value = int.from_bytes(d[4:4 + size], "little", signed=False)
        if key in ((0x6060, 0x00), (0x301E, 0x08), (0x6064, 0x00),
                   (0x607D, 0x01), (0x607D, 0x02)):
            bits = size * 8
            if value & (1 << (bits - 1)):
                value -= 1 << bits
        if not apply_write(node_id, index, sub, value):
            send_sdo_abort(bus, node_id, index, sub, ABORT_NO_OBJECT)
            return
        # Réponse download : 0x60 + index + subindex + 4 octets nuls.
        ack = bytes([0x60, d[1], d[2], sub, 0, 0, 0, 0])
        send_frame(bus, 0x580 + node_id, ack, f"SDO ACK {NODES[node_id]} 0x{index:04X}:{sub:02X}")
        print(f"    valeur mémorisée: {NODES[node_id]} 0x{index:04X}:{sub:02X} = {value}")
        return

    print(f"SDO non géré: node={node_id} cmd=0x{cmd:02X} index=0x{index:04X} sub={sub}")


def update_simulated_measurements():
    now = time.monotonic()
    for node_id, s in states.items():
        dt = min(now - s.last_update, 0.5)
        s.last_update = now

        if PROFILE_MODE == "random":
            s.actual_velocity = random.randint(-VELOCITY_MAX, VELOCITY_MAX) if s.target_velocity else 0
            s.actual_power = random.randint(0, POWER_MAX)
            if node_id in (12, 13, 14) and s.target_velocity:
                s.actual_position += random.randint(-20, 20)
        else:
            # La vitesse réelle converge vers la consigne, avec un petit bruit.
            if s.target_velocity:
                target = max(-VELOCITY_MAX, min(VELOCITY_MAX, s.target_velocity))
                s.actual_velocity += int((target - s.actual_velocity) * 0.35) or (1 if target > s.actual_velocity else -1 if target < s.actual_velocity else 0)
                s.actual_velocity += random.randint(-2, 2)
                s.actual_power = min(POWER_MAX, abs(s.actual_velocity) // 2 + random.randint(0, 20))
                if node_id in (12, 13, 14):
                    s.actual_position += int(s.actual_velocity * dt)
            else:
                s.actual_velocity = 0
                s.actual_power = random.randint(0, 5)

        # Limites de course simulées pour X/Y/Z : arrêt logiciel de la simulation.
        if node_id in (12, 13, 14):
            if (s.limit_enable & 0x01) and s.actual_position <= s.limit_min:
                s.actual_position = s.limit_min
                s.target_velocity = 0
            if (s.limit_enable & 0x02) and s.actual_position >= s.limit_max:
                s.actual_position = s.limit_max
                s.target_velocity = 0
            s.actual_position = max(POSITION_MIN, min(POSITION_MAX, s.actual_position))

        s.power_samples.append(s.actual_power)
        s.power_samples = s.power_samples[-3:]


def heartbeat_loop(bus):
    while not stop_event.is_set():
        for node_id in states:
            if stop_event.is_set():
                break
            send_frame(bus, 0x700 + node_id, bytes([0x05]),
                       f"HEARTBEAT {NODES[node_id]} OPERATIONAL")
        stop_event.wait(HEARTBEAT_PERIOD)


def simulation_loop():
    while not stop_event.wait(SIMULATION_PERIOD):
        update_simulated_measurements()


def main():
    print("Simulateur CANopen SLCAN — banc isolé uniquement")
    print(f"Port={PORT}, UART={SERIAL_BAUDRATE}, CAN={CAN_BITRATE}, profil={PROFILE_MODE}")
    print("Nœuds : " + ", ".join(f"{name}={nid}" for nid, name in NODES.items()))
    print("Ctrl+C pour arrêter.")
    bus = can.Bus(interface="slcan", channel=PORT, bitrate=CAN_BITRATE,
                  ttyBaudrate=SERIAL_BAUDRATE)
    threads = [
    #    threading.Thread(target=heartbeat_loop, args=(bus,), daemon=True),
 #       threading.Thread(target=simulation_loop, daemon=True),
    ]
    for t in threads:
        t.start()
    try:
        while True:
            msg = bus.recv(timeout=0.5)
            if msg is None or msg.is_error_frame or msg.is_remote_frame:
                continue
            if 0x600 <= msg.arbitration_id <= 0x6FF:
                handle_sdo(bus, msg)
            else:
                print(f"RX ID=0x{msg.arbitration_id:03X} DATA={' '.join(f'{b:02X}' for b in msg.data)}")
    except KeyboardInterrupt:
        print("\nArrêt demandé.")
    finally:
        stop_event.set()
        bus.shutdown()


if __name__ == "__main__":
    main()
