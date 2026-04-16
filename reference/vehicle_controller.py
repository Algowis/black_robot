import os
import struct
import time
import socket
import threading
import sys
import subprocess

# --- AUTO-INSTALLER BLOCK ---
def ensure_can_library():
    try:
        import can
    except ImportError:
        print("Module 'python-can' not found. Installing...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "python-can"])
ensure_can_library()
import can

# --- CAN CONSTANTS ---
FRONT_DRIVE_ID = 0x302
MOTOR_MODE_SPEED = 0x01  # 01 = Speed Mode, Bit 2 = 0 (Braking Enabled)
TORQUE_LIMIT = 20        # Maximum Torque in nM (Adjust this if it's too weak/strong!)

# --- UDP CONSTANTS ---
LISTEN_PORT = 8888               # Steam Deck sends to 8888
MSG_DIRECT_CONTROL = 3           # Type 3 is the Steam Deck gamepad command
DIRECT_CONTROL_FMT = "<BHHH"     # 1 byte flag, 3 uint16 for PWM values
BASE_SIZE = 5

def init_can():
    """Initializes the CAN0 interface."""
    os.system("sudo ip link set can0 down 2>/dev/null")
    os.system("sudo ip link set can0 type can bitrate 500000")
    os.system("sudo ip link set can0 up")
    time.sleep(1)

def parse_can_telemetry(msg):
    """Decodes messages from the motor controllers on the CAN bus."""
    base_id = msg.arbitration_id & 0xFF0
    node_id = msg.arbitration_id & 0x00F
    node_name = "FRONT" if node_id == 2 else "REAR " if node_id == 4 else f"NODE_{node_id}"

    try:
        if base_id == 0x2D0 or base_id == 0x200:
            temp_L, temp_R, mosfet_L, mosfet_R, cpu_temp = struct.unpack('<hhbbb', msg.data[:7])
            print(f"[CAN: {node_name} TEMP]  Motor: {temp_L/10.0}°C | CPU: {cpu_temp}°C")

        elif base_id == 0x310:
            speed_L, speed_R, tq_L, tq_R = struct.unpack('<hhhh', msg.data[:8])
            print(f"[CAN: {node_name} DRIVE] Speed L: {speed_L} RPM | Torque L: {tq_L} Nm")

        elif base_id == 0x320:
            tot_A, tot_V, r_A, l_A, fault, mode = struct.unpack('<bHbbbB', msg.data[:7])
            print(f"[CAN: {node_name} POWER] Volts: {tot_V/10.0}V | Total Current: {tot_A}A")

    except struct.error:
        pass

def send_drive_command(bus, throttle_pwm, front_pwm):
    """Translates PWM to CAN range and transmits it to the motor drivers."""
    # Convert PWM (1000-2000) to CAN Command (-1000 to +1000)
    # Left Motor = Front Steering | Right Motor = Throttle
    left_cmd = int(max(-1000, min(1000, (front_pwm - 1500) * 2)))
    right_cmd = int(max(-1000, min(1000, (throttle_pwm - 1500) * 2)))

    # Pack the payload using Little-Endian format ('<')
    # h: int16 (ThrottleLeft)
    # h: int16 (ThrottleRight)
    # B: uint8 (MotorMode)
    # H: uint16 (Limit)
    # B: uint8 (Reserved = 0)
    try:
        payload = struct.pack(">hhBHB", left_cmd, right_cmd, MOTOR_MODE_SPEED, TORQUE_LIMIT, 0)
        
        msg = can.Message(
            arbitration_id=FRONT_DRIVE_ID,
            data=payload,
            is_extended_id=False
        )
        
        bus.send(msg)
    except struct.error as e:
        print(f"[PACK ERROR] Failed to pack CAN payload: {e}")
    except can.CanError as e:
        print(f"[CAN TX ERROR] Failed to send drive command: {e}")

def udp_listener_thread(bus):
    """Listens for Steam Deck commands and triggers the CAN send function."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", LISTEN_PORT))
    
    print(f"[UDP] Listening for Steam Deck commands on port {LISTEN_PORT}...")
    
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            
            if len(data) >= BASE_SIZE + 2:
                stx1, stx2, payload_len, sender_id, msg_type = struct.unpack("<BBBBB", data[:BASE_SIZE])
                
                # We only care about Direct Control messages from the Steam Deck
                if msg_type == MSG_DIRECT_CONTROL:
                    expected_len = BASE_SIZE + payload_len # Base(5) + Payload(7) + CRC(2)
                    
                    if len(data) == expected_len:
                        payload = data[BASE_SIZE:expected_len - 2]
                        
                        try:
                            # Unpack the 7-byte payload
                            control_flag, throttle, front, back = struct.unpack(DIRECT_CONTROL_FMT, payload)
                            gear_low = bool(control_flag & 0x01)
                            
                            print(f"[STEAM DECK] Throttle: {throttle} | Front: {front} | Gear Low: {gear_low}")
                            
                            # ---> BRIDGE THE TWO NETWORKS HERE <---
                            send_drive_command(bus, throttle, front)
                            
                        except struct.error as e:
                            print(f"    [UDP ERROR] Failed to unpack payload: {e}")
            
        except Exception as e:
            print(f"[UDP CRASH] {e}")
            
if __name__ == "__main__":
    init_can()
    
    try:
        # 1. Initialize the CAN bus FIRST so we can pass it to the UDP thread
        print("[CAN] Initializing physical bus...")
        bus = can.interface.Bus(channel='can0', bustype='socketcan')
        
        # 2. Start the UDP listener in the background, giving it the 'bus' object
        udp_thread = threading.Thread(target=udp_listener_thread, args=(bus,), daemon=True)
        udp_thread.start()
        
        print("[READY] Listening to motor telemetry. Press Ctrl+C to stop...")
        
        # 3. Main thread loops forever, printing telemetry it hears on the wire
        for msg in bus:
            parse_can_telemetry(msg)
            
    except KeyboardInterrupt:
        print("\nShutting down listener...")
    finally:
        bus.shutdown()