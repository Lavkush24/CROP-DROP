
import socket
import json
import time

class CoppeliaClient:
    def __init__(self, host='127.0.0.1', port=50002):
        self.host = host
        self.port = port
        self.sock = None
        self.buffer = ""

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.host, self.port))
        self.sock.settimeout(0.1)
    
    def send_motor_command(self, left_speed, right_speed,state=0,reward=0,action=0):
        cmd = {"command": "set_speed", "L": left_speed, "R": right_speed,"State":state,"Reward":reward,"Action":action}
        msg = json.dumps(cmd) + "\n"    
        self.sock.sendall(msg.encode())
    
    def start_simulation(self):
        cmd = {"command": "start_simulation"}
        msg = json.dumps(cmd) + "\n"
        self.sock.sendall(msg.encode())

    def stop_simulation(self):
        cmd = {"command": "stop_simulation"}
        msg = json.dumps(cmd) + "\n"
        self.sock.sendall(msg.encode())

    def receive_sensor_data(self):
        try:
            data = self.sock.recv(1024).decode()
            if not data:
                return None
            self.buffer += data
            if "\n" in self.buffer:
                line, self.buffer = self.buffer.split("\n", 1)
                sensor_msg = json.loads(line)
                if sensor_msg.get("type") == "sensor_update":
                    return sensor_msg["sensors"]
        except socket.timeout:
            pass
        except Exception as e:
            print(f"[CoppeliaClient] Error receiving sensor data: {e}")
        return None

    def close(self):
        if self.sock:
            self.sock.close()
            self.sock = None
