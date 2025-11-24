# Network HMI (network\_hmi)

## Overview

This package provides a ROS 2 node (`bridge_node`) that acts as a bridge between a single network client (e.g., a GUI, a remote controller) and the ROS 2 ecosystem, using a combination of TCP for control/state and UDP for high-frequency data.

Its primary function is to:

1.  **Manage a single client** via a persistent TCP connection.
2.  **Control internal state** (e.g., `start`, `mode`) using JSON commands received over TCP.
3.  **Receive `cmd_vel`** data (linear/angular velocity) from the client via UDP.
4.  **Combine** the received `cmd_vel` data with the internal state and publish it as an `interfaces/msg/JoystickOrder` message on the `/joystick_order` topic.
5.  **Receive `/odom`** messages from ROS 2 and forward them to the client as `real_vel` JSON messages via UDP.

## Key Behaviors & State Management

  * **Single Client:** The bridge is designed for a single-client model. Only one client can be registered at a time. If a second client tries to register, it will be refused until the first client disconnects.
  * **TCP Control Connection:** The TCP connection is kept open for the duration of the client's session. This connection is used for registration, state changes, and heartbeating.
  * **Heartbeat:** The server sends a `{"type":"heartbeat"}` message to the client every **13 seconds** to verify the connection is alive.
  * **Activity Timeout:** The server expects to *receive* data from the client (e.g., a `heartbeat_ack`, a `ping`, or any other command) at regular intervals. If the server doesn't receive *any* data from the client for **15 seconds**, it will consider the client unresponsive, close the TCP session, and deregister the client.
  * **Stateful Operation:** The node maintains an internal state (primarily `start` and `mode`).
      * This state is **only** changed by TCP commands (`start`, `set_mode`, `emergency_stop`).
      * UDP `cmd_vel` packets **do not** change this state.
      * The current `start` and `mode` values are injected into every outgoing `/joystick_order` message, along with the throttle/steer values from the last received UDP packet.
      * Setting `mode = 2` will also implicitly set `start = false`.

-----

## 📦 Dependencies

This node is a C++ ROS 2 node.

  * **ROS 2 Humble** (or newer)
  * **nlohmann-json3-dev:** The header-only nlohmann/json library.
    ```bash
    sudo apt install -y nlohmann-json3-dev
    ```
  * **`interfaces` package:** This node depends on a custom message definition: `interfaces/msg/JoystickOrder.hpp`.

-----

## 🛠️ Building

From the root of your ROS 2 workspace (e.g., `~/ros2_ws`):

```bash
# 1. Source your ROS 2 environment
source /opt/ros/humble/setup.bash

# 2. Build the package (and its dependencies like 'interfaces', dont forget to build interfaces first if needed)
colcon build --packages-select network_hmi
```

-----

## 🚀 Running

Source your workspace's setup file, then run the node:

```bash
# Source the workspace
source install/setup.bash

# Run the node
ros2 run network_hmi bridge_node
```

### Parameters

The node declares two parameters with default values:

  * `tcp_control_port`: **5001** (Port for the TCP control and command server)
  * `udp_data_port`: **5000** (Port for *receiving* `cmd_vel` UDP packets from the client)

-----

## 📡 Network Protocol

Communication is done via newline-delimited JSON messages.

### TCP Control (Port 5001)

#### Client-to-Server (C2S)

  * **Register:** Initiates the session. Must be the first message.

    ```json
    {"type": "register", "client_id": "my_gui", "recv_udp_port": 5003}
    ```

      * `recv_udp_port`: The port on the *client* where the server should send `real_vel` (odom) data.

  * **Heartbeat ACK:** The client's required response to the server's heartbeat.

    ```json
    {"type": "heartbeat_ack"}
    ```

  * **Ping:** A simple keep-alive or RTT check initiated by the client.

    ```json
    {"type": "ping"}
    ```

  * **Start:** Sets the internal state to `start = true`.

    ```json
    {"type": "start"}
    ```

  * **Emergency Stop:** Sets the internal state to `start = false`.

    ```json
    {"type": "emergency_stop"}
    ```

  * **Set Mode:** Sets the internal mode (integer).

    ```json
    {"type": "set_mode", "mode": 1}
    ```

  * **Close Session:** Politely informs the server that the client is disconnecting.

    ```json
    {"type": "close"}
    ```

#### Server-to-Client (S2C)

  * **Registration Response (Success):**

    ```json
    {"ok": true, "udp_data_port": 5000}
    ```

      * `udp_data_port`: The port on the *server* where the client must send `cmd_vel` data.

  * **Registration Response (Failure):**

    ```json
    {"ok": false, "error": "another client already connected"}
    ```

  * **Heartbeat:** Sent every 13 seconds to check if the client is alive.

    ```json
    {"type": "heartbeat"}
    ```

  * **Pong:** The server's response to a `ping`.

    ```json
    {"type": "pong"}
    ```

  * **Command Acknowledgment:** Sent in response to `start`, `set_mode`, etc.

    ```json
    {"ok": true, "message": "Start command acknowledged"}
    ```

### UDP Data

#### Client-to-Server (Port 5000)

  * **Command Velocity:** Provides the joystick/throttle values.
    ```json
    {"type": "cmd_vel", "linear_x": 0.5, "angular_z": -0.2}
    ```
      * This data is used to populate a `JoystickOrder` message.
      * `linear_x` becomes `throttle`. If `linear_x` is negative, `reverse` is set to `true` and `throttle` becomes positive.
      * `angular_z` becomes `steer`.

#### Server-to-Client (Port from `register`)

  * **Real Velocity:** Sent every time the node receives an `/odom` message.
    ```json
    {"type": "real_vel", "linear_x": 0.48, "angular_z": -0.19}
    ```

-----

## 🐍 Example Python Client

This minimal example connects, registers, handles heartbeats, and sends a `start` command.

```python
import socket
import json
import time
import threading

HOST = '127.0.0.1'     # Server IP
TCP_PORT = 5001        # Server TCP control port
CLIENT_UDP_PORT = 5003 # Port this client will listen on for odom data

def tcp_client(sock):
    """Manages the TCP connection."""
    buffer = b""
    while True:
        try:
            chunk = sock.recv(4096)
            if not chunk:
                print("Server closed connection")
                break
            buffer += chunk
            
            while b'\n' in buffer:
                line, buffer = buffer.split(b'\n', 1)
                if not line:
                    continue
                
                try:
                    msg = json.loads(line.decode())
                    print(f"[TCP RECV]: {msg}")
                    
                    if msg.get("type") == "heartbeat":
                        # Respond to heartbeat
                        ack = {"type": "heartbeat_ack"}
                        print(f"[TCP SEND]: {ack}")
                        sock.sendall((json.dumps(ack) + '\n').encode())
                        
                except Exception as e:
                    print(f"Invalid JSON or error: {e}")
                    
        except socket.timeout:
            continue # Just a read timeout, loop again
        except Exception as e:
            print(f"TCP Error: {e}")
            break

# --- Main script ---
try:
    s = socket.create_connection((HOST, TCP_PORT), timeout=5)
    s.settimeout(2.0) # Set a read timeout
    print("Connected to TCP server.")
    
    # 1. Register
    reg_msg = {
        "type": "register",
        "client_id": "py_client_01",
        "recv_udp_port": CLIENT_UDP_PORT
    }
    print(f"[TCP SEND]: {reg_msg}")
    s.sendall((json.dumps(reg_msg) + '\n').encode())
    
    # Start the TCP listener thread
    threading.Thread(target=tcp_client, args=(s,), daemon=True).start()
    
    # 2. Wait a moment, then send a 'start' command
    time.sleep(1)
    start_msg = {"type": "start"}
    print(f"[TCP SEND]: {start_msg}")
    s.sendall((json.dumps(start_msg) + '\n').encode())
    
    # Keep main thread alive
    while True:
        time.sleep(10)

except Exception as e:
    print(f"Failed to connect or run: {e}")
finally:
    if 's' in locals():
        s.close()
    print("Client shut down.")
```

## 🧪 Testing UDP `cmd_vel`

You can use this one-liner to send a single UDP `cmd_vel` packet to the bridge:

```bash
python3 - <<'PY'
import socket, json
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
msg = {"type":"cmd_vel", "linear_x": 0.5, "angular_z": 0.1}
# Sends to port 5000 (the default udp_data_port)
s.sendto(json.dumps(msg).encode(), ('127.0.0.1', 5000))
print("UDP cmd_vel sent.")
PY
```

You can then check that the `/joystick_order` topic is being published:
`ros2 topic echo /joystick_order`