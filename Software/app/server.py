import zmq
import json
import time
import uuid
import syslog
import asyncio
import websockets

# Configuration
SESSION_TIMEOUT = 30  
JOYSTICK_DEADZONE = 0.05  
HOST = '0.0.0.0'
PORT = 5001  # WebSocket Port

# Syslog Setup
syslog.openlog("JoystickServer", syslog.LOG_PID)

# ZMQ Setup
zmq_ctx = zmq.Context()
zmq_sock = zmq_ctx.socket(zmq.PUB)
zmq_sock.connect("ipc:///tmp/botcmds")

# Joystick State
joystick_state = {'x': 0.0, 'y': 0.0}

def send_update(x, y):
    """Send joystick data via ZMQ"""
    packet_x = f"<BR{x:.2f}\n"
    zmq_sock.send_multipart([b'TWSB', packet_x.encode()])
    
    packet_y = f"<BM{y:.2f}\n"
    zmq_sock.send_multipart([b'TWSB', packet_y.encode()])

async def handle_websocket(websocket, path):
    """Handle WebSocket connections"""
    async for message in websocket:
        try:
            data = json.loads(message)
            new_x = round(float(data.get('x', 0)), 2)
            new_y = round(float(data.get('y', 0)), 2)

            if (abs(new_x - joystick_state['x']) >= JOYSTICK_DEADZONE or 
                abs(new_y - joystick_state['y']) >= JOYSTICK_DEADZONE or 
                (new_x == 0 and new_y == 0)):
                
                joystick_state['x'], joystick_state['y'] = new_x, new_y
                send_update(new_x, new_y)

        except (ValueError, TypeError, json.JSONDecodeError):
            syslog.syslog(syslog.LOG_ERR, "Invalid WebSocket input received.")

async def start_server():
    """Start WebSocket server"""
    async with websockets.serve(handle_websocket, HOST, PORT):
        syslog.syslog(syslog.LOG_INFO, f"WebSocket server started on ws://{HOST}:{PORT}")
        await asyncio.Future()  # Run forever

if __name__ == '__main__':
    try:
        asyncio.run(start_server())
    except KeyboardInterrupt:
        syslog.syslog(syslog.LOG_INFO, "Shutting down WebSocket server.")
    finally:
        zmq_sock.close()
        zmq_ctx.term()
