from microdot import Microdot, Response, send_file
import zmq
import json
import time
import uuid

app = Microdot()
Response.default_content_type = 'text/html'

ctx = zmq.Context(1)
sock = ctx.socket(zmq.PUB)
sock.connect("ipc:///tmp/botcmds")

# Global joystick state
joystick_state = {'x': 0.0, 'y': 0.0}

# Session management globals
active_session_token = None
last_active_time = 0
SESSION_TIMEOUT = 30  # seconds

def update_session():
    global last_active_time
    last_active_time = time.time()

def create_new_session():
    global active_session_token, last_active_time
    active_session_token = str(uuid.uuid4())
    last_active_time = time.time()
    return active_session_token

def session_valid(request):
    global active_session_token, last_active_time
    now = time.time()
    # Invalidate session if none or timed out
    if active_session_token is None or (now - last_active_time) > SESSION_TIMEOUT:
        active_session_token = None
        return False
    session = request.cookies.get('session')
    if session == active_session_token:
        update_session()
        return True
    return False

def send_update():
    """Send the current joystick state over ZMQ with the required format."""
    x = joystick_state['x']
    y = joystick_state['y']
    # Create and send the packet for x-axis
    packet = "<BR{:.2f}\n".format(x)
    sock.send_multipart([b'TWSB', packet.encode('utf-8')])
    # Create and send the packet for y-axis
    packet = "<BM{:.2f}\n".format(y)
    sock.send_multipart([b'TWSB', packet.encode('utf-8')])

def update_joystick_state(new_x, new_y):
    """
    Update joystick state if it changed significantly (2 decimal places).
    Force sending an update when a zero state is received so that
    control loops always get a reset signal.
    """
    global joystick_state
    new_x = round(new_x, 2)
    new_y = round(new_y, 2)

    # Only send an update if there's a significant change
    if new_x != joystick_state['x'] or new_y != joystick_state['y']:
        joystick_state['x'] = new_x
        joystick_state['y'] = new_y
        send_update()

@app.route('/')
def index(request):
    global active_session_token, last_active_time
    now = time.time()
    # Create a new session if none exists or it has expired
    if active_session_token is None or (now - last_active_time) > SESSION_TIMEOUT:
        token = create_new_session()
        response = send_file('index.html')
        response.set_cookie('session', token)
        return response
    else:
        # Check if the incoming request has the valid session token
        session = request.cookies.get('session')
        if session == active_session_token:
            update_session()
            response = send_file('index.html')
            response.set_cookie('session', active_session_token)
            return response
        else:
            # Reject new connection; another session is active
            return send_file('error.html'), 403

@app.route('/update', methods=['POST'])
def update_joystick(request):
    if not session_valid(request):
        return {'status': 'error', 'message': 'Invalid session'}, 403
    try:
        data = json.loads(request.body)
        new_x = float(data.get('x', 0.0))
        new_y = float(data.get('y', 0.0))
        update_joystick_state(new_x, new_y)
        return {'status': 'ok'}
    except (ValueError, TypeError, json.JSONDecodeError):
        return {'status': 'error', 'message': 'Invalid input'}, 400

@app.route('/reset', methods=['POST'])
def reset(request):
    if not session_valid(request):
        return {'status': 'error', 'message': 'Invalid session'}, 403
    # Force a zero state update when the joystick is released
    update_joystick_state(0.0, 0.0)
    return {'status': 'reset'}

if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=5000, debug=True)
    except KeyboardInterrupt:
        update_joystick_state(0.0, 0.0)
        print("Server shutting down...")