#!/usr/bin/env python3
import json
import logging
import zmq
from tornado import web, websocket, ioloop
import sys
import os

class WSHandler(websocket.WebSocketHandler):
    def check_origin(self, origin):
        return True

    def open(self):
        logging.info("WebSocket connection opened")
        self.last_values = {'x': 0.0, 'y': 0.0}

    def on_message(self, message):
        try:
            data = json.loads(message)
            throttle = data.get("y", 0.0)
            steering = data.get("x", 0.0)

            # Clamp values to [-1, 1] range
            logging.info(f"Throttle: {throttle:.3f}, Steering: {steering:.3f}")

            # Send steering (X axis)
            if abs(steering - self.last_values['x']) >= 0.001:
                payload = f"<BR{steering:.3f}\n".encode()
                self.application.zmq_socket.send_multipart([b"TWSB", payload])
                self.last_values['x'] = steering

            # Send throttle (Y axis)
            if abs(throttle - self.last_values['y']) >= 0.001:
                payload = f"<BM{throttle:.3f}\n".encode()
                self.application.zmq_socket.send_multipart([b"TWSB", payload])
                self.last_values['y'] = throttle

        except (TypeError, ValueError) as e:
            logging.error(f"Invalid data: {message}", exc_info=True)
        except Exception as e:
            logging.error(f"Error: {str(e)}", exc_info=True)

    def on_close(self):
        logging.info("WebSocket connection closed")
        self.send_command(0.0, 0.0)
        
    def send_command(self, steering, throttle):
        try:
            self.application.zmq_socket.send_multipart([b"TWSB", f"<BR{steering:.3f}\n".encode()])
            self.application.zmq_socket.send_multipart([b"TWSB", f"<BM{throttle:.3f}\n".encode()])
        except zmq.ZMQError as e:
            logging.error(f"ZMQ error: {str(e)}")

class MainHandler(web.RequestHandler):
    def get(self):
        self.set_header("Content-Type", "text/html")
        self.write(self.application.index_html)

def make_app(index_html):
    return web.Application([
        (r"/", MainHandler),
        (r"/ws", WSHandler),
    ], compress_response=True, autoreload=False)

def load_index_html():
    try:
        with open("index.html", "r") as f:
            return f.read()
    except Exception as e:
        return f"<h1>Controller Error</h1><p>{str(e)}</p>"

def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
        handlers=[logging.StreamHandler(sys.stdout)]
    )

    context = zmq.Context.instance()
    zmq_socket = context.socket(zmq.PUB)
    zmq_socket.setsockopt(zmq.LINGER, 100)
    zmq_socket.bind("ipc:///tmp/joycmds")
    logging.info("ZMQ PUB connected to ipc:///tmp/joycmds")

    app = make_app(load_index_html())
    app.zmq_socket = zmq_socket
    app.index_html = load_index_html()
    
    try:
        app.listen(8080, address="0.0.0.0")
        logging.info("Server listening on :8080")
        ioloop.IOLoop.current().start()
    except KeyboardInterrupt:
        logging.info("Shutting down")
    finally:
        zmq_socket.close()
        context.term()

if __name__ == "__main__":
    main()
