#!/usr/bin/env python3
import json
import logging
import zmq
from tornado import web, websocket, ioloop
import sys
import os

# WebSocket handler: receives joystick data and publishes it via ZMQ.
class WSHandler(websocket.WebSocketHandler):
    def check_origin(self, origin):
        # Allow all origins (adjust if needed for security)
        return True

    def open(self):
        logging.info("WebSocket connection opened.")
        self.last_x = None
        self.last_y = None

    def on_message(self, message):
        try:
            # Parse incoming JSON with joystick coordinates.
            data = json.loads(message)
            # Multiply by -2 as required.
            x = float(data.get("x", 0)) * 2
            y = float(data.get("y", 0)) * 2

            self.last_x = x
            self.last_y = y

            # Combine commands into one message to reduce overhead.
            # Use .3f formatting to capture 0.001 changes.
            payload = f"<BR{x:.3f}\n".encode()
            self.application.zmq_socket.send_multipart([b"TWSB", payload])
            payload2 = f"<BM{y:.3f}\n".encode()
            self.application.zmq_socket.send_multipart([b"TWSB", payload2])
        except Exception as e:
            logging.error("Error processing message: %s", e)

    def on_close(self):
        logging.info("WebSocket connection closed.")

# HTTP handler for serving the UI.
class MainHandler(web.RequestHandler):
    def get(self):
        self.set_header("Content-Type", "text/html")
        self.write(self.application.index_html)

def make_app(index_html):
    return web.Application([
        (r"/", MainHandler),
        (r"/ws", WSHandler),
    ], debug=False)

def load_index_html():
    if os.path.exists("index.html"):
        with open("index.html", "r") as f:
            return f.read()

def main():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")
    index_html = load_index_html()

    # Set up the ZMQ publisher.
    context = zmq.Context()
    zmq_socket = context.socket(zmq.PUB)
    # Set linger to 0 to avoid delays on socket close.
    zmq_socket.setsockopt(zmq.LINGER, 0)
    # Connect to the local IPC endpoint where TWSB is listening.
    zmq_socket.connect("ipc:///tmp/botcmds")
    logging.info("ZMQ PUB socket connected to ipc:///tmp/botcmds")

    app = make_app(index_html)
    app.zmq_socket = zmq_socket
    app.index_html = index_html
    app.listen(8080, address="0.0.0.0")
    logging.info("HTTP/WebSocket server started on port 8080")

    try:
        ioloop.IOLoop.current().start()
    except KeyboardInterrupt:
        logging.info("Shutting down server...")
    finally:
        zmq_socket.close()
        context.term()

if __name__ == "__main__":
    main()
