#!/usr/bin/env python3
"""Measures the server-side CPU cost of one WebSocket frame.

Run by bench_protocol.py, not directly interesting on its own.

The client is a separate *process* deliberately. An in-process client
charges its own frame decoding to this process's CPU time, which roughly
doubles the apparent per-frame cost and makes coalescing look like a much
bigger win than it is. What we want is only what the server pays.

Prints one line of JSON on stdout.
"""

import asyncio
import json
import socket
import subprocess
import sys
import threading
import time

import tornado.ioloop
import tornado.web
import tornado.websocket

CLIENTS = set()


class _Echo(tornado.websocket.WebSocketHandler):
    def check_origin(self, origin):
        return True

    def open(self):
        CLIENTS.add(self)

    def on_close(self):
        CLIENTS.discard(self)


def _free_port():
    probe = socket.socket()
    probe.bind(('127.0.0.1', 0))
    port = probe.getsockname()[1]
    probe.close()
    return port


CLIENT_SOURCE = '''
import asyncio, sys
from tornado.websocket import websocket_connect
async def main():
    connection = await websocket_connect("ws://127.0.0.1:%d/ws")
    while True:
        if await connection.read_message() is None:
            break
asyncio.run(main())
'''


def main():
    port = _free_port()
    holder = {}

    def serve():
        asyncio.set_event_loop(asyncio.new_event_loop())
        tornado.web.Application([(r'/ws', _Echo)]).listen(port, address='127.0.0.1')
        holder['loop'] = tornado.ioloop.IOLoop.current()
        holder['loop'].start()

    threading.Thread(target=serve, daemon=True).start()
    for _ in range(100):
        if 'loop' in holder:
            break
        time.sleep(0.05)
    loop = holder['loop']

    client = subprocess.Popen([sys.executable, '-c', CLIENT_SOURCE % port],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for _ in range(100):
        if CLIENTS:
            break
        time.sleep(0.05)
    if not CLIENTS:
        client.kill()
        raise SystemExit('the benchmark client never connected')

    def send(payload):
        for connection in list(CLIENTS):
            connection.write_message(payload)

    def measure(payload, count):
        time.sleep(0.3)
        start = time.process_time()
        for _ in range(count):
            loop.add_callback(send, payload)
            time.sleep(0.0005)
        elapsed = time.process_time() - start
        time.sleep(0.3)
        return elapsed / count

    one = json.dumps({'type': 'pose', 'x': 1.0, 'y': 2.0, 'yaw': 0.3, 'stamp': 1.0})
    many = json.dumps({'type': 'batch', 'items': [json.loads(one)] * 8})

    single = measure(one, 2000)
    batch = measure(many, 500)

    client.kill()
    print(json.dumps({'single_us': single * 1e6, 'batch_us': batch * 1e6}))


if __name__ == '__main__':
    main()
