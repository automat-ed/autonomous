import asyncio
import sys
import socketio

sio = socketio.Client()

@sio.event
def connect():
    print('connected')

@sio.event
def connect_error(e):
    print(e)

@sio.event
def disconnect():
    print('disconnected')

def main():
    sio.connect('http://localhost:2000')
    sio.wait()

if __name__ == '__main__':
    main()
