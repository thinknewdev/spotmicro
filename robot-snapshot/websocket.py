import socketio

sio = socketio.Client()

@sio.on("message")
def on_message(data):
    print("Received:", data)

sio.connect("http://127.0.0.1:5000")
sio.emit("message", "Hello Server!")
sio.wait()
