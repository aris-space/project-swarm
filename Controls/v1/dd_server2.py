# RPi: ZeroMQ Server (in separate thread)
import zmq
import numpy as np

context = zmq.Context()
pub_socket = context.socket(zmq.PUB)  # Telemetry
pub_socket.bind("tcp://*:5555")
rep_socket = context.socket(zmq.REP)  # Parameters
rep_socket.bind("tcp://*:5556")

while True:
    # Send telemetry (e.g., every 50ms)
    pub_socket.send(np.array([0.0,1.0,2.0]))

    # Check for new parameters
    try:
        new_params = rep_socket.recv_json(zmq.NOBLOCK)
        update_pid(new_params)
        rep_socket.send(b"ACK")
    except zmq.Again:
        pass