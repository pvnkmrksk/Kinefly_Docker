#!/usr/bin/env python3

import socket
import select
import zmq
import json
import math
import click

def process_line(line, socket_zmq):
    """
    Process one line of data from FicTrac.
    Extract FicTrac variables and send the data to ZMQ.
    """
    toks = line.split(", ")

    # Check the data format.
    if len(toks) < 24 or toks[0] != "FT":
        print('Bad read: Unexpected data format.')
        return

    x = float(toks[15])
    y = float(toks[16])
    yaw = float(toks[17])

    data_zmq = {
        "x": x,
        "y": y,
        "z": 0.0,
        "yaw": math.degrees(yaw),
        "pitch": 0.0,
        "roll": 0.0
    }
    try:
        socket_zmq.send_string(json.dumps(data_zmq))
    except Exception as e:
        print(f"Failed to send data over ZMQ. Error: {e}")


def colored_print(message, color):
    color_codes = {
        "red": "\033[91m",
        "green": "\033[92m",
        "reset": "\033[0m"
    }
    print(f"{color_codes[color]}{message}{color_codes['reset']}")


@click.command()
@click.option('--zmq-url', type=str, default='tcp://*:9871', help='ZMQ binding URL.')
@click.option('--sock-host', type=str, default='127.0.0.1', help='UDP socket host IP.')
@click.option('--sock-port', type=int, default=1111, help='UDP socket port.')
@click.option('--timeout', type=int, default=1, help='Timeout for receiving data from FicTrac.')




def main(zmq_url, sock_host, sock_port, timeout):
    """Receive data from FicTrac and transmit it to ZMQ."""

    print(f"\n\n\n\n\nInit ZMQ binding on {zmq_url}...")
    context = zmq.Context()
    socket_zmq = context.socket(zmq.PUB)
    socket_zmq.bind(zmq_url)
    
    print(f"Binding UDP socket on {sock_host}:{sock_port}...")
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind((sock_host, sock_port))
        sock.setblocking(0)

        data = ""
        colored_print("Listening for data...", "green")

        no_data_flag = False  # Introduced a flag here

        while True:
            ready = select.select([sock], [], [], timeout)
            if ready[0]:
                if no_data_flag:  # If the last cycle had no data but now data is incoming.
                    colored_print("\n\nYayyy! Data reception resumed.","green")
                    no_data_flag = False

                try:
                    new_data = sock.recv(1024)
                    if not new_data:
                        print("No data received. Terminating...")
                        break

                    data += new_data.decode('UTF-8')
                    endline = data.find("\n")
                    line = data[:endline]
                    data = data[endline+1:]
                    
                    process_line(line, socket_zmq)
                except Exception as e:
                    print(f"Error processing data: {e}")
            else:
                if not no_data_flag:  # Only print this message the first time data stops coming in.
                    colored_print('\n\nNo data received within timeout period.\nRetrying...', "red")

                no_data_flag = True



if __name__ == '__main__':
    main()
