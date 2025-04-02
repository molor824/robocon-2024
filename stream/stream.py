import cv2 as cv
import socket
import struct

def main():
    cap = cv.VideoCapture(0)
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('0.0.0.0', 8123))
    server_socket.listen(10)

    client_socket, client_address = server_socket.accept()
    print(f"Accepted connection from {client_address}")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame_data = frame.tobytes()
        frame_shape_data = struct.pack('>LLL', *frame.shape)
        client_socket.sendall(frame_shape_data + frame_data)

        if cv.waitKey(1) & 0xff == ord('q'):
            break

if __name__ == '__main__':
    main()
