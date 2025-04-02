import cv2 as cv
# import socket
# import struct
# import numpy as np

# SERVER_IP = '192.168.8.12'
# # SERVER_IP = '127.0.0.1'
# SERVER_PORT = 10001

BUFFER_SIZE = 1024

# def receive(sock: socket.socket, data: bytes, size: int):
#     while len(data) < size:
#         data += sock.recv(BUFFER_SIZE)
#     return (data[:size], data[size:])

def main():
    cap = cv.VideoCapture('tcp://192.168.8.12:10001')
    while True:
        success, frame = cap.read()
        if not success:
            break

        cv.imshow('Output', frame)

        if cv.waitKey(1) & 0xff == ord('q'):
            break
    # with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
    #     sock.connect((SERVER_IP, SERVER_PORT))
    #     try:
    #         data = bytes()
    #         shape_data_size = struct.calcsize('>LLL')
    #         while True:
    #             shape_data, data = receive(sock, data, shape_data_size)
    #             shape = struct.unpack('>LLL', shape_data)

    #             frame_data, data = receive(sock, data, shape[0] * shape[1] * shape[2])
    #             image = np.frombuffer(frame_data, dtype=np.uint8).reshape(shape)
                
    #             cv.imshow('result', image)

    #             if cv.waitKey(1) & 0xff == ord('q'):
    #                 break
    #     finally:
    #         cv.destroyAllWindows()

if __name__ == '__main__':
    main()
