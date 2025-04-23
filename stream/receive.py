import cv2 as cv

def main():
    cap = cv.VideoCapture('tcp://192.168.8.12:10001')
    while True:
        success, frame = cap.read()
        if not success:
            break

        cv.imshow('Output', frame)

        if cv.waitKey(1) & 0xff == ord('q'):
            break

if __name__ == '__main__':
    main()
