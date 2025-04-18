from ultralytics import YOLO
from multiprocessing import Pool
import cv2 as cv

model = YOLO("basketball-nano.pt", task="detect")

def track(img: cv.Mat):
    global model
    return list(model.track(img, stream=True))

def main():
    pool = Pool(1)
    results = None
    track_future = None
    duration = None

    cap = cv.VideoCapture('test.webm')

    while True:
        success, img = cap.read()
        if not success:
            break

        if not track_future:
            track_future = pool.apply_async(track, (img,))
        
        if track_future.ready():
            results = track_future.get()
            track_future = None

        if results:
            for result in results:
                for box in result.boxes:
                    cls = box.cls
                    x1, y1, x2, y2 = [int(a) for a in box.xyxy[0]]
                    color = None
                    match cls[0]:
                        case 0:
                            color = (255, 0, 0)
                        case 1:
                            color = (0, 255, 0)
                    cv.rectangle(img, (x1, y1), (x2, y2), color, 2)
        
        cv.imshow("Output", img)
        if cv.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv.destroyAllWindows()
    pool.close()

if __name__ == '__main__':
    main()
