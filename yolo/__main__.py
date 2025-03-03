from ultralytics import YOLO
import cv2 as cv

model = YOLO(model="yolo11n.pt")

cap = cv.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

while True:
    ret, img = cap.read()
    results = model(img, stream=True)

    for r in results:
        boxes = r.boxes
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

            # render box
            cv.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 3)

            confidence = box.conf[0]
            class_name = box.cls[0]

            cv.putText(img, f"{class_name} {confidence}", [x1, y1], cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

    cv.imshow("Webcam", img)

    if cv.waitKey(1) == ord("q"):
        break

cap.release()
cv.destroyAllWindows()
