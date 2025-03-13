from ultralytics import YOLO
import cv2
import json

def detect_image(model_path, image_path):
    # Initialize the YOLO model
    model = YOLO(model_path)

    # Load the image
    image = cv2.imread(image_path)

    # Run inference
    results = model(image)

    # Extract bounding boxes and other details
    detections = []
    for result in results[0].boxes:
        bbox = result.xyxy[0].tolist()  # Bounding box coordinates [x_min, y_min, x_max, y_max]
        confidence = result.conf[0].item()  # Confidence score
        class_id = result.cls[0].item()  # Class ID
        detections.append({
            "bbox": bbox,
            "confidence": confidence,
            "class_id": class_id
        })

    # Send detections to another node or save to a file
    with open("./python_code/detections.json", "w") as f:
        json.dump(detections, f)
    print("Detections saved to detections.json")

    # Render predictions on the image
    annotated_image = results[0].plot()


    # Display the annotated image
    # cv2.namedWindow('Detection Results', cv2.WINDOW_NORMAL)
    cv2.imshow("Detection Results", annotated_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    model_path = "yolov9s.pt"  # Path to the YOLOv9 model file
    image_path = "./python_code/test.jpg"  # Path to the input image

    detect_image(model_path, image_path)
