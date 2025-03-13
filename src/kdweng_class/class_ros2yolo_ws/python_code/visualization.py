import cv2
import json

def draw_detections(image_path, json_path):
    # Load the image
    image = cv2.imread(image_path)

    # Load the detections from the JSON file
    with open(json_path, "r") as f:
        detections = json.load(f)

    # Loop through each detection and draw bounding boxes
    for detection in detections:
        bbox = detection["bbox"]  # [x_min, y_min, x_max, y_max]
        confidence = detection["confidence"]  # Confidence score
        class_id = detection["class_id"]  # Class ID

        # Draw the bounding box
        x_min, y_min, x_max, y_max = map(int, bbox)

        cv2.rectangle(image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 1)

        # Add a label with the class ID and confidence score
        label = f"Class {class_id}: {confidence:.2f}"
        cv2.putText(image, label, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

    # Display the image
    # cv2.namedWindow('Detections', cv2.WINDOW_NORMAL)
    cv2.imshow("Detections", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    image_path = "./python_code/test.jpg"  # Path to the input image
    json_path = "./python_code/detections.json"  # Path to the JSON file with detections

    draw_detections(image_path, json_path)
