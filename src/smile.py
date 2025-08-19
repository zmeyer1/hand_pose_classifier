# uses mediapose to detect a face and extracts the position of the mouth
# and eyes to classify the face as smiling or not

import mediapipe as mp
import cv2
import numpy as np  
from mediapipe import solutions

mp_face_mesh = mp.solutions.face_mesh
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_face_connections = mp_face_mesh.FACEMESH_CONTOURS

class SmileDetector:
    def __init__(self, min_detection_confidence=0.5):
        self.face_mesh = mp_face_mesh.FaceMesh(
            static_image_mode=False,
            max_num_faces=1,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=0.5)

    def detect(self, image):
        results = self.face_mesh.process(image)
        return results

    def draw_landmarks(self, image, results):
        if results.multi_face_landmarks:
            for face_landmarks in results.multi_face_landmarks:
                mp_drawing.draw_landmarks(
                    image=image,
                    landmark_list=face_landmarks,
                    connections=mp_face_connections,
                    landmark_drawing_spec=mp_drawing_styles.get_default_face_mesh_tesselation_style(),
                    connection_drawing_spec=mp_drawing_styles.get_default_face_mesh_contours_style())
        return image

    def is_smiling(self, results):
        if not results.multi_face_landmarks:
            return False
        
        for face_landmarks in results.multi_face_landmarks:
            # Get the coordinates of the mouth landmarks
            upper_lip = face_landmarks.landmark[61]
            lower_lip = face_landmarks.landmark[185]
            left_corner = face_landmarks.landmark[61]
            right_corner = face_landmarks.landmark[291]

            # Calculate the distance between upper and lower lip
            lip_distance = np.linalg.norm(np.array([upper_lip.x, upper_lip.y]) - np.array([lower_lip.x, lower_lip.y]))
            # Calculate the distance between left and right corners of the mouth
            mouth_width = np.linalg.norm(np.array([left_corner.x, left_corner.y]) - np.array([right_corner.x, right_corner.y]))
            # A simple heuristic: if the lip distance is greater than a threshold relative to the mouth width, consider it a smile
            if lip_distance > 0.1 * mouth_width:
                return True
        return False


if __name__ == "__main__":
    # Example usage
    cap = cv2.VideoCapture(0)
    smile_detector = SmileDetector()

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert to Gray
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)  # Convert back to BGR because mediapipe expects RGB input

        results = smile_detector.detect(frame)
        frame = smile_detector.draw_landmarks(frame, results)

        # add text to the frame indicating if the person is smiling
        if smile_detector.is_smiling(results):
            cv2.putText(frame, "Smiling", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        else:
            cv2.putText(frame, "Not Smiling", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

        cv2.imshow('Smile Detector', frame)

        if cv2.waitKey(5) & 0xFF == 27:  # Press 'ESC' to exit
            break

    cap.release()
    cv2.destroyAllWindows()