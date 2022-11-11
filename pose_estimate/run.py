import json

import cv2
import mediapipe as mp
import numpy as np
import socket


# For adding new landmarks based on default predicted landmarks
def add_extra_points(landmark_list):
    left_shoulder = landmark_list[11]
    right_shoulder = landmark_list[12]
    left_hip = landmark_list[23]
    right_hip = landmark_list[24]

    # Calculating hip position and visibility
    hip = {
          'x': (left_hip['x'] + right_hip['x']) / 2.0,
          'y': (left_hip['y'] + right_hip['y']) / 2.0,
          'z': (left_hip['z'] + right_hip['z']) / 2.0,
          'visibility': (left_hip['visibility'] + right_hip['visibility']) / 2.0
        }
    landmark_list.append(hip)

    # Calculating spine position and visibility
    spine = {
          'x': (left_hip['x'] + right_hip['x'] + right_shoulder['x'] + left_shoulder['x']) / 4.0,
          'y': (left_hip['y'] + right_hip['y'] + right_shoulder['y'] + left_shoulder['y']) / 4.0,
          'z': (left_hip['z'] + right_hip['z'] + right_shoulder['z'] + left_shoulder['z']) / 4.0,
          'visibility': (left_hip['visibility'] + right_hip['visibility'] + right_shoulder['visibility'] + left_shoulder['visibility']) / 4.0
        }
    landmark_list.append(spine)

    left_mouth = landmark_list[9]
    right_mouth = landmark_list[10]
    nose = landmark_list[0]
    left_ear = landmark_list[7]
    right_ear = landmark_list[8]
    # Calculating neck position and visibility
    neck = {
          'x': (left_mouth['x'] + right_mouth['x'] + right_shoulder['x'] + left_shoulder['x']) / 4.0,
          'y': (left_mouth['y'] + right_mouth['y'] + right_shoulder['y'] + left_shoulder['y']) / 4.0,
          'z': (left_mouth['z'] + right_mouth['z'] + right_shoulder['z'] + left_shoulder['z']) / 4.0,
          'visibility': (left_mouth['visibility'] + right_mouth['visibility'] + right_shoulder['visibility'] + left_shoulder['visibility']) / 4.0
        }
    landmark_list.append(neck)

    # Calculating head position and visibility
    head = {
          'x': (nose['x'] + left_ear['x'] + right_ear['x']) / 3.0,
          'y': (nose['y'] + left_ear['y'] + right_ear['y']) / 3.0,
          'z': (nose['z'] + left_ear['z'] + right_ear['z']) / 3.0,
          'visibility': (nose['visibility'] + left_ear['visibility'] + right_ear['visibility']) / 3.0,
        }
    landmark_list.append(head)


def world_landmarks_list_to_array(landmark_list, image_shape):
    rows, cols, _ = image_shape
    array = []
    for lmk in landmark_list.landmark:
        new_row = {
          'x': lmk.x * cols,
          'y': lmk.y * rows,
          'z': lmk.z * cols,
          'visibility': lmk.visibility
        }
        array.append(new_row)
    return array
    return np.asarray([(lmk.x * cols, lmk.y * rows, lmk.z * cols,lmk.visibility)
                       for lmk in landmark_list.landmark])


def landmarks_list_to_array(landmark_list):

    array = []
    for lmk in landmark_list.landmark:
        new_row = {
          'x': lmk.x,
          'y': lmk.y,
          'z': lmk.z,
          'visibility': lmk.visibility
        }
        array.append(new_row)
    return array
    return np.asarray([(lmk.x, lmk.y, lmk.z, lmk.visibility)
                       for lmk in landmark_list.landmark])





def Save_Json(path, index,dump_data):
    json_path = path + "" + str(index) + ".json"
    with open(json_path, 'w') as fl:
        # np.around(pose_landmarks, 4).tolist()
        fl.write(json.dumps(dump_data, indent=2, separators=(',', ': ')))
        fl.close()



def Pose_Images():
    mp_drawing = mp.solutions.drawing_utils
    mp_drawing_styles = mp.solutions.drawing_styles
    mp_pose = mp.solutions.pose
    # For static images:
    IMAGE_FILES = ["test.jpg"]
    BG_COLOR = (192, 192, 192) # gray
    with mp_pose.Pose(
        static_image_mode=True,
        model_complexity=2,
        enable_segmentation=True,
        min_detection_confidence=0) as pose:
        cap = cv2.VideoCapture(0)
      while(True):
        hx, image = cap.read() 
        if hx is False:
            print('read video error')
            exit(0)
        image_height, image_width, _ = image.shape
        # Convert the BGR image to RGB before processing.
        results = pose.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        new_pose = world_landmarks_list_to_array(
                results.pose_world_landmarks, image.shape)
        pose_landmarks = landmarks_list_to_array(results.pose_landmarks)
        print(results.pose_landmarks)
        print(pose_landmarks)
        #send pose 
        pose_data = {
            'predictions': pose_landmarks,
            'predictions_world': new_pose
        }
        json_data = json.dumps(pose_data)
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        dest_addr = ('127.0.0.1', 10360)
        text = json_data.encode('utf-8')
        udp_socket.sendto(text, dest_addr)
        
        if cv.waitKey(1) & 0xFF == ord('q'):       # 按q退出
        break
      cap.release()


if __name__ == '__main__':
    Pose_Images()