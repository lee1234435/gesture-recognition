# import cv2
# import mediapipe as mp
# import numpy as np
# from tensorflow.keras.models import load_model

# actions = ['1','2','3','O','X']
# seq_length = 30

# model = load_model('gesture-recognition/models/model2_1.4.keras')

# # MediaPipe hands model
# mp_hands = mp.solutions.hands
# mp_drawing = mp.solutions.drawing_utils
# hands = mp_hands.Hands(
#     max_num_hands=1,
#     min_detection_confidence=0.8,
#     min_tracking_confidence=0.8)

# cap = cv2.VideoCapture(0)

# # w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
# # h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
# # fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')Q
# # out = cv2.VideoWriter('input.mp4', fourcc, cap.get(cv2.CAP_PROP_FPS), (w, h))
# # out2 = cv2.VideoWriter('output.mp4', fourcc, cap.get(cv2.CAP_PROP_FPS), (w, h))

# seq = []
# action_seq = []

# while cap.isOpened():
#     ret, img = cap.read()
#     img0 = img.copy()

#     img = cv2.flip(img, 1)
#     img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
#     result = hands.process(img)
#     img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

#     if result.multi_hand_landmarks is not None:
#         for res in result.multi_hand_landmarks:
#             joint = np.zeros((21, 4))
#             for j, lm in enumerate(res.landmark):
#                 joint[j] = [lm.x, lm.y, lm.z, lm.visibility]

#             # Compute angles between joints
#             v1 = joint[[0,1,2,3,0,5,6,7,0,9,10,11,0,13,14,15,0,17,18,19], :3] # Parent joint
#             v2 = joint[[1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20], :3] # Child joint
#             v = v2 - v1 # [20, 3]
#             # Normalize v
#             v = v / np.linalg.norm(v, axis=1)[:, np.newaxis]

#             # Get angle using arcos of dot product
#             angle = np.arccos(np.einsum('nt,nt->n',
#                 v[[0,1,2,4,5,6,8,9,10,12,13,14,16,17,18],:], 
#                 v[[1,2,3,5,6,7,9,10,11,13,14,15,17,18,19],:])) # [15,]

#             angle = np.degrees(angle) # Convert radian to degree

#             d = np.concatenate([joint.flatten(), angle])

#             seq.append(d)

#             mp_drawing.draw_landmarks(img, res, mp_hands.HAND_CONNECTIONS)

#             if len(seq) < seq_length:
#                 continue

#             input_data = np.expand_dims(np.array(seq[-seq_length:], dtype=np.float32), axis=0)

#             y_pred = model.predict(input_data).squeeze()

#             i_pred = int(np.argmax(y_pred))
#             conf = y_pred[i_pred]

#             if conf < 0.95:
#                 continue

#             action = actions[i_pred]
#             action_seq.append(action)
#             if len(action_seq) < 12: # 액션 시퀀스 길이가 3보다 작으면 계속
#                 continue

#             this_action = '?' # 기본 값 설정
#             if action_seq[-1] == action_seq[-2] == action_seq[-3]== action_seq[-4]== action_seq[-5]== action_seq[-6]== action_seq[-7]== action_seq[-8]== action_seq[-9]== action_seq[-10]== action_seq[-11]== action_seq[-12]:
#                         # if action_seq[-1] == action_seq[-2] == action_seq[-3]: # 마지막 3개의 액션이 같으면
#                 this_action = action # 해당 액션 설정
#             cv2.putText(img, f'{this_action.upper()}', org=(int(res.landmark[0].x * img.shape[1]), int(res.landmark[0].y * img.shape[0] + 20)), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0, 0, 255), thickness=2)
               
#     # out.write(img0)
#     # out2.write(img)
#     cv2.imshow('img', img)
#     if cv2.waitKey(1) == ord('q'):
#         break


# import cv2
# import mediapipe as mp
# import numpy as np
# from tensorflow.keras.models import load_model
# from collections import deque
# import time

# actions = ['1', '2', '3', 'O', 'X']
# seq_length = 30
# confidence_threshold = 0.98
# consistent_action_threshold = 15
# time_threshold = 0.5  # seconds

# model = load_model('gesture-recognition/models/model2_1.4.keras')

# # MediaPipe hands model
# mp_hands = mp.solutions.hands
# mp_drawing = mp.solutions.drawing_utils
# hands = mp_hands.Hands(
#     max_num_hands=1,
#     min_detection_confidence=0.8,
#     min_tracking_confidence=0.8)

# cap = cv2.VideoCapture(0)

# seq = []
# action_seq = deque(maxlen=consistent_action_threshold)
# action_start_time = None

# while cap.isOpened():
#     ret, img = cap.read()
#     img0 = img.copy()

#     img = cv2.flip(img, 1)
#     img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
#     result = hands.process(img)
#     img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

#     if result.multi_hand_landmarks is not None:
#         for res in result.multi_hand_landmarks:
#             joint = np.zeros((21, 4))
#             for j, lm in enumerate(res.landmark):
#                 joint[j] = [lm.x, lm.y, lm.z, lm.visibility]

#             # Compute angles between joints
#             v1 = joint[[0,1,2,3,0,5,6,7,0,9,10,11,0,13,14,15,0,17,18,19], :3] # Parent joint
#             v2 = joint[[1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20], :3] # Child joint
#             v = v2 - v1 # [20, 3]
#             # Normalize v
#             v = v / np.linalg.norm(v, axis=1)[:, np.newaxis]

#             # Get angle using arcos of dot product
#             angle = np.arccos(np.einsum('nt,nt->n',
#                 v[[0,1,2,4,5,6,8,9,10,12,13,14,16,17,18],:], 
#                 v[[1,2,3,5,6,7,9,10,11,13,14,15,17,18,19],:])) # [15,]

#             angle = np.degrees(angle) # Convert radian to degree

#             d = np.concatenate([joint.flatten(), angle])

#             seq.append(d)

#             mp_drawing.draw_landmarks(img, res, mp_hands.HAND_CONNECTIONS)

#             if len(seq) < seq_length:
#                 continue

#             input_data = np.expand_dims(np.array(seq[-seq_length:], dtype=np.float32), axis=0)

#             y_pred = model.predict(input_data).squeeze()

#             i_pred = int(np.argmax(y_pred))
#             conf = y_pred[i_pred]

#             if conf < confidence_threshold:
#                 action_start_time = None
#                 continue

#             action = actions[i_pred]
#             action_seq.append(action)

#             if len(action_seq) == consistent_action_threshold:
#                 if len(set(action_seq)) == 1:
#                     current_time = time.time()
#                     if action_start_time is None:
#                         action_start_time = current_time
#                     else:
#                         if current_time - action_start_time >= time_threshold:
#                             this_action = action_seq[0]
#                             action_start_time = None
#                             cv2.putText(img, f'{this_action.upper()}', org=(int(res.landmark[0].x * img.shape[1]), int(res.landmark[0].y * img.shape[0] + 20)), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0, 0, 255), thickness=2)
#                 else:
#                     action_start_time = None
#             else:
#                 action_start_time = None

#     cv2.imshow('img', img)
#     if cv2.waitKey(1) == ord('q'):
#         break

# cap.release()
# cv2.destroyAllWindows()

# import cv2
# import mediapipe as mp
# import numpy as np
# from tensorflow.keras.models import load_model
# from collections import deque
# import time

# actions = ['1', '2', '3', 'O', 'X']
# seq_length = 30
# confidence_threshold = 0.95
# consistent_action_threshold = 15
# time_threshold = 0.25  # seconds

# model = load_model('gesture-recognition/models/model2_1.4.keras')

# # MediaPipe hands model
# mp_hands = mp.solutions.hands
# mp_drawing = mp.solutions.drawing_utils
# hands = mp_hands.Hands(
#     max_num_hands=1,
#     min_detection_confidence=0.8,
#     min_tracking_confidence=0.8)

# cap = cv2.VideoCapture(0)

# seq = []
# action_seq = deque(maxlen=consistent_action_threshold)
# action_start_time = None

# def process_landmarks(landmarks):
#     # 리스트 컴프리헨션의 올바른 구문을 사용하여 landmarks 리스트를 처리
#     return [[lm.x * 100, lm.y * 100, lm.z * 100] for lm in landmarks]


# while cap.isOpened():
#     ret, img = cap.read()
#     img0 = img.copy()

#     img = cv2.flip(img, 1)
#     img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
#     result = hands.process(img)
#     img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

#     if result.multi_hand_landmarks is not None:
#         for res in result.multi_hand_landmarks:
#             joint = np.zeros((21, 4))
#             for j, lm in enumerate(res.landmark):
#                 joint[j] = [lm.x, lm.y, lm.z, lm.visibility]

#             # Compute angles between joints
#             v1 = joint[[0,1,2,3,0,5,6,7,0,9,10,11,0,13,14,15,0,17,18,19], :3] # Parent joint
#             v2 = joint[[1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20], :3] # Child joint
#             v = v2 - v1 # [20, 3]
#             # Normalize v
#             v = v / np.linalg.norm(v, axis=1)[:, np.newaxis]

#             # Get angle using arcos of dot product
#             angle = np.arccos(np.einsum('nt,nt->n',
#                 v[[0,1,2,4,5,6,8,9,10,12,13,14,16,17,18],:], 
#                 v[[1,2,3,5,6,7,9,10,11,13,14,15,17,18,19],:])) # [15,]

#             angle = np.degrees(angle) # Convert radian to degree

#             d = np.concatenate([joint.flatten(), angle])

#             seq.append(d)

#             mp_drawing.draw_landmarks(img, res, mp_hands.HAND_CONNECTIONS)

#             if len(seq) < seq_length:
#                 continue

#             input_data = np.expand_dims(np.array(seq[-seq_length:], dtype=np.float32), axis=0)

#             y_pred = model.predict(input_data).squeeze()

#             i_pred = int(np.argmax(y_pred))
#             conf = y_pred[i_pred]

#             if conf < confidence_threshold:
#                 action_start_time = None
#                 continue

#             action = actions[i_pred]
#             action_seq.append(action)

#             if len(action_seq) == consistent_action_threshold:
#                 if len(set(action_seq)) == 1:
#                     current_time = time.time()
#                     if action_start_time is None:
#                         action_start_time = current_time
#                     else:
#                         if current_time - action_start_time >= time_threshold:
#                             this_action = action_seq[0]
#                             action_start_time = None
#                             cv2.putText(img, f'{this_action.upper()}', org=(int(res.landmark[0].x * img.shape[1]), int(res.landmark[0].y * img.shape[0] + 20)), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0, 0, 255), thickness=2)
#                 else:
#                     action_start_time = None
#             else:
#                 action_start_time = None

#     cv2.imshow('img', img)
#     if cv2.waitKey(1) == ord('q'):
#         break

# cap.release()
# cv2.destroyAllWindows()


import cv2
import mediapipe as mp
import numpy as np
from tensorflow.keras.models import load_model
from collections import deque
import time

actions = ['1', '2', '3', 'O', 'X']
seq_length = 30
confidence_threshold = 0.95
consistent_action_threshold = 15
time_threshold = 0.5  # seconds

model = load_model('gesture-recognition/models/model2_1.4.keras')

# MediaPipe hands model
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
hands = mp_hands.Hands(
    max_num_hands=1,
    min_detection_confidence=0.8,
    min_tracking_confidence=0.8)

cap = cv2.VideoCapture(0)

seq = []
action_seq = deque(maxlen=consistent_action_threshold)
action_start_time = None

def process_landmarks(landmarks):
    return [[lm.x * 100, lm.y * 100, lm.z * 100] for lm in landmarks]

while cap.isOpened():
    ret, img = cap.read()
    img0 = img.copy()

    img = cv2.flip(img, 1)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    result = hands.process(img)
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    if result.multi_hand_landmarks is not None:
        for res in result.multi_hand_landmarks:
            joint = np.zeros((21, 4))
            for j, lm in enumerate(res.landmark):
                joint[j] = [lm.x, lm.y, lm.z, lm.visibility]

            # Compute angles between joints
            v1 = joint[[0,1,2,3,0,5,6,7,0,9,10,11,0,13,14,15,0,17,18,19], :3] # Parent joint
            v2 = joint[[1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20], :3] # Child joint
            v = v2 - v1 # [20, 3]
            # Normalize v
            v = v / np.linalg.norm(v, axis=1)[:, np.newaxis]

            # Get angle using arcos of dot product
            angle = np.arccos(np.einsum('nt,nt->n',
                v[[0,1,2,4,5,6,8,9,10,12,13,14,16,17,18],:], 
                v[[1,2,3,5,6,7,9,10,11,13,14,15,17,18,19],:])) # [15,]

            angle = np.degrees(angle) # Convert radian to degree

            d = np.concatenate([joint.flatten(), angle])

            seq.append(d)

            mp_drawing.draw_landmarks(img, res, mp_hands.HAND_CONNECTIONS)

            if len(seq) < seq_length:
                continue

            input_data = np.expand_dims(np.array(seq[-seq_length:], dtype=np.float32), axis=0)

            y_pred = model.predict(input_data).squeeze()

            i_pred = int(np.argmax(y_pred))
            conf = y_pred[i_pred]

            if conf < confidence_threshold:
                action_start_time = None
                action_seq.clear()  # Reset the action sequence as well
                continue

            action = actions[i_pred]
            action_seq.append(action)

            if len(action_seq) == consistent_action_threshold:
                if len(set(action_seq)) == 1:
                    current_time = time.time()
                    if action_start_time is None:
                        action_start_time = current_time
                    else:
                        if current_time - action_start_time >= time_threshold:
                            this_action = action_seq[0]
                            action_start_time = None  # Reset the timer after detecting the action
                            cv2.putText(img, f'{this_action.upper()}', org=(int(res.landmark[0].x * img.shape[1]), int(res.landmark[0].y * img.shape[0] + 20)), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0, 0, 255), thickness=2)
                else:
                    action_start_time = None
                    # action_seq.clear()  # Reset the action sequence if the detected actions are not consistent
            else:
                action_start_time = None
                # action_seq.clear()  # Reset the action sequence if it is not yet filled

    cv2.imshow('img', img)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
