
#!/usr/bin/env python3

# S/N : XYZARIS0V3P2311N02
# Robot IP : 192.168.1.192
# code_version : 3.1.5.2

import cv2
import mediapipe as mp
import numpy as np
import socket
import json
import os
import sys
import math
import time
import queue
import datetime
import random
import traceback
import threading
import bluetooth
import speech_recognition as sr
import playsound

from threading import Timer 
from scipy.optimize import minimize
from xarm import version
from xarm.wrapper import XArmAPI
from tensorflow.keras.models import load_model
from threading import Thread, Event
from playsound import playsound
from gtts import gTTS

class RobotMain(object):
    """Robot Main Class"""
    
    # =============================== init parameter ========================================= #    
    def __init__(self, robot, **kwargs):
        self.alive = True
        self._arm = robot
        self._tcp_speed = 100
        self._tcp_acc = 2000
        self._angle_speed = 20
        self._angle_acc = 500
        self._vars = {}
        self._funcs = {}
        self._robot_init()
        self.state = 'stopped'
        self.start = 0
        self.audio_file = 'refresh.mp3'
        self.current_state = np.array([269.4, -20.1, 20.4, 176.9, 53.4, -1.1])
        self.dh_params = [ [0, 243.3, 0, -90],
                           [-90, 0, 200, 180],
                           [-90, 0, 87, 90],
                           [0, 227.6, 0, 90],
                           [0, 0, 0, -90],
                           [0, 61.5, 0, 0] ]
        self.coordinates = []
        # self.coordinates = queue.Queue()
        self.solutions = []
        self.last_processed_time = time.time()
        self.process_interval = 1  # 1초 간격으로 좌표 처리
        self.cleanup_interval = 3 # per 3 sec

        
        self.position_home = [179.2, -42.1, 7.4, 186.7, 41.5, -1.6] #angle
        self.position_jig_A_grab = [-257.3, -138.3, 198, 68.3, 86.1, -47.0] #linear
        self.position_jig_B_grab = [-152.3, -129.0, 198, 4.8, 89.0, -90.7] #linear
        self.position_jig_C_grab = [-76.6, -144.6, 198, 5.7, 88.9, -50.1] #linear
        self.position_sealing_check = [-136.8, 71.5, 307.6, 69.6, -73.9, -59] #Linear
        self.position_capsule_place = [234.9, 135.9, 465.9, 133.6, 87.2, -142.1] #Linear
        self.position_before_capsule_place = self.position_capsule_place.copy()
        self.position_before_capsule_place[2] += 25
        self.position_cup_grab = [214.0, -100.2, 145.0, -25.6, -88.5, 95.8] #linear
        self.position_topping_A = [-200.3, 162.8, 359.9, -31.7, 87.8, 96.1] #Linear
        self.position_topping_B = [106.5, -39.7, 15.0, 158.7, 40.4, 16.9] #Angle
        self.position_topping_C = [43.6, 137.9, 350.1, -92.8, 87.5, 5.3] #Linear
        self.position_icecream_with_topping = [168.7, 175.6, 359.5, 43.9, 88.3, 83.3] #Linear
        self.position_icecream_no_topping = [48.4, -13.8, 36.3, 193.6, 42.0, -9.2] #angle
        self.position_jig_A_serve = [-258.7, -136.4, 208.2, 43.4, 88.7, -72.2] #Linear
        self.position_jig_B_serve = [-166.8, -126.5, 200.9, -45.2, 89.2, -133.6] #Linear
        self.position_jig_C_serve = [-63.1, -138.2, 199.5, -45.5, 88.1, -112.1] #Linear
        self.position_capsule_grab = [234.2, 129.8, 464.5, -153.7, 87.3, -68.7] #Linear
        
        self.this_action = '?' # 현재 동작
        self.actions = ['banana', 'choco', 'strawberry'] # 가능한 동작 리스트
        self.seq_length = 30 # 시퀀스 길이
        self.model = load_model('/home/lee/Desktop/hand/gesture-recognition/models/model2_1.0.keras') # 동작 인식 모델
        self.seq = [] # 관절 데이터 시퀀스
        self.action_seq = [] # 동작 시퀀스

        self.mp_hands = mp.solutions.hands # 미디어파이프 손 모듈
        self.mp_drawing = mp.solutions.drawing_utils # 미디어파이프 그리기 도구
        self.hands = self.mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7
        )
        self.cap = cv2.VideoCapture(0) # 웹캠에서 비디오 캡처 시작
        # self.cap.set(cv2.CAP_PROP_BUFFERSIZE,1)

        self.camera_matrix = None
        # self.cap = None
        self.lock = threading.Lock()
        

        self.last_processed_time = time.time()
        self.start_cleanup_timer()
        # self.comm_thread = threading.Thread(target=self.send_coordinates_thread)
        # self.comm_thread.start()
        # self.comm_thread.daemon = True
    # =============================== init, error state ========================================= #
    def _robot_init(self):
        self._arm.clean_warn()
        self._arm.clean_error()
        self._arm.motion_enable(True)
        self._arm.set_mode(0)
        self._arm.set_state(0)
        time.sleep(1)
        self._arm.register_error_warn_changed_callback(self._error_warn_changed_callback)
        self._arm.register_state_changed_callback(self._state_changed_callback)
        
        # self._arm.motion_enable(enable=True)
        # self._arm.set_gravity_direction([0, 0, 9.8])
        # self._arm.reset(wait=True)
        
        
        
        if hasattr(self._arm, 'register_count_changed_callback'):
            self._arm.register_count_changed_callback(self._count_changed_callback)

    # ===============================  Register error/warn changed callback =============================== #
    def _error_warn_changed_callback(self, data):
        if data and data['error_code'] != 0:
            self.alive = False
            self.pprint('err={}, quit'.format(data['error_code']))
            self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)

    # ===============================  Register state changed callback =============================== #
    def _state_changed_callback(self, data):
        if data and data['state'] == 4:
            self.alive = False
            self.pprint('state=4, quit')
            self._arm.release_state_changed_callback(self._state_changed_callback)

    # ===============================  Register count changed callback =============================== #
    def _count_changed_callback(self, data):
        if self.is_alive:
            self.pprint('counter val: {}'.format(data['count']))

    # ===============================  Register count changed callback =============================== #
    def _check_code(self, code, label):
        if not self.is_alive or code != 0:
            self.alive = False
            ret1 = self._arm.get_state()
            ret2 = self._arm.get_err_warn_code()
            self.pprint('{}, code={}, connected={}, state={}, error={}, ret1={}. ret2={}'.format(label, code,
                                                                                                    self._arm.connected,
                                                                                                    self._arm.state,
                                                                                                    self._arm.error_code,
                                                                                                    ret1, ret2))
        return self.is_alive

    @staticmethod
    def pprint(*args, **kwargs):
        try:
            stack_tuple = traceback.extract_stack(limit=2)[0]
            print('[{}][{}] {}'.format(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())), stack_tuple[1],
                                        ' '.join(map(str, args))))
        except:
            print(*args, **kwargs)

    @property
    def arm(self):
        return self._arm

    @property
    def VARS(self):
        return self._vars

    @property
    def FUNCS(self):
        return self._funcs

    @property
    def is_alive(self):
        if self.alive and self._arm.connected and self._arm.error_code == 0:
            # print("True --------")
            # print(self.alive)
            # print(self._arm.connected)
            # print(self._arm.error_code)
            if self._arm.state == 5:
                cnt = 0
                while self._arm.state == 5 and cnt < 5:
                    cnt += 1
                    time.sleep(0.1)
            return self._arm.state < 4 # 만약 상태 값이 4보다 작다면 True, 그렇지 않으면 False를 반환. 즉 , self._arm.state이 0,1,2,3 일때만 true로 작동 가능한 상태
        else:
            # print("False --------")
            # print(self.alive)
            # print(self._arm.connected)
            # print(self._arm.error_code)
            return False

    # ===============================  fail sealing =============================== #
    def position_reverse_sealing_fail(self, linear_jig_position = [-257.3, -138.3, 192.1, 68.3, 86.1, -47.0]):
        reverse_position = linear_jig_position.copy()
        reverse_position[2] = reverse_position[2] - 10
        reverse_position[3] = -reverse_position[3]
        reverse_position[4] = -reverse_position[4]
        reverse_position[5] = reverse_position[5] - 180
        return reverse_position

    # ===============================  socket connecting (client) =============================== #
    def socket_connect(self):

        self.HOST = '192.168.1.192'
        self.PORT = 20002
        self.BUFSIZE = 1024
        self.ADDR = (self.HOST, self.PORT)

        # self.serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.clientSocket.shutdown(1)
            self.clientSocket.close()
        except:
            pass

        self.serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self
        self.serverSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # self.serverSocket.allow_reuse_address = True
        while True:
            try:
                self.serverSocket.bind(self.ADDR)
                print("bind")

                while True:
                    self.serverSocket.listen(1)
                    print(f'[LISTENING] Server is listening on robot_server')
                    time.sleep(1)
                    try:
                        while True:
                            try:
                                self.clientSocket, addr_info = self.serverSocket.accept()
                                print("socket accepted")
                                break
                            except:
                                time.sleep(1)
                                print('except')
                                # break

                        break

                    except socket.timeout:
                        print("socket timeout")

                    except:
                        pass
                break
            except:
                pass
        # self.clientSocket.settimeout(10.0)
        print("accept")
        print("--client info--")
        # print(self.clientSocket)

        self.connected = True
        self.state = 'ready'

        # ------------------- receive msg start -----------
        while self.connected:
            print('loop start')
            time.sleep(0.5)
            try:
                print('waiting')
                self.clientSocket.settimeout(10.0)
                self.recv_msg = self.clientSocket.recv(1024).decode('utf-8')
                # try:
                #    self.recv_msg = self.clientSocket.recv(1024).decode('utf-8')
                # except Exception as e:
                #    self.pprint('MainException: {}'.format(e))
                print('\n' + self.recv_msg)
                if self.recv_msg == '':
                    print('here')
                    # continue
                    # pass
                    # break
                    raise Exception('empty msg')
                self.recv_msg = self.recv_msg.split('/')

                if self.recv_msg[0] == 'app_ping':
                    # print('app_ping received')
                    send_msg = 'robot_ping'
                    now_temp = arm.temperatures
                    now_cur = arm.currents
                    send_msg = [
                        {
                            'type': 'A', 'joint_name': 'Base', 'temperature': now_temp[0],
                            'current': round(now_cur[0], 3) * 100
                        }, {
                            'type': 'B', 'joint_name': 'Shoulder', 'temperature': now_temp[1],
                            'current': round(now_cur[1], 3) * 100
                        }, {
                            'type': 'C', 'joint_name': 'Elbow', 'temperature': now_temp[2],
                            'current': round(now_cur[2], 3) * 100
                        }, {
                            'type': 'D', 'joint_name': 'Wrist1', 'temperature': now_temp[3],
                            'current': round(now_cur[3], 3) * 100
                        }, {
                            'type': 'E', 'joint_name': 'Wrist2', 'temperature': now_temp[4],
                            'current': round(now_cur[4], 3) * 100
                        }, {
                            'type': 'F', 'joint_name': 'Wrist3', 'temperature': now_temp[5],
                            'current': round(now_cur[5], 3) * 100
                        }
                    ]
                    try:
                        time.sleep(0.5)
                        self.clientSocket.send(f'{send_msg}'.encode('utf-8'))
                        print('robot_ping')

                    except Exception as e:
                        self.pprint('MainException: {}'.format(e))
                        print('ping send fail')
                    # send_msg = arm.temperatures
                    if self.state == 'ready':
                        print('STATE : ready for new msg')
                    else:
                        print('STATE : now moving')
                else:
                    self.recv_msg[0] = self.recv_msg[0].replace("app_ping", "")
                    if self.recv_msg[0] in ['breath', 'greet', 'farewell' 'dance_random', 'dance_a', 'dance_b',
                                            'dance_c',
                                            'sleep', 'comeon']:
                        print(f'got message : {self.recv_msg[0]}')
                        if self.state == 'ready':
                            self.state = self.recv_msg[0]
                    elif self.recv_msg[0] == 'robot_script_stop':
                        code = self._arm.set_state(4)
                        if not self._check_code(code, 'set_state'):
                            return
                        sys.exit()
                        self.is_alive = False
                        print('program exit')

                    # 픽업존 아이스크림 뺐는지 여부 확인
                    elif self.recv_msg[0].find('icecream_go') >= 0 or self.recv_msg[0].find(
                            'icecream_stop') >= 0 and self.state == 'icecreaming':
                        print(self.recv_msg[0])
                        if self.recv_msg[0].find('icecream_go') >= 0:
                            self.order_msg['makeReq']['latency'] = 'go'
                        else:
                            self.order_msg['makeReq']['latency'] = 'stop'
                            print('000000000000000000000000000000')

                    # 실링 존재 여부 확인

                    if self.recv_msg[0].find('sealing_pass') >= 0 and self.state == 'icecreaming':
                        self.order_msg['makeReq']['sealing'] = 'go'
                        print('socket_go')
                    elif self.recv_msg[0].find('sealing_reject') >= 0 and self.state == 'icecreaming':
                        self.order_msg['makeReq']['sealing'] = 'stop'
                        print('socket_stop')

                    else:
                        # print('else')
                        try:
                            self.order_msg = json.loads(self.recv_msg[0])
                            if self.order_msg['type'] == 'ICECREAM':
                                if self.state == 'ready':
                                    print('STATE : icecreaming')
                                    print(f'Order message : {self.order_msg}')
                                    self.state = 'icecreaming'
                            # else:
                            #    self.clientSocket.send('ERROR : already moving'.encode('utf-8'))
                            else:
                                self.clientSocket.send('ERROR : wrong msg received'.encode('utf-8'))
                        except:
                            pass
                self.recv_msg[0] = 'zzz'

            except Exception as e:
                self.pprint('MainException: {}'.format(e))
                # if e == 'empty msg' :
                #    pass
                # self.connected = False
                print('connection lost')
                while True:
                    time.sleep(2)
                    try:

                        try:
                            self.serverSocket.shutdown(socket.SHUT_RDWR)
                            self.serverSocket.close()
                        except:
                            pass

                        print('socket_making')
                        self.serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        self.serverSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

                        self.serverSocket.bind(self.ADDR)
                        print("bind")

                        while True:
                            print('listening')
                            self.serverSocket.listen(1)
                            print(f'reconnecting')
                            try:
                                self.clientSocket, addr_info = self.serverSocket.accept()
                                break

                            except socket.timeout:
                                print('socket.timeout')
                                break

                            except:
                                pass
                        break
                    except Exception as e:
                        self.pprint('MainException: {}'.format(e))
                        print('except')
                        # pass


    # ===============================  tunning =============================== #
    def process_input(self, thetas):
        thetas_deg = np.degrees(thetas)
        self.solutions.append(thetas_deg)
        # print(thetas_deg)
        # thetas_deg = self.send_coordinates_to_notebook2()
        code = self._arm.set_servo_angle(angle=thetas_deg, speed=20, mvacc=500, wait=True, radius=10.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
    def adsd(self):
        prev_x, prev_y = None, None
        canvas = None
        last_time = time.time()
        initial_hand_position = None
        tracking_margin = 100
        with self.mp_hands.Hands(max_num_hands=1) as hands:
            while self.cap.isOpened():
                ret, frame = self.cap.read()
                if not ret:
                    break
                frame = cv2.flip(frame, 1)
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                result = hands.process(frame_rgb)
                if canvas is None:
                    canvas = frame.copy() * 0
                current_time = time.time()
                if result.multi_hand_landmarks is None:
                    print("No hands detected")
                if result.multi_hand_landmarks:
                    for hand_landmarks in result.multi_hand_landmarks:
                        x = int(hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST].x * frame.shape[1])
                        y = int(hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST].y * frame.shape[0])
                        if initial_hand_position is None:
                            initial_hand_position = (x, y)
                        if initial_hand_position is not None:
                            if prev_x is None or prev_y is None:
                                prev_x, prev_y = x, y
                            if abs(x - prev_x) < tracking_margin and abs(y - prev_y) < tracking_margin:
                                if canvas is None:
                                    canvas = frame.copy()
                                if current_time - last_time >= 3:
                                    self.coordinates.append((x, y))
                                    print(f'Coordinates: ({x}, {y})')
                                    last_time = current_time
                                prev_x, prev_y = x, y
                                self.mp_drawing.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                frame = cv2.addWeighted(frame, 0.7, canvas, 0.7, 0)
                cv2.imshow('Hand Tracking', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.cleanup_timer.cancel()
                    break
                if current_time - self.last_processed_time > self.process_interval:
                    self.process_latest_coordinates()
                    self.last_processed_time = current_time
        self.cap.release()
        cv2.destroyAllWindows()
        
    # def process_latest_coordinates(self):
    #     if self.coordinates:
    #         pixel_x, pixel_y = self.coordinates[-1]
    #         # [x,y]
    #         self.send_coordinates_to_notebook2(pixel_x, pixel_y)
    
    
    def process_latest_coordinates(self):
        if self.coordinates:
            pixel_x, pixel_y = self.coordinates[-1]
            threading.Thread(target=self.send_coordinates_to_notebook2, args=(pixel_x, pixel_y)).start()
            
    def send_coordinates_to_notebook2(self, x, y):
        server_address = ('172.30.1.82', 10000)
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.connect(server_address)
            message = f'{x},{y}'
            print(f"message : {message} ")
            sock.sendall(message.encode('utf-8'))
            data = sock.recv(1024)
            print(f"data : {data} ")
            decoded_data = data.decode('utf-8')
            thetas = list(map(float, decoded_data.split(','))) 
            # thetas = np.fromstring(data.decode('utf-8'), sep=',')
            print(f"thetas : {thetas} ")
            self.process_input(thetas)
            # return thetas_deg_list
            
    # def send_coordinates_thread(self):
    #     while True:
    #         x, y = self.coordinates.get()
    #         self.send_coordinates_to_notebook2(x, y)
    #         self.coordinates.task_done()

            
            
    def start_cleanup_timer(self):
        self.cleanup_timer = Timer(self.cleanup_interval, self.cleanup_lists)
        self.cleanup_timer.start()
        print("start_cleanup_timer")
        
    def cleanup_lists(self):
        if len(self.coordinates) > 0:
            del self.coordinates[:len(self.coordinates) // 2]
        if len(self.solutions) > 0:
            del self.solutions[:len(self.solutions) // 2]
        self.start_cleanup_timer()
        print("cleanup_lists")
        
        
    def run_robot_arm_tracking(self):
        self.adsd()  # 손 추적 시작
        

    def tracking_home(self):
        code = self._arm.set_servo_angle(angle=[269.4, -34, 18.9, 182.6, 38.1, 0.8], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=10.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

if __name__ == '__main__':
    RobotMain.pprint('xArm-Python-SDK Version:{}'.format(version.__version__))
    arm = XArmAPI('192.168.1.184', baud_checkset=False)
    robot_main = RobotMain(arm)
    print("start")
    # robot_main.tracking_home()
    robot_main.run_robot_arm_tracking()
    print("finish")
    
    
    
    
