#!/usr/bin/env python3
import sys
sys.path.append("../gpt_api")
import json
import threading
import audioop
from config import ASSISTANT_ID, API_KEY
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from rcl_interfaces.msg import Log
import speech_recognition as sr
from gpt_api.ChatHandler import ChatThreadHandler
from gpt_api.DataPreprocessing import ResponseParser
from gpt_api.RobotAction import ActionExecutor, Robot
from gpt_api.Transfersound import TextToSpeech
from cv_bridge import CvBridge
import time
import logging

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller_gui')
        self.detection_data = ""  # 최신 탐지 데이터를 저장
        self.detection_image = None
        self.transcription = ""
        self.response = ""
        self.current_position = [0, 18, 20]
        self.bridge = CvBridge()
        # ROS2 Subscriber 설정
        self.create_subscription(String, '/detection_data', self.handle_detection_data, 10)
        self.create_subscription(Image, '/detection_image', self.image_detection_data, 10)
        self.create_subscription(Log, '/rosout', self.handle_rosout_logs, 10)  # 전체 로그 구독

        # GPT 및 음성 처리 초기화
        self.chat_handler = ChatThreadHandler(assistant_id=ASSISTANT_ID, api_key=API_KEY, logger=self.get_logger())
        self.tts = TextToSpeech(API_KEY)
        self.robot = Robot(self)

        # Speech Recognition 초기화
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone(sample_rate=48000)
        self.is_recording = False

        # 음성 감지 및 처리 시작
        threading.Thread(target=self.detect_and_record_audio, daemon=True).start()

        # 로그 저장 변수 초기화
        self.logger_output = ""

        # 로거 설정
        self.logger = logging.getLogger('robot_controller_gui_logger')
        self.logger.setLevel(logging.INFO)
        log_handler = logging.StreamHandler()
        log_handler.setFormatter(logging.Formatter('[%(levelname)s] %(message)s'))
        self.logger.addHandler(log_handler)

    def handle_detection_data(self, msg):
        try:
            self.detection_data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().info("Invalid JSON received in detection_data")

    def image_detection_data(self, msg):
        self.detection_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def handle_rosout_logs(self, msg):
        log_message = f"[INFO] {msg.name}: {msg.msg}"
        self.log_to_gui(log_message)

    def process_data(self):
        detection_text = ""
        if isinstance(self.detection_data, list):
            for block in self.detection_data:
                if isinstance(block, dict):
                    name = block.get("label", "unknown")
                    coordinates = [block.get("x", 0), block.get("y", 0), block.get("z", 0)]
                    detection_text += f"\n- Name: {name}\n  Coordinates: {coordinates}\n"
        return detection_text

    def update_transcription(self, text):
        self.transcription = text

    def update_response(self, response):
        self.response = response

    def execute_gpt_response(self, response):
        try:
            parser = ResponseParser(response)
            parsed = parser.get_parsed_response()

            # Text-to-Speech
            self.tts.speak(parsed.reply)

            # Action Execution
            if parsed and hasattr(parsed, 'actions'):
                executor = ActionExecutor(parsed.actions, self.robot)
                executor.execute_all()
                self.current_position = executor.UpdatePosition()
        except Exception as e:
            self.get_logger().info(f"Error in processing GPT response: {str(e)}")

    def detect_and_record_audio(self):
        with self.microphone as source:
            # self.recognizer.adjust_for_ambient_noise(source)  # 환경 소음 보정
            self.recognizer.energy_threshold = 50  # 기본값: 약 300 (필요에 따라 조정)
            self.get_logger().info(f"Energy threshold set to: {self.recognizer.energy_threshold}")

            self.get_logger().info("Ready to detect sound...")
        
            silence_threshold = 2  # 침묵 기준 시간 (초)
            silence_start = None  # 침묵 시작 시간
            audio_buffer = []  # 녹음 데이터를 저장할 버퍼

            while True:
                try:
                    # 소리 스트림을 열기
                    audio_stream = self.microphone.stream
                    buffer = audio_stream.read(self.microphone.CHUNK)
                    
                    # 오디오 볼륨 분석
                    energy = audioop.rms(buffer, self.microphone.SAMPLE_WIDTH)  # RMS 에너지 계산
                    
                    if energy > self.recognizer.energy_threshold:
                        # 소리가 감지되면 녹음
                        if self.is_recording == False:
                            self.get_logger().info(f"Sound detected. Recording...")
                            self.is_recording = True

                        silence_start = None  # 침묵 타이머 초기화
                        audio_buffer.append(buffer)
                    else:
                        # 침묵 상태 감지
                        current_time = time.time()
                        if silence_start is None:
                            silence_start = current_time
                        elif current_time - silence_start >= silence_threshold and audio_buffer:
                            # 침묵 지속 시간이 기준을 초과하면 녹음 종료
                            self.get_logger().info("Silence detected. Processing audio...")
                            self.process_audio_combined(audio_buffer)
                            self.is_recording = False
                            audio_buffer = []  # 버퍼 초기화
                            silence_start = None

                except Exception as e:
                    self.get_logger().info(f"Error in audio detection: {e}")

    def process_audio_combined(self, audio_buffer):
        try:
            # 여러 오디오 조각을 결합
            combined_audio = sr.AudioData(b"".join(audio_buffer), 
                                        self.microphone.SAMPLE_RATE, 
                                        self.microphone.SAMPLE_WIDTH)
            text = self.recognizer.recognize_google(combined_audio, language="ko-KR").lower()
            self.log_to_gui(f"Recognized text: {text}")
            self.update_transcription(text)

            # GPT와 통신
            message = f"Order: {text} \nCurrentPosition: {self.current_position} \nData: {self.process_data()}"
            response = self.chat_handler.run_chat(message)
            self.update_response(response)
            self.execute_gpt_response(response)

        except sr.UnknownValueError:
            self.log_to_gui("Could not understand audio.")
        except sr.RequestError as e:
            self.log_to_gui(f"Speech recognition service error: {e}")

    def log_to_gui(self, message):
        self.logger_output += message + "\n"
        self.logger.info(message)  # 로거에도 출력

    def get_logger_output(self):
        output = self.logger_output
        self.logger_output = ""  # 기존 로그는 비우기
        return output
