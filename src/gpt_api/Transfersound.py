from openai import OpenAI
import soundfile as sf
import pyaudio
import numpy as np
import io
import wave
import tempfile
import time

class TextToSpeech:
    def __init__(self, api_key, model="tts-1", voice="nova"):
        self.client = OpenAI(api_key=api_key)
        self.model = model
        self.voice = voice
        self.p = pyaudio.PyAudio()
        
    def generate_speech(self, text):
        # 음성 생성 요청
        response = self.client.audio.speech.create(
            model=self.model,
            voice=self.voice,
            input=text
        )
        
        # 바이너리 데이터를 numpy 배열로 변환
        data, samplerate = sf.read(io.BytesIO(response.content), dtype='int16')
        return data, samplerate
    
    def play_speech(self, data, samplerate):
        try:
            output_device_index = self.get_default_output_device()
        except OSError:
            output_device_index = None
        
        # 스트림 설정
        stream = self.p.open(format=pyaudio.paInt16,
                             channels=1,  # 모노 오디오
                             rate=samplerate,
                             output=True,
                             output_device_index=output_device_index)
        
        # 데이터 스트리밍
        stream.write(data.tobytes())
        
        # 스트림 종료
        stream.stop_stream()
        stream.close()

    def get_default_output_device(self, preferred_name="pulse"):
        count = self.p.get_device_count()
        for i in range(count):
            device_info = self.p.get_device_info_by_index(i)
            # print(f'device_info : {device_info}')
            if device_info["maxOutputChannels"] > 0 and preferred_name in device_info["name"]:
                return i
        raise OSError(f"No input device found with name containing '{preferred_name}'")


    def speak(self, text):
        data, samplerate = self.generate_speech(text)
        self.play_speech(data, samplerate)
    
    def __del__(self):
        # PyAudio 종료
        self.p.terminate()

class SpeechProcessor:
    def __init__(self, api_key, rate=44100, channels=1, format=pyaudio.paInt16, chunk=8192, threshold=1500, silence_limit=3):
        self.client = OpenAI(api_key=api_key)
        self.rate = rate
        self.channels = channels
        self.format = format
        self.chunk = chunk
        self.threshold = threshold
        self.silence_limit = silence_limit
        self.p = pyaudio.PyAudio()
        self.input_device_index = self.get_default_input_device()

    def record_audio(self):
        stream = self.p.open(format=self.format, channels = self.channels,
                             rate=self.rate, input = True,
                             frames_per_buffer = self.chunk,
                             input_device_index = self.input_device_index)

        print("Listening for sound...")
        frames = []
        recording = False
        silence_start = None

        while True:
            data = stream.read(self.chunk)
            # 음성 강도 계산
            amplitude = np.frombuffer(data, dtype=np.int16)
            volume = np.abs(amplitude).mean()
            
            if not recording:
                print(volume)
                time.sleep(0.1)  # 볼륨 출력 간 딜레이 추가

            if volume > self.threshold:
                if not recording:
                    print("Sound detected, starting recording...")
                    recording = True
                frames.append(data)
                silence_start = None
            elif recording:
                if silence_start is None:
                    silence_start = time.time()
                else:
                    elapsed_time = time.time() - silence_start
                    if elapsed_time > self.silence_limit:
                        print("Silence detected, stopping recording...")
                        break
                frames.append(data)

        print("Recording stopped...")

        # 스트림 종료
        stream.stop_stream()
        stream.close()

        # 임시 파일 생성
        temp_file = tempfile.NamedTemporaryFile(delete=False, suffix='.wav')

        # WAV 파일로 저장
        with wave.open(temp_file.name, 'wb') as wf:
            wf.setnchannels(self.channels)
            wf.setsampwidth(self.p.get_sample_size(self.format))
            wf.setframerate(self.rate)
            wf.writeframes(b''.join(frames))

        return temp_file

    def transcribe_audio(self, temp_file):
        # OpenAI에 파일 전송
        with open(temp_file.name, 'rb') as audio_file:
            transcription = self.client.audio.transcriptions.create(
                model="whisper-1", 
                file=audio_file, 
                response_format="text"
            )
        return transcription
    
    def get_default_input_device(self, preferred_name="pulse"):
        count = self.p.get_device_count()
        for i in range(count):
            device_info = self.p.get_device_info_by_index(i)
            # print(f'device_info : {device_info}')
            if device_info["maxInputChannels"] > 0 and preferred_name in device_info["name"]:
                return i
        raise OSError(f"No input device found with name containing '{preferred_name}'")



    def __del__(self):
        # PyAudio 종료
        self.p.terminate()