#!/usr/bin/env python3
import json
import os
import openai
import time

class ChatThreadHandler:
    def __init__(self, assistant_id, api_key='OPENAI_API_KEY', logger=None):
        self.api_key = api_key or os.getenv('OPENAI_API_KEY')
        if not self.api_key:
            raise ValueError("API 키는 인자로 제공되거나 OPENAI_API_KEY 환경 변수에 설정되어야 합니다.")
        
        openai.api_key = self.api_key
        self.assistant_id = assistant_id
        self.thread_id = None
        self.run = None
        self.logger = logger
        self.create_new_thread()
        
    def create_new_thread(self):
        thread = openai.beta.threads.create()
        self.thread_id = thread.id
        self.logger.info(f"새로운 스레드가 생성되었습니다. 스레드 ID: {self.thread_id}")
        return thread

    def submit_message(self, user_message):
        if not self.thread_id:
            raise RuntimeError("스레드 ID가 설정되지 않았습니다. 먼저 create_new_thread()를 호출하세요.")
        
        openai.beta.threads.messages.create(
            thread_id=self.thread_id,
            role='user',
            content=user_message
        )
        self.logger.info(f"스레드 {self.thread_id}에 메시지를 제출했습니다: \n{user_message}")
        
        self.run = openai.beta.threads.runs.create(
            thread_id=self.thread_id,
            assistant_id=self.assistant_id
        )
        self.logger.info(f"실행이 시작되었습니다. 실행 ID: {self.run.id}")
        return self.run

    def wait_on_run(self, poll_interval=0.5):
        if not self.run:
            raise RuntimeError("실행이 시작되지 않았습니다. 먼저 submit_message()를 호출하세요.")
        
        self.logger.info("실행이 완료될 때까지 기다리는 중...")
        while self.run.status in ["queued", "in_progress"]:
            self.logger.info(f"현재 실행 상태: {self.run.status}")
            time.sleep(poll_interval)
            self.run = openai.beta.threads.runs.retrieve(
                thread_id=self.thread_id,
                run_id=self.run.id
            )
        self.logger.info(f"실행이 완료되었습니다. 상태: {self.run.status}")
        return self.run

    def get_response(self):
        if not self.thread_id:
            raise RuntimeError("스레드 ID가 설정되지 않았습니다. 먼저 create_new_thread()를 호출하세요.")
        
        messages = openai.beta.threads.messages.list(thread_id=self.thread_id, order="asc")
        if len(messages.data) < 2:
            raise ValueError("어시스턴트로부터의 응답이 아직 없습니다.")
        
        response = messages.data[-1]  # 마지막 메시지가 어시스턴트의 응답이라고 가정
        self.logger.info("응답을 받았습니다")
        return response

    def print_message(self, response):
        self.logger.info(f"[응답]\n{response.content[0].text.value}\n")

    def show_json(self, obj):
        self.logger.info(json.dumps(obj.model_dump(), ensure_ascii=False, indent=2))

    def run_chat(self, user_message):
        self.submit_message(user_message)
        self.wait_on_run()
        response = self.get_response()
        #self.print_message(response)
        return response.content[0].text.value

