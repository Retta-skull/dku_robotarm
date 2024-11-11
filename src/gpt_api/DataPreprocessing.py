#!/usr/bin/env python3
import json
import re
from dataclasses import dataclass
from typing import List, Tuple, Optional

@dataclass
class Block:
    name: str
    coordinates: Tuple[int, int, int]

@dataclass
class ActionData:
    action_type: str
    parameters: Optional[List[int]] = None

@dataclass
class ParsedResponse:
    order: str
    reply: str
    planned_actions: int
    actions: List[ActionData]
    status: str
    data: List[Block]

class ResponseParser:
    def __init__(self, json_response: str):
        # 주석을 제거한 JSON 문자열 생성
        self.json_response = self.remove_comments(json_response)
        self.parsed_response = self.parse_response()

    def remove_comments(self, json_string: str) -> str:
        # 주석(# 또는 //)으로 시작하는 부분 제거
        json_string = re.sub(r'#.*', '', json_string)  # # 주석 제거
        json_string = re.sub(r'//.*', '', json_string) # // 주석 제거
        return json_string

    def parse_response(self) -> ParsedResponse:
        try:
            data = json.loads(self.json_response)
            
            order = data.get("Order", "")
            reply = data.get("Reply", "")
            planned_actions = data.get("Planned_Actions", 0)
            actions = data.get("Actions", [])
            status = data.get("Status", "")
            data_blocks = data.get("Data", [])

            # ActionData 객체 리스트 생성
            action_objects = []
            for action in actions:
                action_type = action.get("action_type", "")
                parameters = action.get("parameters")
                action_objects.append(ActionData(action_type=action_type, parameters=parameters))

            # Block 객체 리스트 생성
            blocks = []
            for block in data_blocks:
                name = block.get("name", "")
                coordinates = tuple(block.get("coordinates", [0, 0, 0]))
                blocks.append(Block(name=name, coordinates=coordinates))

            parsed = ParsedResponse(
                order=order,
                reply=reply,
                planned_actions=planned_actions,
                actions=action_objects,
                status=status,
                data=blocks
            )
            print("성공적으로 파싱이 진행되었습니다.")
            return parsed
        
        except json.JSONDecodeError as e:
            print(f"JSON 디코딩 오류: {e}")
            # 오류 발생 시 기본값을 가진 ParsedResponse 반환
            print(self.json_response)
            return ParsedResponse("", "", 0, [], "오류 - JSON 디코딩 실패", [])
        except Exception as e:
            print(f"파싱 중 오류 발생: {e}")
            return ParsedResponse("", "", 0, [], f"오류 - {e}", [])

    def get_parsed_response(self) -> ParsedResponse:
        return self.parsed_response

    def display_parsed_response(self):
        parsed = self.parsed_response
        print(f"Order: {parsed.order}")
        print(f"Reply: {parsed.reply}")
        print(f"Planned Actions: {parsed.planned_actions}")
        print(f"Actions:")
        for action in parsed.actions:
            if action.parameters:
                print(f"  - {action.action_type}({', '.join(map(str, action.parameters))})")
            else:
                print(f"  - {action.action_type}")
        print(f"Status: {parsed.status}")
        print("Data:")
        for block in parsed.data:
            print(f"  - Name: {block.name}, Coordinates: {block.coordinates}")