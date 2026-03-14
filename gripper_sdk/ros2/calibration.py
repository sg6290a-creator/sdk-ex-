#!/usr/bin/env python3
"""
그리퍼 캘리브레이션 유틸리티

수행 항목:
  1. 각 모터의 open/close 한계 위치 탐색 (전류 기반 stall 감지)
  2. 결과를 JSON으로 저장/로드
"""
import json
import time
import os
from typing import Dict, Tuple
from gripper_sdk.core.motor_group import MotorGroup


CALIB_FILE = os.path.join(os.path.dirname(__file__), '../config/calibration.json')

# 캘리브레이션 파라미터
CALIB_CURRENT      = 150   # 탐색 전류 [mA] - 낮게 시작
STALL_CURRENT_THR  = 250   # stall 판단 전류 임계값 [mA]
STALL_DURATION     = 0.3   # stall 유지 시간 [s]
MOVE_TIMEOUT       = 5.0   # 방향당 최대 탐색 시간 [s]
SETTLE_TIME        = 0.5   # 방향 전환 전 대기 [s]


def find_limit(group: MotorGroup, motor_id: int,
               direction: int) -> Tuple[int, bool]:
    """
    direction: +1 (open 방향), -1 (close 방향)
    stall 감지 시 현재 위치 반환.
    반환: (position_tick, stall_detected)
    """
    motor = group.gripper.motors[motor_id]
    stall_start = None
    deadline = time.time() + MOVE_TIMEOUT

    while time.time() < deadline:
        cur = motor.getPresentCurrent()
        pos = motor.getPresentPosition()

        if abs(cur) >= STALL_CURRENT_THR:
            if stall_start is None:
                stall_start = time.time()
            elif time.time() - stall_start >= STALL_DURATION:
                return pos, True
        else:
            stall_start = None

        # 전류 명령으로 천천히 이동
        group.write_currents({motor_id: direction * CALIB_CURRENT})
        time.sleep(0.02)

    pos = motor.getPresentPosition()
    return pos, False


def calibrate(group: MotorGroup,
              motor_ids=None) -> Dict[int, Dict]:
    """
    각 모터의 open/close 한계 위치 탐색.
    반환: {motor_id: {'open': tick, 'close': tick, 'center': tick}}
    """
    if motor_ids is None:
        motor_ids = list(group.motors.keys())

    results = {}

    for mid in motor_ids:
        print(f"\n[ID {mid}] 캘리브레이션 시작")
        motor = group.gripper.motors[mid]

        # open 방향 (+)
        print(f"  → open 방향 탐색 중...")
        open_pos, ok = find_limit(group, mid, direction=+1)
        print(f"  open 위치: {open_pos} {'(stall)' if ok else '(timeout)'}")
        group.write_currents({mid: 0})
        time.sleep(SETTLE_TIME)

        # close 방향 (-)
        print(f"  → close 방향 탐색 중...")
        close_pos, ok = find_limit(group, mid, direction=-1)
        print(f"  close 위치: {close_pos} {'(stall)' if ok else '(timeout)'}")
        group.write_currents({mid: 0})
        time.sleep(SETTLE_TIME)

        center = (open_pos + close_pos) // 2
        results[mid] = {'open': open_pos, 'close': close_pos, 'center': center}
        print(f"  center: {center}")

    return results


def save_calibration(data: Dict, path: str = CALIB_FILE):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    # key를 str로 변환해서 저장
    with open(path, 'w') as f:
        json.dump({str(k): v for k, v in data.items()}, f, indent=2)
    print(f"\n캘리브레이션 저장 완료: {path}")


def load_calibration(path: str = CALIB_FILE) -> Dict[int, Dict]:
    if not os.path.exists(path):
        raise FileNotFoundError(f"캘리브레이션 파일 없음: {path}")
    with open(path) as f:
        raw = json.load(f)
    return {int(k): v for k, v in raw.items()}
