#!/usr/bin/env python3
"""
키보드 텔레오퍼레이션 유틸리티

키 조작:
  1~4       ID 1/2/3/4 선택/해제 (토글)
  a         전체 모터 선택
  q         선택 모터 전류 증가
  e         선택 모터 전류 감소
  r         전류 0 리셋
  ESC       종료
"""
import tty
import termios
import select
import time
from typing import Dict, Set

from gripper_sdk.core.motor_group import MotorGroup
from gripper_sdk.config.params import MOTOR_IDS, CURRENT_LIMIT

STEP    = 20
LOOP_HZ = 50


def _print_status(currents: Dict[int, int], selected: Set[int],
                  actual: Dict[int, int]):
    parts = []
    for mid in MOTOR_IDS:
        sel = '*' if mid in selected else ' '
        parts.append(f"[{sel}ID{mid} cmd:{currents[mid]:+4d} act:{actual.get(mid, 0):+4d}mA]")
    sys.stdout.write('\r' + '  '.join(parts) + '\033[K')
    sys.stdout.flush()


def run(group: MotorGroup,
        step: int = STEP,
        loop_hz: int = LOOP_HZ,
        current_limit: int = CURRENT_LIMIT):

    currents: Dict[int, int] = {mid: 0 for mid in MOTOR_IDS}
    selected: Set[int] = set(MOTOR_IDS)
    dt = 1.0 / loop_hz

    print("=== 키보드 텔레오퍼레이션 (Current Mode) ===")
    print("1~9,0,-: 모터 ID1~11 선택  q: 전류↑  e: 전류↓  a: 전체  s: 리셋  ESC: 종료\n")

    # 키 → 모터 ID 매핑
    key_to_id = {str(i): i for i in range(1, 10)}
    key_to_id.update({'0': 10, '-': 11})

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        # raw 모드를 루프 전에 한 번만 설정
        tty.setraw(fd)

        while True:
            t0 = time.time()

            # 논블로킹 키 읽기
            rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
            key = os.read(fd, 1).decode('utf-8', errors='ignore') if rlist else None

            if key is not None:
                if key == '\x1b':
                    break
                elif key == 'q':
                    for mid in selected:
                        currents[mid] = min(currents[mid] + step, current_limit)
                elif key == 'e':
                    for mid in selected:
                        currents[mid] = max(currents[mid] - step, -current_limit)
                elif key == 's':
                    currents = {mid: 0 for mid in MOTOR_IDS}
                elif key == 'a':
                    selected = set(MOTOR_IDS)
                elif key in key_to_id:
                    mid = key_to_id[key]
                    if mid in MOTOR_IDS:
                        selected.discard(mid) if mid in selected else selected.add(mid)

            group.write_currents(currents)
            actual = group.read_currents()
            _print_status(currents, selected, actual)

            elapsed = time.time() - t0
            if dt - elapsed > 0:
                time.sleep(dt - elapsed)

    finally:
        # 터미널 복원 및 전류 0
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        sys.stdout.write('\n종료.\n')
        group.write_currents({mid: 0 for mid in MOTOR_IDS})


if __name__ == '__main__':
    from gripper_sdk.core.gripper import OperatingMode
    from gripper_sdk.core.gripper import Gripper

    with Gripper() as gripper:
        gripper.disable()                          # 혹시 켜져있으면 끄고
        gripper.set_mode(OperatingMode.CURRENT)
        gripper.enable()
        run(MotorGroup(gripper))
