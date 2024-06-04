#!/usr/bin/env python3
#-- coding:utf-8 --
####################################################################
# 프로그램 명 : main.py
# 작  성  자 : (주)자이트론
# 본 프로그램은 상업라이센스에 의해 제공되므로 무단배포 및 상업적 이용을 금합니다.
####################################################################

import simulator
import signal
import sys
import os

# 시그널 처리 함수
def signal_handler(sig, frame):
    """
    SIGINT (Ctrl+C) 신호가 감지되었을 때 호출되는 함수

    - 종료할 모든 프로세스를 종료
    - 현재 Python 스크립트를 종료 (성공 종료 코드 0).
    """
    os.system('killall -9 roslaunch roscore python')
    sys.exit(0) 

# 시그널 처리를 위한 신호 처리 함수
signal.signal(signal.SIGINT, signal_handler)

# simulator module에서 main 함수 실행
simulator.main()
