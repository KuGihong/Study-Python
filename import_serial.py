import serial
import time

# 아두이노와의 시리얼 통신 설정
ser = serial.Serial('/dev/ttyACM0', 9600)  # 포트와 속도를 아두이노에 맞게 설정 (COM1은 예시)

try:
    message = "mode1"  # 전달할 메시지(명령)
    ser.write(message.encode('utf-8'))  # 아두이노로 메시지 전송
    print("Send message: " + message)
    time.sleep(1)  # 1초 동안 대기 (원하는 대기 시간으로 수정)

    # 아두이노로부터 메시지 읽기
    received_data = ser.readline()
    print("Received message: " + received_data.decode('utf-8'))

except serial.SerialException:
    print("error")

finally:
    ser.close()
