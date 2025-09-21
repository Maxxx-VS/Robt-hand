import cv2
import numpy as np
import serial
import time
import math
import threading

SERIAL_PORT = '/dev/ttyUSB0'
BAUDRATE = 115200
SER_TIMEOUT = 0.1

IDX_PLATFORM = 0
IDX_LOWER = 1
IDX_MIDDLE = 2
IDX_UPPER = 3
IDX_GRIP = 4

ANGLE_MIN = 5
ANGLE_MAX = 175

servo_targets = {
    IDX_PLATFORM: 90,
    IDX_LOWER: 90,
    IDX_MIDDLE: 90,
    IDX_UPPER: 90,
    IDX_GRIP: 110
}


def clamp_angle(a):
    a = int(round(a))
    return max(ANGLE_MIN, min(ANGLE_MAX, a))


LOWER_GREEN = np.array([40, 70, 50])
UPPER_GREEN = np.array([85, 255, 255])

K_px = 0.06
K_py = 0.06
K_dist = 0.0005

CENTER_THRESH_PIX = 15
CENTER_RADIUS = 40
AREA_NEAR = 20000
AREA_FAR = 2000

MIN_AREA_FOR_TRACKING = 500  # минимальная площадь для начала отслеживания


def send_servo_set(ser, idx, angle):
    angle = clamp_angle(angle)
    cmd = f"S{idx}:{angle}\n"
    ser.write(cmd.encode('ascii'))
    time.sleep(0.01)
    return cmd


def wait_for_response(ser, keyword="DONE", timeout=5.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        line = ser.readline().decode('ascii', errors='ignore').strip()
        if line:
            print("[ARDUINO]", line)
            if keyword in line:
                return True
        time.sleep(0.05)
    return False


def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=SER_TIMEOUT)
    except Exception as e:
        print("Не удалось открыть сериал-порт:", e)
        return

    print("Ожидаем READY от Arduino...")
    if not wait_for_response(ser, "READY", timeout=6.0):
        print("Arduino не ответил READY. Продолжаем.")

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Не удалось открыть камеру.")
        ser.close()
        return

    time.sleep(0.5)
    tracking = False
    object_locked = False  # флаг, указывающий, что объект захвачен (центр в круге)

    while True:
        if not tracking:
            command = input("Введите команду (1-инициализация, 2-поиск, 3-стоп, q-выход): ")
            if command == '1':
                ser.write(b"1\n")
                wait_for_response(ser, "DONE")
            elif command == '2':
                ser.write(b"2\n")
                if wait_for_response(ser, "DONE"):
                    tracking = True
                    object_locked = False
                    print("Режим поиска активирован")
            elif command == '3':
                ser.write(b"3\n")
                wait_for_response(ser, "DONE")
            elif command == 'q':
                break
            continue

        ret, frame = cap.read()
        if not ret:
            time.sleep(0.1)
            continue

        h, w = frame.shape[:2]
        cx_img, cy_img = w // 2, h // 2

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        object_center = None
        object_area = 0

        if contours:
            largest = max(contours, key=cv2.contourArea)
            object_area = cv2.contourArea(largest)
            if object_area > 100:  # минимальная площадь для отображения
                M = cv2.moments(largest)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    object_center = (cx, cy)
                    cv2.drawContours(frame, [largest], -1, (0, 255, 0), 2)
                    cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)
                    cv2.putText(frame, f"Area:{int(object_area)}", (cx + 10, cy + 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        cv2.line(frame, (cx_img, 0), (cx_img, h), (255, 255, 255), 1)
        cv2.line(frame, (0, cy_img), (w, cy_img), (255, 255, 255), 1)
        cv2.circle(frame, (cx_img, cy_img), 5, (255, 0, 0), -1)
        cv2.circle(frame, (cx_img, cy_img), CENTER_RADIUS, (0, 255, 255), 2)

        quad_text = "No object"
        if object_center is not None and object_area > 100:
            ox, oy = object_center

            # Определяем квадрант
            if ox < cx_img and oy < cy_img:
                quad_text = "Top-Left"
            elif ox >= cx_img and oy < cy_img:
                quad_text = "Top-Right"
            elif ox < cx_img and oy >= cy_img:
                quad_text = "Bottom-Left"
            else:
                quad_text = "Bottom-Right"

            cv2.putText(frame, quad_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

            # Проверяем, находится ли объект в центральном круге
            dx, dy = ox - cx_img, oy - cy_img
            dist_to_center = math.hypot(dx, dy)

            # Если объект достаточно большой для отслеживания
            if object_area > MIN_AREA_FOR_TRACKING:
                # Если объект не в центре и не заблокирован
                if dist_to_center > CENTER_RADIUS and not object_locked:
                    # Автонаведение
                    if abs(dx) > CENTER_THRESH_PIX:
                        delta_platform = -K_px * dx
                        servo_targets[IDX_PLATFORM] = clamp_angle(servo_targets[IDX_PLATFORM] + delta_platform)
                        send_servo_set(ser, IDX_PLATFORM, servo_targets[IDX_PLATFORM])

                    if abs(dy) > CENTER_THRESH_PIX:
                        delta_lower = -K_py * dy
                        servo_targets[IDX_LOWER] = clamp_angle(servo_targets[IDX_LOWER] + delta_lower)
                        send_servo_set(ser, IDX_LOWER, servo_targets[IDX_LOWER])

                    area_error = AREA_NEAR - object_area
                    if abs(area_error) > 200:
                        delta_dist = K_dist * area_error
                        servo_targets[IDX_MIDDLE] = clamp_angle(servo_targets[IDX_MIDDLE] + delta_dist)
                        servo_targets[IDX_UPPER] = clamp_angle(servo_targets[IDX_UPPER] - delta_dist * 0.6)
                        send_servo_set(ser, IDX_MIDDLE, servo_targets[IDX_MIDDLE])
                        send_servo_set(ser, IDX_UPPER, servo_targets[IDX_UPPER])

                    if object_area > AREA_NEAR * 1.6:
                        servo_targets[IDX_GRIP] = clamp_angle(servo_targets[IDX_GRIP] - 2)
                        send_servo_set(ser, IDX_GRIP, servo_targets[IDX_GRIP])

                # Если объект в центре, устанавливаем флаг блокировки
                elif dist_to_center <= CENTER_RADIUS:
                    object_locked = True
                    cv2.putText(frame, "OBJECT LOCKED", (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

                # Если объект был заблокирован, но вышел из круга, снимаем блокировку
                elif object_locked and dist_to_center > CENTER_RADIUS:
                    object_locked = False

            # Если объект слишком мал для отслеживания, снимаем блокировку
            elif object_locked:
                object_locked = False

        else:
            # Объект не найден или слишком мал
            object_locked = False
            cv2.putText(frame, "No green object found", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # Вращаем платформу для поиска объекта
            t = time.time()
            sweep = 90 + int(85 * math.sin(t * 0.5))
            if abs(sweep - servo_targets[IDX_PLATFORM]) > 2:
                servo_targets[IDX_PLATFORM] = clamp_angle(sweep)
                send_servo_set(ser, IDX_PLATFORM, servo_targets[IDX_PLATFORM])

        # Отображаем текущие углы сервоприводов
        cv2.putText(frame, f"Servo P:{servo_targets[IDX_PLATFORM]}",
                    (10, h - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(frame,
                    f"L:{servo_targets[IDX_LOWER]} M:{servo_targets[IDX_MIDDLE]} U:{servo_targets[IDX_UPPER]} G:{servo_targets[IDX_GRIP]}",
                    (10, h - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        # Отображаем статус блокировки
        status_text = "LOCKED" if object_locked else "SEARCHING"
        cv2.putText(frame, f"Status: {status_text}", (w - 200, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow("Frame", frame)
        cv2.imshow("Mask", mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            tracking = False
            ser.write(b"3\n")
            wait_for_response(ser, "DONE")
        time.sleep(0.02)

    cap.release()
    cv2.destroyAllWindows()
    ser.close()


if __name__ == "__main__":
    main()