import cv2
import numpy as np
import serial
import time
import math

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

MIN_AREA_FOR_TRACKING = 500


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


class CameraManager:
    def __init__(self):
        self.cap = None
        self.camera_index = 0
        self.max_attempts = 3

    def initialize_camera(self):
        """Инициализирует камеру с несколькими попытками"""
        for attempt in range(self.max_attempts):
            try:
                if self.cap is not None:
                    self.cap.release()
                    time.sleep(0.5)

                self.cap = cv2.VideoCapture(self.camera_index)

                if not self.cap.isOpened():
                    print(f"Попытка {attempt + 1}: Не удалось открыть камеру")
                    continue

                # Устанавливаем параметры камеры
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                self.cap.set(cv2.CAP_PROP_FPS, 30)

                # Даем камере время на инициализацию
                time.sleep(1)

                # Проверяем, что камера работает
                for i in range(5):
                    ret, frame = self.cap.read()
                    if ret and frame is not None:
                        print(f"Камера успешно инициализирована (попытка {attempt + 1})")
                        return True
                    time.sleep(0.1)

                print(f"Попытка {attempt + 1}: Камера открыта, но не возвращает кадры")

            except Exception as e:
                print(f"Попытка {attempt + 1}: Ошибка при инициализации камеры: {e}")

            time.sleep(1)

        return False

    def read_frame(self):
        """Читает кадр с обработкой ошибок"""
        if self.cap is None:
            return False, None

        try:
            ret, frame = self.cap.read()
            if not ret or frame is None:
                print("Ошибка чтения кадра, пытаемся переинициализировать камеру...")
                if self.initialize_camera():
                    ret, frame = self.cap.read()
                    return ret, frame
                return False, None
            return True, frame
        except Exception as e:
            print(f"Исключение при чтении кадра: {e}")
            return False, None

    def release(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None


def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=SER_TIMEOUT)
    except Exception as e:
        print("Не удалось открыть сериал-порт:", e)
        return

    print("Ожидаем READY от Arduino...")
    if not wait_for_response(ser, "READY", timeout=6.0):
        print("Arduino не ответил READY. Продолжаем.")

    # Инициализируем менеджер камеры
    camera_manager = CameraManager()
    if not camera_manager.initialize_camera():
        print("Не удалось инициализировать камеру после нескольких попыток")
        ser.close()
        return

    tracking = False
    object_locked = False
    frame_count = 0

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

        # Читаем кадр с камеры
        ret, frame = camera_manager.read_frame()
        if not ret:
            print("Не удалось получить кадр с камеры, пробуем снова...")
            time.sleep(0.1)
            continue

        frame_count += 1

        # Пропускаем обработку каждого второго кадра для снижения нагрузки
        if frame_count % 2 == 0:
            continue

        h, w = frame.shape[:2]
        cx_img, cy_img = w // 2, h // 2

        # Обработка изображения
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)

        # Морфологические операции для улучшения качества маски
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        # Поиск контуров
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        object_center = None
        object_area = 0

        if contours:
            # Находим самый большой контур
            largest_contour = max(contours, key=cv2.contourArea)
            object_area = cv2.contourArea(largest_contour)

            if object_area > 100:  # Минимальная площадь для отображения
                M = cv2.moments(largest_contour)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    object_center = (cx, cy)

                    # Отрисовка контура и центра
                    cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 2)
                    cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)
                    cv2.putText(frame, f"Area:{int(object_area)}", (cx + 10, cy + 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Отрисовка центра и целевой области
        cv2.line(frame, (cx_img, 0), (cx_img, h), (255, 255, 255), 1)
        cv2.line(frame, (0, cy_img), (w, cy_img), (255, 255, 255), 1)
        cv2.circle(frame, (cx_img, cy_img), 5, (255, 0, 0), -1)
        cv2.circle(frame, (cx_img, cy_img), CENTER_RADIUS, (0, 255, 255), 2)

        # Обработка объекта
        if object_center is not None and object_area > 100:
            ox, oy = object_center

            # Определение квадранта
            if ox < cx_img and oy < cy_img:
                quad_text = "Top-Left"
            elif ox >= cx_img and oy < cy_img:
                quad_text = "Top-Right"
            elif ox < cx_img and oy >= cy_img:
                quad_text = "Bottom-Left"
            else:
                quad_text = "Bottom-Right"

            cv2.putText(frame, quad_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

            # Проверка положения объекта относительно центра
            dx, dy = ox - cx_img, oy - cy_img
            dist_to_center = math.hypot(dx, dy)

            # Если объект достаточно большой для отслеживания
            if object_area > MIN_AREA_FOR_TRACKING:
                # Если объект вне центра и не заблокирован
                if dist_to_center > CENTER_RADIUS and not object_locked:
                    # Наведение по горизонтали
                    if abs(dx) > CENTER_THRESH_PIX:
                        delta_platform = -K_px * dx
                        servo_targets[IDX_PLATFORM] = clamp_angle(servo_targets[IDX_PLATFORM] + delta_platform)
                        send_servo_set(ser, IDX_PLATFORM, servo_targets[IDX_PLATFORM])

                    # Наведение по вертикали
                    if abs(dy) > CENTER_THRESH_PIX:
                        delta_lower = -K_py * dy
                        servo_targets[IDX_LOWER] = clamp_angle(servo_targets[IDX_LOWER] + delta_lower)
                        send_servo_set(ser, IDX_LOWER, servo_targets[IDX_LOWER])

                    # Регулировка расстояния
                    area_error = AREA_NEAR - object_area
                    if abs(area_error) > 200:
                        delta_dist = K_dist * area_error
                        servo_targets[IDX_MIDDLE] = clamp_angle(servo_targets[IDX_MIDDLE] + delta_dist)
                        servo_targets[IDX_UPPER] = clamp_angle(servo_targets[IDX_UPPER] - delta_dist * 0.6)
                        send_servo_set(ser, IDX_MIDDLE, servo_targets[IDX_MIDDLE])
                        send_servo_set(ser, IDX_UPPER, servo_targets[IDX_UPPER])

                # Блокировка объекта при попадании в центр
                elif dist_to_center <= CENTER_RADIUS:
                    object_locked = True
                    cv2.putText(frame, "OBJECT LOCKED", (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

                # Разблокировка при выходе из центра
                elif object_locked and dist_to_center > CENTER_RADIUS:
                    object_locked = False

            # Снятие блокировки для маленьких объектов
            elif object_locked:
                object_locked = False

        else:
            # Объект не найден
            object_locked = False
            cv2.putText(frame, "No green object found", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # Поисковое движение платформы
            t = time.time()
            sweep = 90 + int(85 * math.sin(t * 0.3))  # Более медленное движение
            if abs(sweep - servo_targets[IDX_PLATFORM]) > 3:
                servo_targets[IDX_PLATFORM] = clamp_angle(sweep)
                send_servo_set(ser, IDX_PLATFORM, servo_targets[IDX_PLATFORM])

        # Отображение информации
        cv2.putText(frame, f"Servo P:{servo_targets[IDX_PLATFORM]}",
                    (10, h - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(frame,
                    f"L:{servo_targets[IDX_LOWER]} M:{servo_targets[IDX_MIDDLE]} U:{servo_targets[IDX_UPPER]} G:{servo_targets[IDX_GRIP]}",
                    (10, h - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        status_text = "LOCKED" if object_locked else "SEARCHING"
        cv2.putText(frame, f"Status: {status_text}", (w - 200, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Отображение окон
        cv2.imshow("Frame", frame)
        cv2.imshow("Mask", mask)

        # Обработка клавиш
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            tracking = False
            ser.write(b"3\n")
            wait_for_response(ser, "DONE")
            print("Выход из режима поиска")

        # Небольшая задержка для снижения нагрузки на CPU
        time.sleep(0.03)

    # Освобождение ресурсов
    camera_manager.release()
    cv2.destroyAllWindows()
    ser.close()
    print("Программа завершена")


if __name__ == "__main__":
    main()