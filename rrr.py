import cv2
import numpy as np


def show_usb_camera_fisheye_fix(camera_index=0, window_name='USB Camera'):
    cap = cv2.VideoCapture(camera_index)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)

    if not cap.isOpened():
        print(f"Ошибка: Не удалось открыть камеру {camera_index}")
        return

    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Фактическое разрешение захвата: {actual_width}x{actual_height}")

    # Мнимая матрица камеры
    K = np.array([[actual_width, 0, actual_width / 2],
                  [0, actual_width, actual_height / 2],
                  [0, 0, 1]], dtype=np.float32)

    # Коэффициенты дисторсии fisheye (4x1)
    D = np.zeros((4, 1), dtype=np.float32)

    cv2.namedWindow(window_name)
    cv2.createTrackbar("k1", window_name, 50, 100, lambda x: None)      # от -1 до +1
    cv2.createTrackbar("k2", window_name, 50, 100, lambda x: None)      # от -1 до +1
    cv2.createTrackbar("Balance", window_name, 50, 100, lambda x: None) # от 0 до 1

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Ошибка: кадр не получен")
                break

            # Читаем значения слайдеров
            k1_val = cv2.getTrackbarPos("k1", window_name)
            k2_val = cv2.getTrackbarPos("k2", window_name)
            balance_val = cv2.getTrackbarPos("Balance", window_name)

            # Переводим в диапазон [-1.0 ... +1.0]
            k1 = (k1_val - 50) / 50.0
            k2 = (k2_val - 50) / 50.0
            balance = balance_val / 100.0

            D[:] = np.array([[k1], [k2], [0.0], [0.0]], dtype=np.float32)

            # Новая матрица камеры
            new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                K, D, (actual_width, actual_height), np.eye(3), balance=balance
            )

            # Карты преобразования
            map1, map2 = cv2.fisheye.initUndistortRectifyMap(
                K, D, np.eye(3), new_K, (actual_width, actual_height), cv2.CV_16SC2
            )

            undistorted = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR)

            # Уменьшаем для отображения
            frame_resized = cv2.resize(frame, (960, 540))
            undist_resized = cv2.resize(undistorted, (960, 540))

            # Склеиваем оригинал и исправленный кадр
            combined = np.hstack((frame_resized, undist_resized))
            cv2.imshow(window_name, combined)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                cv2.imwrite('camera_original.jpg', frame_resized)
                cv2.imwrite('camera_corrected.jpg', undist_resized)
                print("Снимки сохранены: camera_original.jpg и camera_corrected.jpg")

    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("Ресурсы освобождены")


if __name__ == "__main__":
    show_usb_camera_fisheye_fix(camera_index=0)


# k1 = 45 k2 = 0 Balance = 100
# k1 = 0 k2 = 100 Balance = 100