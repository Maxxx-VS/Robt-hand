import cv2
import subprocess
import re


def get_max_camera_resolution(device_index=0):
    """Получает максимальное доступное разрешение камеры с помощью v4l2-ctl"""
    try:
        # Получаем информацию о поддерживаемых разрешениях
        result = subprocess.run([
            'v4l2-ctl',
            f'--device=/dev/video{device_index}',
            '--list-formats-ext'
        ], capture_output=True, text=True, timeout=10)

        if result.returncode != 0:
            return None

        # Ищем все разрешения в выводе
        resolutions = []
        lines = result.stdout.split('\n')
        for line in lines:
            match = re.search(r'(\d+)x(\d+)', line)
            if match:
                w = int(match.group(1))
                h = int(match.group(2))
                resolutions.append((w, h))

        if not resolutions:
            return None

        # Находим максимальное разрешение (по количеству пикселей)
        max_resolution = max(resolutions, key=lambda x: x[0] * x[1])
        return max_resolution

    except (subprocess.TimeoutExpired, FileNotFoundError, subprocess.SubprocessError):
        return None


def show_usb_camera_max_res(camera_index=0, window_name='USB Camera'):
    """Выводит видео с USB-камеры в максимальном разрешении"""

    # Пытаемся получить максимальное разрешение
    max_res = get_max_camera_resolution(camera_index)

    # Создаем объект для захвата видео
    cap = cv2.VideoCapture(camera_index)

    # Устанавливаем максимальное разрешение, если удалось его определить
    if max_res:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, max_res[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, max_res[1])
        print(f"Установлено максимальное разрешение: {max_res[0]}x{max_res[1]}")
    else:
        print("Не удалось определить максимальное разрешение. Используется стандартное.")

    # Проверяем, открылась ли камера
    if not cap.isOpened():
        print(f"Ошибка: Не удалось открыть камеру с индексом {camera_index}")
        return

    # Получаем фактическое разрешение
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Фактическое разрешение: {actual_width}x{actual_height}")

    try:
        while True:
            # Захватываем кадр
            ret, frame = cap.read()

            if not ret:
                print("Ошибка: Не удалось получить кадр с камеры")
                break

            # Показываем кадр в окне
            cv2.imshow(window_name, frame)

            # Обрабатываем клавиши
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):  # Выход по 'q'
                break
            elif key == ord('s'):  # Сохранить снимок по 's'
                cv2.imwrite('camera_snapshot.jpg', frame)
                print("Снимок сохранен как 'camera_snapshot.jpg'")

    finally:
        # Освобождаем ресурсы
        cap.release()
        cv2.destroyAllWindows()
        print("Ресурсы освобождены")


# Запуск скрипта
if __name__ == "__main__":
    show_usb_camera_max_res(camera_index=0)