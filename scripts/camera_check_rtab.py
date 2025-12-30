#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from message_filters import Subscriber, ApproximateTimeSynchronizer
import tf2_ros
import statistics
import time


def to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class CameraCheck(Node):
    def __init__(self,
                 rgb_topic='/camera/camera/color/image_raw',
                 depth_topic='/camera/camera/depth/image_rect_raw',
                 rgb_info_topic='/camera/camera/color/camera_info',
                 depth_info_topic='/camera/camera/depth/camera_info',
                 samples=50,
                 timeout_s=15.0):
        super().__init__('camera_check_rtab')
        self.get_logger().info('Запущен узел проверки камеры')
        self.rgb_info = None
        self.depth_info = None
        self.samples_target = samples
        self.timeout = timeout_s
        self.dt_list = []
        self.rgb_last = None
        self.depth_last = None

        # CameraInfo subscriptions (single-shot capture)
        self.create_subscription(CameraInfo, rgb_info_topic, self.rgb_info_cb, 10)
        self.create_subscription(CameraInfo, depth_info_topic, self.depth_info_cb, 10)

        # Image synchronizer
        self.sub_rgb = Subscriber(self, Image, rgb_topic)
        self.sub_depth = Subscriber(self, Image, depth_topic)
        self.sync = ApproximateTimeSynchronizer([self.sub_rgb, self.sub_depth], queue_size=20, slop=0.1)
        self.sync.registerCallback(self.images_cb)

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.start_time = time.time()

    def rgb_info_cb(self, msg: CameraInfo):
        if self.rgb_info is None:
            self.rgb_info = msg
            self.get_logger().info('Got rgb camera_info')

    def depth_info_cb(self, msg: CameraInfo):
        if self.depth_info is None:
            self.depth_info = msg
            self.get_logger().info('Got depth camera_info')

    def images_cb(self, rgb: Image, depth: Image):
        # compute dt = depth - rgb in ms
        dt = to_sec(depth.header.stamp) - to_sec(rgb.header.stamp)
        self.dt_list.append(dt * 1000.0)
        self.rgb_last = rgb
        self.depth_last = depth
        # log progress occasionally
        if len(self.dt_list) % 10 == 0:
            self.get_logger().info(f'Собрано {len(self.dt_list)}/{self.samples_target} образцов')

    def summarize(self):
        print('--- Результат проверки камеры ---')
        # Topics seen
        if self.rgb_last:
            print(f'RGB топик: найден, encoding={self.rgb_last.encoding}, размер={self.rgb_last.width}x{self.rgb_last.height}, frame_id={self.rgb_last.header.frame_id}')
        else:
            print('RGB топик: НЕТ ДАННЫХ')
        if self.depth_last:
            print(f'DEPTH топик: найден, encoding={self.depth_last.encoding}, размер={self.depth_last.width}x{self.depth_last.height}, frame_id={self.depth_last.header.frame_id}')
        else:
            print('DEPTH topic: NO DATA')

        # Encodings
        if self.rgb_last and self.rgb_last.encoding not in ['rgb8', 'bgr8']:
            print('Внимание: кодировка RGB не rgb8/bgr8')
        if self.depth_last and self.depth_last.encoding not in ['16UC1', '32FC1']:
            print('Внимание: кодировка глубины не 16UC1/32FC1')

        # Sizes
        if self.rgb_last and self.depth_last:
            if self.rgb_last.width == self.depth_last.width and self.rgb_last.height == self.depth_last.height:
                print('Разрешение: RGB и DEPTH СОВПАДАЮТ')
            else:
                print('Разрешение: RGB и DEPTH РАЗЛИЧАЮТСЯ')

        # CameraInfo
        if self.rgb_info:
            print(f'camera_info (RGB): frame_id={self.rgb_info.header.frame_id}, размер={self.rgb_info.width}x{self.rgb_info.height}')
        else:
            print('camera_info (RGB): ОТСУТСТВУЕТ')
        if self.depth_info:
            print(f'camera_info (DEPTH): frame_id={self.depth_info.header.frame_id}, размер={self.depth_info.width}x{self.depth_info.height}')
        else:
            print('camera_info (DEPTH): ОТСУТСТВУЕТ')

        # Alignment hint
        if self.depth_last and self.rgb_last:
            if self.depth_last.header.frame_id == self.rgb_last.header.frame_id:
                print('Подсказка: глубина ВЫРОВНЕНА по цвету (одинаковый frame_id)')
            else:
                print('Подсказка: глубина НЕ выровнена по цвету (разные frame_id)')

        # dt stats
        if len(self.dt_list) > 0:
            vals = self.dt_list
            print(f'Выборка dt: {len(vals)}')
            print(f' dt (ms) - mean={statistics.mean(vals):.2f}, median={statistics.median(vals):.2f}, min={min(vals):.2f}, max={max(vals):.2f}, stdev={statistics.pstdev(vals):.2f}')
            if abs(statistics.mean(vals)) <= 30.0:
                print('Синхронизация: OK (среднее dt ≲ 30 ms)')
            else:
                print('Синхронизация: ВНИМАНИЕ (среднее dt > 30 ms) — рассмотрите approx_sync или align_depth')
        else:
            print('Нет собранных dt-образцов')

        # TF check
        try:
            # try lookup camera_depth_optical_frame -> camera_color_optical_frame
            trans = self.tf_buffer.lookup_transform('camera_color_optical_frame', 'camera_depth_optical_frame', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            t = trans.transform.translation
            r = trans.transform.rotation
            print('TF: НАЙДЕН camera_depth_optical_frame -> camera_color_optical_frame')
            print(f' Смещение: x={t.x:.3f} y={t.y:.3f} z={t.z:.3f}')
            print(f' Поворот (кватернион): x={r.x:.4f} y={r.y:.4f} z={r.z:.4f} w={r.w:.4f}')
        except Exception as e:
            print('TF: НЕ УДАЛОСЬ ПОЛУЧИТЬ camera_depth_optical_frame -> camera_color_optical_frame (tf может быть не готов)')
            # try to show tf_static head
            print('Попробуйте: ros2 topic echo /tf_static --once')

        print('--- Конец отчёта ---')


def main(args=None):
    rclpy.init(args=args)
    node = CameraCheck()
    start = time.time()
    try:
        # spin until we collected samples or timeout
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if len(node.dt_list) >= node.samples_target:
                break
            if time.time() - start > node.timeout:
                break
    except KeyboardInterrupt:
        pass

    node.summarize()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
