#!/usr/bin/env python3
"""Single RViz viewer for live localization and explicitly selected recorded outputs.

Display-only common XY translation; no per-stream yaw reset, no estimator input.
Time navigation redraws buffered results and never rewinds ROS /clock.
"""
import argparse
import bisect
import csv
import json
import math
import os
from pathlib import Path
import queue
import sys
import time
from typing import Dict, Tuple
import numpy as np
import rosbag
import rospkg
import yaml

WGS84_A_M = 6378137.0
WGS84_E2 = 6.69437999014e-3
PACKAGE = Path(rospkg.RosPack().get_path('mando_localization'))
WORKSPACE = PACKAGE.parent.parent
FRAME = 'localization_debug'
PREFIX = '/mando_localization/visualization/debug'
DEFAULT_CONFIG = yaml.safe_load((PACKAGE/'config/localization_viewer.yaml').read_text())
TOPICS = dict(DEFAULT_CONFIG['topics'])
COLORS = {key:tuple(value) for key,value in DEFAULT_CONFIG['colors'].items()}


def load_rddf(rddf_dir: Path) -> Tuple[Dict[str, np.ndarray], Dict]:
    project_path = rddf_dir / "yongin_route_project.json"
    project = json.loads(project_path.read_text(encoding="utf-8-sig"))
    routes: Dict[str, np.ndarray] = {}
    expected_header = [
        "route_id",
        "route_name",
        "closed",
        "index",
        "latitude",
        "longitude",
        "east_m",
        "north_m",
        "distance_m",
        "path_yaw_rad",
    ]
    for path in sorted(rddf_dir.glob("*.csv")):
        with path.open(encoding="utf-8-sig", newline="") as stream:
            reader = csv.DictReader(stream)
            if reader.fieldnames != expected_header:
                raise RuntimeError(f"unexpected RDDF header: {path}")
            rows = list(reader)
        if not rows:
            raise RuntimeError(f"empty RDDF CSV: {path}")
        name = rows[0]["route_name"]
        routes[name] = np.asarray(
            [(float(row["east_m"]), float(row["north_m"])) for row in rows],
            dtype=float,
        )
    if not routes or not all(np.isfinite(points).all() for points in routes.values()):
        raise ValueError('RDDF routes must contain finite points')
    return routes, project

def project_wgs84(
    latitude_deg: float, longitude_deg: float, origin: Dict[str, float]
) -> Tuple[float, float]:
    latitude_rad = math.radians(float(origin["lat"]))
    sin_latitude = math.sin(latitude_rad)
    denominator = math.sqrt(1.0 - WGS84_E2 * sin_latitude * sin_latitude)
    prime_vertical_radius_m = WGS84_A_M / denominator
    meridian_radius_m = (
        WGS84_A_M
        * (1.0 - WGS84_E2)
        / (denominator * denominator * denominator)
    )
    east_m = (
        prime_vertical_radius_m
        * math.cos(latitude_rad)
        * math.radians(longitude_deg - float(origin["lng"]))
    )
    north_m = meridian_radius_m * math.radians(
        latitude_deg - float(origin["lat"])
    )
    return east_m, north_m

def yaw_of(q):
    values = np.asarray([q.x, q.y, q.z, q.w], dtype=float)
    norm = float(np.linalg.norm(values))
    if not np.isfinite(values).all() or abs(norm - 1.0) > 0.01:
        raise ValueError('invalid orientation quaternion')
    x, y, z, w = values / norm
    return math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))


def final_bag(path):
    path = Path(path).expanduser().resolve(strict=True)
    if path.suffix != '.bag' or Path(str(path) + '.active').exists():
        raise RuntimeError('Recording must be finalized before viewing: ' + str(path))
    return path


def decode_sample(key, message, project):
    if key in ('local', 'global'):
        p = message.pose.pose.position
        value = (p.x, p.y, yaw_of(message.pose.pose.orientation))
        if not np.isfinite(value).all():
            raise ValueError('non-finite odometry')
    elif key == 'gps':
        if (message.status.status < 0 or not np.isfinite([message.latitude, message.longitude]).all()
                or not -90 <= message.latitude <= 90 or not -180 <= message.longitude <= 180):
            raise ValueError('invalid GPS fix')
        value = project_wgs84(message.latitude, message.longitude, project['origin'])
    elif key == 'speed':
        value = float(message.speed)
        if not math.isfinite(value):
            raise ValueError('non-finite speed')
    elif key in ('state', 'valid'):
        value = message.data
    elif key.startswith('imu_'):
        value = yaw_of(message.orientation)
    elif key == 'calibration':
        statuses = [status for status in message.status if status.name == 'CalibratedIMU']
        if not statuses:
            return None
        status = statuses[-1]
        value = {'diagnostic_level': status.level, 'diagnostic_message': status.message}
        for pair in status.values:
            try:
                value[pair.key] = json.loads(pair.value)
            except ValueError:
                value[pair.key] = pair.value
    else:
        # LaserScan ranges are already float32 on the wire; keeping compact
        # arrays avoids retaining millions of boxed Python float objects.
        value = {'ranges': np.asarray(message.ranges, dtype=np.float32),
                 'angle_min': message.angle_min, 'angle_increment': message.angle_increment,
                 'range_min': message.range_min, 'range_max': message.range_max}
    return value


class SceneData:
    """Same decoding, anchor, ordering and retention rules for files and live input."""
    def __init__(self, rddf_dir, source_start=None, duration=0., processed_bag=None, limit=200000):
        self.routes, self.project = load_rddf(Path(rddf_dir))
        self.start, self.duration, self.limit = source_start, duration, limit
        self.data = {key: [] for key in TOPICS}
        self.times = {key: [] for key in TOPICS}
        self.mount = None
        self.shift = None
        self.first_local = self.first_gps = None
        self.summary = {'processed_bag': str(processed_bag) if processed_bag else None,
                        'common_xy_translation_m': None, 'additional_viewer_yaw_rotation_rad': 0.,
                        'skipped': 0, 'buffer_dropped': 0, 'clock_resets': 0}
        self.last_receipt = None

    def reset(self, stamp):
        for key in TOPICS:
            self.data[key].clear(); self.times[key].clear()
        self.first_local = self.first_gps = self.shift = None
        self.start, self.duration = stamp, 0.
        self.summary['common_xy_translation_m'] = None
        self.summary['clock_resets'] += 1

    def ingest(self, key, message, stamp, live=False):
        if key == 'tf':
            for tr in message.transforms:
                if tr.header.frame_id.lstrip('/') == 'base_link' and tr.child_frame_id.lstrip('/') == 'laser_link':
                    q, p = tr.transform.rotation, tr.transform.translation
                    vals = [p.x,p.y,p.z,q.x,q.y,q.z,q.w]
                    if np.isfinite(vals).all() and abs(np.linalg.norm(vals[3:])-1)<.01:
                        self.mount = tr
            return
        if not math.isfinite(stamp) or stamp <= 0:
            self.summary['skipped'] += 1; return
        if self.start is None:
            self.start = stamp
        if live and self.last_receipt is not None and stamp < self.last_receipt-1.0:
            self.reset(stamp)
        self.last_receipt = stamp
        elapsed = stamp-self.start
        if elapsed < 0 or (not live and elapsed > self.duration):
            self.summary['skipped'] += 1; return
        if self.times[key] and elapsed < self.times[key][-1]:
            self.summary['skipped'] += 1; return
        try:
            value = decode_sample(key, message, self.project)
            if value is None: return
        except (ValueError, TypeError, OverflowError):
            self.summary['skipped'] += 1; return
        self.times[key].append(elapsed); self.data[key].append(value)
        if live: self.duration = max(self.duration, elapsed)
        if key == 'local' and self.first_local is None: self.first_local = np.asarray(value[:2])
        if key == 'gps' and self.first_gps is None: self.first_gps = np.asarray(value)
        if self.shift is None and self.first_local is not None and self.first_gps is not None:
            self.shift = self.first_gps-self.first_local
            self.summary['common_xy_translation_m'] = self.shift.tolist()
        limit = min(self.limit, 6000) if key == 'scan' else self.limit
        if live and len(self.data[key]) > limit:
            count = max(1,limit//10)
            del self.data[key][:count]; del self.times[key][:count]
            self.summary['buffer_dropped'] += count

    def points(self, key, count=None):
        width = 2 if key == 'gps' else 3
        values = np.asarray(self.data[key][:count], dtype=float).reshape((-1,width)).copy()
        if key != 'gps':
            if self.shift is None: return np.empty((0,width))
            values[:,:2] += self.shift
        return values


def load_data(processed_bag, source_bag, rddf_dir, limit):
    processed_bag = final_bag(processed_bag)
    source_bag = final_bag(source_bag) if source_bag else processed_bag
    with rosbag.Bag(str(source_bag)) as bag:
        start, end = bag.get_start_time(), bag.get_end_time()
    model = SceneData(rddf_dir, start, end-start, processed_bag, limit)
    reverse = {value: key for key,value in TOPICS.items()}
    with rosbag.Bag(str(processed_bag)) as bag:
        for topic,message,stamp in bag.read_messages(topics=list(reverse)+['/tf_static']):
            model.ingest('tf' if topic == '/tf_static' else reverse[topic],message,stamp.to_sec())
    if not any(model.data[key] for key in ('local','global')):
        raise ValueError('Selected bag has no computed Local/Global output. Recompute with replay.launch first.')
    model.summary.update(counts={key:len(rows) for key,rows in model.data.items()},duration_s=model.duration,
                         source_bag=str(source_bag))
    return model

def run_gui(model, seek, live=False, config=None):
    import rospy
    from geometry_msgs.msg import Point, TransformStamped
    from visualization_msgs.msg import Marker, MarkerArray
    from tf2_msgs.msg import TFMessage
    from std_msgs.msg import String, Float64, Bool
    from python_qt_binding import QtCore, QtGui, QtWidgets
    from rviz import bindings as rviz
    from tf.transformations import quaternion_matrix

    class Slider(QtWidgets.QSlider):
        def point(self, event):
            return round(self.maximum() * max(0, min(1, event.x()/max(1, self.width()))))

        def mousePressEvent(self, event):
            if event.button() == QtCore.Qt.LeftButton:
                self.setSliderDown(True)
                self.setValue(self.point(event))

        def mouseMoveEvent(self, event):
            if self.isSliderDown():
                self.setValue(self.point(event))

        def mouseReleaseEvent(self, event):
            self.setSliderDown(False)

    class Viewer(QtWidgets.QWidget):
        seek_signal = QtCore.Signal(float)
        follow_signal = QtCore.Signal(bool)

        def __init__(self):
            super().__init__()
            self.model = model
            self.routes, self.data, self.times = model.routes, model.data, model.times
            self.duration, self.mount, self.summary = model.duration, model.mount, model.summary
            self.follow_live = live
            self.inbox = queue.Queue(maxsize=10000)
            self.input_dropped = 0
            self.position = self.duration if seek is None else max(0.0, min(self.duration, seek))
            self.playing, self.last, self.rate, self.drag_play = False, time.monotonic(), 1.0, False
            rospy.init_node('mando_localization_rviz', disable_signals=True)
            self.scene = rospy.Publisher(PREFIX+'/scene', MarkerArray, queue_size=1, latch=True)
            self.status = rospy.Publisher(PREFIX+'/status', String, queue_size=1, latch=True)
            self.tf = rospy.Publisher('/tf_static', TFMessage, queue_size=1, latch=True)
            transform = TransformStamped()
            transform.header.frame_id, transform.child_frame_id = FRAME+'_world', FRAME
            transform.transform.rotation.w = 1
            self.tf.publish(TFMessage([transform]))
            self.seek_signal.connect(self.seek)
            self.follow_signal.connect(lambda follow: self.go_latest() if follow else setattr(self, "follow_live", False))
            self.follow_remote = rospy.Subscriber(PREFIX+"/follow_live", Bool, lambda msg: self.follow_signal.emit(msg.data))
            self.remote = rospy.Subscriber(PREFIX+'/seek_seconds', Float64,
                                            lambda msg: self.seek_signal.emit(msg.data))
            self.setWindowTitle('Localization · RDDF 공통 RViz · '+('실시간 입력' if live else str(model.summary['processed_bag'])))
            layout = QtWidgets.QVBoxLayout(self)
            heading = QtWidgets.QLabel('RDDF 공통 뷰어  |  초록 Local · 빨강 Global · 보라 Raw GPS · 파랑 RDDF')
            heading.setStyleSheet('font-size:16px;font-weight:bold;padding:6px')
            layout.addWidget(heading)
            self.frame = rviz.VisualizationFrame()
            self.frame.setSplashPath('')
            self.frame.initialize()
            self.frame.setMenuBar(None)
            self.frame.setStatusBar(None)
            layout.addWidget(self.frame, 1)
            manager = self.frame.getManager()
            manager.setFixedFrame(FRAME)
            manager.removeAllDisplays()
            display = manager.createDisplay('rviz/MarkerArray', '기록 시점 장면', True)
            display.subProp('Marker Topic').setValue(PREFIX+'/scene')
            grid = manager.createDisplay('rviz/Grid', '5 m 격자', True)
            grid.subProp('Cell Size').setValue(5.0)
            grid.subProp('Plane Cell Count').setValue(150)
            view = manager.getViewManager()
            view.setCurrentViewControllerType('rviz/TopDownOrtho')
            self.view = view.getCurrent()
            self.view.subProp('Angle').setValue(0.0)
            self.info = QtWidgets.QLabel()
            self.info.setStyleSheet('font-size:14px;padding:5px')
            self.info.setWordWrap(True)
            layout.addWidget(self.info)
            self.slider = Slider(QtCore.Qt.Horizontal)
            self.slider.setRange(0, math.ceil(self.duration*10))
            self.slider.setMinimumHeight(32)
            layout.addWidget(self.slider)
            self.slider.sliderPressed.connect(self.drag_start)
            self.slider.sliderReleased.connect(self.drag_end)
            self.slider.valueChanged.connect(lambda value: self.seek(value/10) if self.slider.isSliderDown() else None)
            controls = QtWidgets.QHBoxLayout()
            layout.addLayout(controls)

            def button(label, callback):
                widget = QtWidgets.QPushButton(label)
                widget.setMinimumHeight(37)
                widget.clicked.connect(callback)
                controls.addWidget(widget)
                return widget

            button('처음', lambda: self.seek(0))
            button('−10초', lambda: self.seek(self.position-10))
            self.play_button = button('▶ 재생', self.toggle)
            button('+10초', lambda: self.seek(self.position+10))
            button('최신 / 끝', self.go_latest)
            button('전체 경로 맞춤', self.fit_view)
            speed = QtWidgets.QComboBox()
            speed.addItems(['0.25×', '0.5×', '1×', '2×', '4×', '8×'])
            speed.setCurrentIndex(2)
            speed.currentIndexChanged.connect(lambda index: setattr(self, 'rate', [.25, .5, 1, 2, 4, 8][index]))
            controls.addWidget(speed)
            self.jump = QtWidgets.QLineEdit()
            self.jump.setPlaceholderText('분:초 (예: 08:50)')
            self.jump.setMaximumWidth(180)
            self.jump.returnPressed.connect(self.jump_time)
            controls.addWidget(self.jump)
            button('이동', self.jump_time)
            self.clock_label = QtWidgets.QLabel()
            controls.addWidget(self.clock_label)
            note = QtWidgets.QLabel('Space 재생/정지 · ←/→ 10초 · Local/Global 동일 XY 이동 · yaw 회전 없음 · 시간 탐색은 화면만 이동 · RDDF는 설계 경로')
            note.setStyleSheet('color:#777;padding:4px')
            note.setWordWrap(True)
            layout.addWidget(note)
            for key, callback in [('Space', self.toggle), ('Left', lambda: self.seek(self.position-10)),
                                  ('Right', lambda: self.seek(self.position+10))]:
                QtWidgets.QShortcut(QtGui.QKeySequence(key), self, callback)
            self.timer = QtCore.QTimer(self)
            self.timer.timeout.connect(self.tick)
            self.timer.start(int(config['refresh_ms']))
            self.subscribers = self.subscribe_live() if live else []
            self.resize(1600, 1050)
            self.fit_view()
            self.render()

        def fit_view(self):
            points = np.vstack(list(self.routes.values()) + [self.model.points(key)[:, :2] for key in ('local', 'global', 'gps')])
            lower, upper = points.min(axis=0), points.max(axis=0)
            center, span = (lower+upper)/2, np.maximum(upper-lower, 1)
            self.view.subProp('X').setValue(float(center[0]))
            self.view.subProp('Y').setValue(float(center[1]))
            scale = .8*min(max(600, self.frame.width())/span[0], max(500, self.frame.height())/span[1])
            self.view.subProp('Scale').setValue(float(scale))

        def enqueue(self, key, message):
            try:
                self.inbox.put_nowait((key,message,rospy.Time.now().to_sec()))
            except queue.Full:
                self.input_dropped += 1

        def subscribe_live(self):
            from nav_msgs.msg import Odometry
            from sensor_msgs.msg import Imu, NavSatFix, LaserScan
            from diagnostic_msgs.msg import DiagnosticArray
            from std_msgs.msg import Bool
            from erp42_msgs.msg import SerialFeedBack
            types = dict(local=Odometry, **{'global':Odometry}, gps=NavSatFix, scan=LaserScan,
                         speed=SerialFeedBack, state=String, valid=Bool, imu_raw=Imu,
                         imu_normalized=Imu, imu_calibrated=Imu, calibration=DiagnosticArray)
            subs = [rospy.Subscriber(topic,types[key],lambda msg,k=key:self.enqueue(k,msg),queue_size=100)
                    for key,topic in TOPICS.items()]
            subs.append(rospy.Subscriber('/tf_static',TFMessage,lambda msg:self.enqueue('tf',msg),queue_size=20))
            return subs

        def go_latest(self):
            self.seek(self.duration)
            self.follow_live = live

        def drag_start(self):
            self.drag_play, self.playing = self.playing, False

        def drag_end(self):
            self.playing, self.last = self.drag_play, time.monotonic()

        def seek(self, value):
            if math.isfinite(value):
                self.follow_live = False
                self.position = max(0.0, min(self.duration, value))
                self.last = time.monotonic()
                self.render()

        def toggle(self):
            if self.follow_live:
                self.follow_live = False
                self.playing = False
                return
            if self.position >= self.duration:
                self.position = 0
            self.playing = not self.playing
            self.last = time.monotonic()
            self.render()

        def jump_time(self):
            try:
                value = sum(float(part)*60**index for index, part in enumerate(reversed(self.jump.text().split(':'))))
                if not math.isfinite(value):
                    raise ValueError('non-finite time')
                self.seek(value)
            except ValueError:
                self.jump.setText('분:초 형식으로 입력')

        def tick(self):
            now = time.monotonic()
            if rospy.is_shutdown():
                self.close(); return
            if live:
                for _ in range(10000):
                    try: key,message,stamp = self.inbox.get_nowait()
                    except queue.Empty: break
                    self.model.ingest(key,message,stamp,live=True)
                self.duration, self.mount = self.model.duration, self.model.mount
                self.slider.setMaximum(math.ceil(self.duration*10))
                if self.follow_live: self.position = self.duration
                self.render()
            if self.playing:
                self.position = min(self.duration, self.position+(now-self.last)*self.rate)
                if self.position >= self.duration:
                    self.playing = False
                self.render()
            self.last = now

        def latest(self, key, default=None):
            index = bisect.bisect_right(self.times[key], self.position)-1
            return self.data[key][index] if index >= 0 else default

        def marker(self, ident, kind, color, points, width=.22):
            message = Marker()
            message.header.frame_id, message.ns, message.id = FRAME, 'localization_debug', ident
            message.type, message.action = kind, Marker.ADD
            message.pose.orientation.w = 1
            message.scale.x = message.scale.y = message.scale.z = width
            message.color.r, message.color.g, message.color.b = color
            message.color.a = 1
            message.points = [Point(float(point[0]), float(point[1]), float(point[2]) if len(point)>2 else 0.0) for point in points]
            return message

        def render(self):
            markers, current, last_times, counts = [], {}, {}, {}
            for route in self.routes.values():
                markers.append(self.marker(len(markers), Marker.LINE_STRIP, COLORS['rddf'], route, .15))
            for key in ('local', 'global', 'gps'):
                count = bisect.bisect_right(self.times[key], self.position)
                counts[key] = count
                if not count:
                    continue
                values, stamps = self.model.points(key,count), np.asarray(self.times[key][:count])
                if not len(values): continue
                last_times[key] = float(stamps[-1])
                boundaries = np.r_[0, np.flatnonzero(np.diff(stamps)>.5)+1, count]
                lines = []
                for left, right in zip(boundaries[:-1], boundaries[1:]):
                    xy = values[left:right, :2]
                    xy = xy[::max(1, math.ceil(len(xy)/5000))]
                    if len(xy)>1:
                        lines.extend(np.stack([xy[:-1], xy[1:]], axis=1).reshape((-1, 2)))
                markers.append(self.marker(len(markers), Marker.LINE_LIST, COLORS[key], lines, .3))
                if self.position-stamps[-1]>.5:
                    continue
                current[key] = values[-1]
                if key == 'gps':
                    markers.append(self.marker(len(markers), Marker.POINTS, COLORS[key], [values[-1]], .9))
                else:
                    x, y, yaw = values[-1]
                    box = np.array([[-.675, -.425], [.675, -.425], [.675, .425], [-.675, .425], [-.675, -.425]])
                    rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
                    markers.append(self.marker(len(markers), Marker.LINE_STRIP, COLORS[key], box@rotation.T+[x, y], .18))
                    markers.append(self.marker(len(markers), Marker.ARROW, COLORS[key], [(x, y), (x+3*math.cos(yaw), y+3*math.sin(yaw))], .4))
            scan_index = bisect.bisect_right(self.times['scan'], self.position)-1
            scan_fresh = scan_index >= 0 and self.position-self.times['scan'][scan_index]<.5
            scan_visible = scan_fresh and 'global' in current and self.mount is not None
            if scan_visible:
                scan = self.data['scan'][scan_index]
                q, tr = self.mount.transform.rotation, self.mount.transform.translation
                rotation = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
                ranges = scan['ranges']
                angles = scan['angle_min']+np.arange(len(ranges))*scan['angle_increment']
                valid_ranges = np.isfinite(ranges)&(ranges>=scan['range_min'])&(ranges<=scan['range_max'])
                xyz = np.column_stack([ranges[valid_ranges]*np.cos(angles[valid_ranges]), ranges[valid_ranges]*np.sin(angles[valid_ranges]), np.zeros(sum(valid_ranges))])@rotation.T
                xyz += [tr.x, tr.y, tr.z]
                x, y, yaw = current['global']
                c, s = math.cos(yaw), math.sin(yaw)
                xyz[:, :2] = xyz[:, :2]@np.array([[c, s], [-s, c]])+[x, y]
                markers.append(self.marker(len(markers), Marker.POINTS, (.1, 1, 1), xyz, .10))
            clear = Marker()
            clear.action = Marker.DELETEALL
            self.scene.publish(MarkerArray([clear]+markers))
            state, valid = self.latest('state', '기록 없음'), self.latest('valid', False)
            speed = self.latest('speed')
            speed_text = '기록 없음' if speed is None else '{:.2f} m/s'.format(speed)
            calibration = self.latest('calibration', {})
            calib_text = 'CalibratedIMU 진단: 기록 없음'
            if calibration:
                calib_text = 'CalibratedIMU {} | correction_count={} | 목표 offset={}° / 적용={}° | {}'.format(
                    calibration.get('state', '알 수 없음'), calibration.get('correction_count', '?'),
                    calibration.get('yaw_offset_deg', '?'), calibration.get('applied_yaw_offset_deg', '?'),
                    calibration.get('diagnostic_message', ''))
            headings = []
            for key, label in [('imu_raw', 'Raw'), ('imu_calibrated', 'Calibrated')]:
                yaw = self.latest(key)
                if yaw is not None:
                    headings.append('{} yaw {:.1f}°'.format(label, math.degrees(yaw)))
            self.info.setText('상태: {}  valid={}  속도 {}  LiDAR: {}\n{}\n{}'.format(
                state + (' | GPS·Local 기준점 대기' if self.model.shift is None else ''), valid, speed_text, '기록 있음' if scan_visible else '이 시점 표시 없음', calib_text, ' | '.join(headings)))
            self.clock_label.setText('{:02d}:{:04.1f} / {:02d}:{:04.1f}'.format(
                int(self.position)//60, self.position%60, int(self.duration)//60, self.duration%60))
            self.play_button.setText('⏸ 화면 정지' if self.playing or self.follow_live else '▶ 화면 재생')
            if not self.slider.isSliderDown():
                self.slider.blockSignals(True)
                self.slider.setValue(round(self.position*10))
                self.slider.blockSignals(False)
            self.status.publish(String(json.dumps({
                'mode': 'live' if live else 'recorded', 'following_live': self.follow_live, 'input_dropped': self.input_dropped, 'buffer_dropped': self.summary['buffer_dropped'], 'clock_resets': self.summary['clock_resets'], 'anchor_ready': self.model.shift is not None, 'elapsed': self.position, 'duration': self.duration, 'playing': self.playing,
                'counts': counts, 'last_sample_times': last_times, 'scan_visible': bool(scan_visible),
                'state': state, 'valid': bool(valid), 'calibration': calibration,
                'common_xy_translation_m': self.summary['common_xy_translation_m'],
                'processed_bag': self.summary['processed_bag'],
            }, allow_nan=False)))

        def closeEvent(self, event):
            self.timer.stop()
            rospy.signal_shutdown('current-code viewer closed')
            event.accept()

    app = QtWidgets.QApplication(sys.argv)
    window = Viewer()
    window.showMaximized()
    QtCore.QTimer.singleShot(500, window.fit_view)
    window.raise_()
    window.activateWindow()
    print('READY unified RViz', 'live' if live else model.summary['processed_bag'], flush=True)
    return app.exec_()


def main():
    import rospy
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=PACKAGE/'config/localization_viewer.yaml')
    parser.add_argument('--mode', choices=['live','recorded'], default='live')
    parser.add_argument('--processed-bag', type=Path)
    parser.add_argument('--source-bag', type=Path)
    parser.add_argument('--rddf-dir', type=Path)
    parser.add_argument('--seek', type=float)
    parser.add_argument('--check-only', action='store_true')
    args = parser.parse_args(rospy.myargv()[1:])
    try:
        config = yaml.safe_load(args.config.read_text())
        TOPICS.update(config['topics'])
        COLORS.update({key:tuple(value) for key,value in config['colors'].items()})
        if args.seek is not None and not math.isfinite(args.seek): raise ValueError('seek must be finite')
        rddf = args.rddf_dir or Path(config['rddf_directory'])
        if not rddf.is_absolute(): rddf = PACKAGE/rddf
        if config['max_samples_per_topic']<2 or config['refresh_ms']<20: raise ValueError('invalid buffer/refresh limits')
        if args.mode == 'recorded':
            if not args.processed_bag: raise ValueError('--processed-bag is required; no historical bag is selected automatically')
            model = load_data(args.processed_bag,args.source_bag,rddf,config['max_samples_per_topic'])
        else:
            if args.processed_bag or args.source_bag: raise ValueError('bag arguments require --mode recorded')
            model = SceneData(rddf,limit=config['max_samples_per_topic'])
        if args.check_only:
            print(json.dumps(model.summary,indent=2,ensure_ascii=False)); return 0
        return run_gui(model,args.seek,args.mode=='live',config)
    except (OSError,ValueError,RuntimeError,rosbag.ROSBagException) as error:
        print('ERROR:',error,file=sys.stderr); return 1


if __name__ == '__main__':
    sys.exit(main())
