#!/usr/bin/env python3
"""
Robot-side server — runs ON the Agibot X2.
Streams camera via subprocess ros2 calls. No rclpy threading issues.

Deploy: sshpass -p 1 scp robot_server.py agi@IP:~/
Run:    ssh agi@IP "bash -i -c 'python3 ~/robot_server.py &'"
"""

import subprocess
import threading
import time
import json
import sys
import struct
import cv2
import numpy as np
from flask import Flask, jsonify, request, Response

app = Flask(__name__)

latest_frame = None
frame_lock = threading.Lock()
cam_proc = None
cam_lock = threading.Lock()

lidar_points = []
lidar_lock = threading.Lock()
lidar_proc = None


# ── Camera streaming via subprocess ─────────────────────────────

CAM_STREAMER = '''
import rclpy, sys, struct
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import CompressedImage

TOPIC = sys.argv[1]

class S(Node):
    def __init__(self):
        super().__init__("cam_sub")
        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                          history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                          durability=QoSDurabilityPolicy.VOLATILE)
        self.create_subscription(CompressedImage, TOPIC, self.cb, qos)
        self.n = 0
    def cb(self, msg):
        self.n += 1
        if self.n % 2 != 0: return
        d = bytes(msg.data)
        sys.stdout.buffer.write(struct.pack(">I", len(d)))
        sys.stdout.buffer.write(d)
        sys.stdout.buffer.flush()

rclpy.init()
rclpy.spin(S())
'''

LIDAR_STREAMER = '''
import rclpy, sys, json
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import PointCloud2

class L(Node):
    def __init__(self):
        super().__init__("lidar_sub")
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                          history=QoSHistoryPolicy.KEEP_LAST, depth=5,
                          durability=QoSDurabilityPolicy.VOLATILE)
        self.create_subscription(PointCloud2,
            "/aima/hal/sensor/lidar_chest_front/lidar_pointcloud_down_sampling",
            self.cb, qos)
        self.n = 0
    def cb(self, msg):
        self.n += 1
        if self.n % 5 != 0: return
        try:
            step = msg.point_step
            data = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(-1, step)
            if len(data) > 2000:
                idx = np.random.choice(len(data), 2000, replace=False)
                data = data[idx]
            xs = np.frombuffer(data[:, 0:4].tobytes(), dtype=np.float32)
            ys = np.frombuffer(data[:, 4:8].tobytes(), dtype=np.float32)
            zs = np.frombuffer(data[:, 8:12].tobytes(), dtype=np.float32)
            mask = np.isfinite(xs) & np.isfinite(ys) & np.isfinite(zs)
            pts = np.column_stack([xs[mask], ys[mask], zs[mask]])
            out = json.dumps({"p": pts.round(2).tolist()})
            sys.stdout.write(out + "\\n")
            sys.stdout.flush()
        except: pass

rclpy.init()
rclpy.spin(L())
'''


DDS_CONFIG = "/agibot/data/home/agi/.aima/env/ros_dds_configuration.xml"

def get_ros_env():
    """Get environment with ROS + DDS config."""
    import os
    env = os.environ.copy()
    env["FASTRTPS_DEFAULT_PROFILES_FILE"] = DDS_CONFIG
    return env


def write_helper(name, script):
    """Write helper script to /tmp."""
    path = f"/tmp/_{name}.py"
    with open(path, "w") as f:
        f.write(script)
    return path


def start_cam_process(topic):
    global cam_proc, latest_frame
    stop_cam_process()
    latest_frame = None
    script_path = write_helper("cam_sub", CAM_STREAMER)

    # Use bash -i to get full ROS env (PYTHONPATH, LD_LIBRARY_PATH, FASTRTPS, etc.)
    proc = subprocess.Popen(
        ["bash", "-i", "-c", f"python3 -u {script_path} '{topic}'"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    with cam_lock:
        cam_proc = proc

    def reader():
        global latest_frame, cam_proc
        try:
            while proc.poll() is None:
                hdr = proc.stdout.read(4)
                if len(hdr) < 4: break
                length = struct.unpack(">I", hdr)[0]
                if length > 5_000_000 or length == 0: continue
                data = proc.stdout.read(length)
                if len(data) != length: continue

                is_depth = "compressedDepth" in topic
                if is_depth:
                    if len(data) <= 12: continue
                    png = data[12:]
                    img = cv2.imdecode(np.frombuffer(png, np.uint8), cv2.IMREAD_UNCHANGED)
                    if img is None: continue
                    if img.dtype == np.uint16:
                        img = (img / 16).clip(0, 255).astype(np.uint8)
                    elif img.dtype == np.float32:
                        img = (img * 50).clip(0, 255).astype(np.uint8)
                    img = cv2.applyColorMap(img, cv2.COLORMAP_JET)
                    img = cv2.flip(img, -1)
                    ok, jpeg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 75])
                    if not ok: continue
                    frame = jpeg.tobytes()
                else:
                    img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
                    if img is not None:
                        img = cv2.flip(img, -1)
                        ok, jpeg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
                        frame = jpeg.tobytes() if ok else data
                    else:
                        frame = data

                with frame_lock:
                    latest_frame = frame
        except Exception as e:
            print(f"Cam reader error: {e}", flush=True)
        finally:
            with cam_lock:
                if cam_proc == proc:
                    cam_proc = None

    threading.Thread(target=reader, daemon=True).start()


def stop_cam_process():
    global cam_proc
    with cam_lock:
        if cam_proc:
            try: cam_proc.kill()
            except: pass
            cam_proc = None


def start_lidar_process():
    global lidar_proc
    stop_lidar_process()
    script_path = write_helper("lidar_sub", LIDAR_STREAMER)

    proc = subprocess.Popen(
        ["bash", "-i", "-c", f"python3 -u {script_path}"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    lidar_proc = proc

    def reader():
        global lidar_points, lidar_proc
        try:
            for line in iter(proc.stdout.readline, b''):
                try:
                    d = json.loads(line.decode())
                    with lidar_lock:
                        lidar_points = d.get("p", [])
                except: pass
        except: pass
        finally:
            if lidar_proc == proc:
                lidar_proc = None

    threading.Thread(target=reader, daemon=True).start()


def stop_lidar_process():
    global lidar_proc
    if lidar_proc:
        try: lidar_proc.kill()
        except: pass
        lidar_proc = None


# ── Routes ──────────────────────────────────────────────────────

CAMERA_TOPICS = {
    'depth': '/camera/depth/image_raw/compressedDepth',
    'rgbd_front': '/aima/hal/sensor/rgbd_head_front/rgb_image/compressed',
    'stereo_left': '/aima/hal/sensor/stereo_head_front_left/rgb_image/compressed',
    'stereo_right': '/aima/hal/sensor/stereo_head_front_right/rgb_image/compressed',
    'head_front': '/aima/hal/sensor/rgb_head_front_center/rgb_image/compressed',
    'head_rear': '/aima/hal/sensor/rgb_head_rear/rgb_image/compressed',
}


@app.route('/health')
def health():
    return jsonify({"status": "ok", "cam": cam_proc is not None, "lidar": lidar_proc is not None})


@app.route('/camera/start')
def camera_start():
    cam = request.args.get('cam', 'depth')
    topic = CAMERA_TOPICS.get(cam, CAMERA_TOPICS['depth'])
    start_cam_process(topic)
    return jsonify({"status": "started", "topic": topic})


@app.route('/camera/stop')
def camera_stop():
    stop_cam_process()
    return jsonify({"status": "stopped"})


@app.route('/camera/frame')
def camera_frame():
    with frame_lock:
        f = latest_frame
    if f:
        return Response(f, mimetype='image/jpeg')
    return Response(b'', status=204)


@app.route('/camera/stream')
def camera_stream():
    def gen():
        while True:
            with frame_lock:
                f = latest_frame
            if f:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + f + b'\r\n')
            time.sleep(0.066)
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/lidar/start')
def lidar_start():
    start_lidar_process()
    return jsonify({"status": "started"})


@app.route('/lidar/stop')
def lidar_stop():
    stop_lidar_process()
    return jsonify({"status": "stopped"})


@app.route('/lidar/points')
def lidar_points_route():
    with lidar_lock:
        pts = lidar_points
    return jsonify({"points": pts, "count": len(pts)})


if __name__ == '__main__':
    print("\n  Robot Bridge Server (subprocess mode)")
    print("  http://0.0.0.0:8080\n", flush=True)
    from werkzeug.serving import run_simple
    run_simple('0.0.0.0', 8080, app, threaded=True, use_reloader=False)
