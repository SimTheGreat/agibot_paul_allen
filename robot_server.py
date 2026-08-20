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
import os
import struct
import cv2
import numpy as np
from flask import Flask, jsonify, request, Response

app = Flask(__name__)

latest_frame = None
frame_lock = threading.Lock()
cam_proc = None
cam_lock = threading.Lock()
cam_active = False
cam_current_topic = None

# Multi-camera support
cam_frames = {}   # cam_key -> jpeg bytes
cam_frames_lock = threading.Lock()
cam_procs = {}    # cam_key -> subprocess
cam_procs_lock = threading.Lock()

lidar_points = []
lidar_lock = threading.Lock()
lidar_proc = None


# ── Camera streaming via subprocess ─────────────────────────────

CAM_STREAMER = '''
import rclpy, sys, struct, cv2, numpy as np, os
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

TOPIC = sys.argv[1]
IS_RAW = "/compressed" not in TOPIC and "compressedDepth" not in TOPIC
NODE_NAME = "cam_sub_" + str(os.getpid())

class S(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        msg_type = Image if IS_RAW else CompressedImage
        # Match publisher QoS: RELIABLE + TRANSIENT_LOCAL (primary)
        # Also try RELIABLE + VOLATILE as fallback
        for dur in [QoSDurabilityPolicy.TRANSIENT_LOCAL, QoSDurabilityPolicy.VOLATILE]:
            qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                              history=QoSHistoryPolicy.KEEP_LAST, depth=5,
                              durability=dur)
            self.create_subscription(msg_type, TOPIC, self.cb, qos)
        self.n = 0
        sys.stderr.write(f"Subscribing to {TOPIC} as {'Image' if IS_RAW else 'CompressedImage'}\\n")
        sys.stderr.flush()

    def cb(self, msg):
        self.n += 1
        if self.n % 2 != 0: return
        try:
            if IS_RAW:
                # Convert raw Image to JPEG
                h, w = msg.height, msg.width
                enc = msg.encoding
                data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
                if enc == "rgb8":
                    img = data.reshape(h, w, 3)
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                elif enc == "bgr8":
                    img = data.reshape(h, w, 3)
                elif enc == "mono8":
                    img = data.reshape(h, w)
                elif enc == "bgra8":
                    img = data.reshape(h, w, 4)
                    img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
                elif enc == "rgba8":
                    img = data.reshape(h, w, 4)
                    img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
                else:
                    img = data.reshape(h, w, -1)
                ok, jpeg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 75])
                if not ok: return
                d = jpeg.tobytes()
            else:
                raw = bytes(msg.data)
                # Check if it's valid JPEG (starts with FFD8)
                if len(raw) > 2 and raw[0] == 0xFF and raw[1] == 0xD8:
                    d = raw
                else:
                    # Non-JPEG compressed: decode and re-encode
                    img = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
                    if img is None: return
                    ok, jpeg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 75])
                    if not ok: return
                    d = jpeg.tobytes()
            sys.stdout.buffer.write(struct.pack(">I", len(d)))
            sys.stdout.buffer.write(d)
            sys.stdout.buffer.flush()
        except Exception as e:
            sys.stderr.write(f"Frame error: {e}\\n")
            sys.stderr.flush()

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


IMU_STREAMER = '''
import rclpy, sys, json
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Imu

class I(Node):
    def __init__(self):
        super().__init__("imu_sub")
        for rel in [QoSReliabilityPolicy.BEST_EFFORT, QoSReliabilityPolicy.RELIABLE]:
            qos = QoSProfile(reliability=rel, history=QoSHistoryPolicy.KEEP_LAST,
                              depth=1, durability=QoSDurabilityPolicy.VOLATILE)
            self.create_subscription(Imu, "/aima/hal/imu/chest/state", self.cb, qos)
            self.create_subscription(Imu, "/aima/hal/imu/torso/state", self.cb_torso, qos)
        self.n = 0
    def cb(self, msg):
        self.n += 1
        if self.n % 5 != 0: return
        self._send("chest", msg)
    def cb_torso(self, msg):
        self.n += 1
        if self.n % 5 != 0: return
        self._send("torso", msg)
    def _send(self, src, msg):
        try:
            d = {
                "src": src,
                "o": [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w],
                "av": [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
                "la": [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
            }
            sys.stdout.write(json.dumps(d) + "\\n")
            sys.stdout.flush()
        except: pass

rclpy.init()
rclpy.spin(I())
'''

# Topics that need 180-degree flip
FLIP_TOPICS = {
    '/camera/depth/image_raw/compressedDepth',
    '/aima/hal/sensor/rgb_head_front_center/rgb_image',
    '/aima/hal/sensor/rgb_head_front_center/rgb_image/compressed',
    '/aima/hal/sensor/rgb_head_rear/rgb_image/compressed',
}

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
    global cam_proc, latest_frame, cam_active, cam_current_topic
    # If same topic subprocess is already running, just resume
    if cam_proc is not None and cam_proc.poll() is None and cam_current_topic == topic:
        cam_active = True
        return
    # Different topic or no subprocess — kill old, start new
    kill_cam_process()
    latest_frame = None
    cam_current_topic = topic
    cam_active = True
    script_path = write_helper("cam_sub", CAM_STREAMER)

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

                if not cam_active:
                    continue  # paused, keep reading but don't update frame

                needs_flip = topic in FLIP_TOPICS
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
                    if needs_flip:
                        img = cv2.rotate(img, cv2.ROTATE_180)
                    ok, jpeg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 75])
                    if not ok: continue
                    frame = jpeg.tobytes()
                elif needs_flip:
                    img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
                    if img is None:
                        frame = data
                    else:
                        img = cv2.rotate(img, cv2.ROTATE_180)
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
                    cam_current_topic = None

    threading.Thread(target=reader, daemon=True).start()


def pause_cam():
    global cam_active, latest_frame
    cam_active = False
    latest_frame = None


def kill_cam_process():
    global cam_proc, cam_current_topic
    with cam_lock:
        if cam_proc:
            try:
                # Kill all children then parent
                import signal
                pid = cam_proc.pid
                subprocess.run(["pkill", "-9", "-P", str(pid)], timeout=3)
                cam_proc.kill()
            except: pass
            cam_proc = None
            cam_current_topic = None


def start_multi_cam(cam_key, topic):
    """Start a camera subprocess for a specific cam key."""
    stop_multi_cam(cam_key)
    script_path = write_helper(f"cam_{cam_key}", CAM_STREAMER)
    env = get_ros_env()
    proc = subprocess.Popen(
        ["python3", "-u", script_path, topic],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env)
    with cam_procs_lock:
        cam_procs[cam_key] = proc

    def reader():
        try:
            while proc.poll() is None:
                hdr = proc.stdout.read(4)
                if len(hdr) < 4: break
                length = struct.unpack(">I", hdr)[0]
                if length > 5_000_000 or length == 0: continue
                data = proc.stdout.read(length)
                if len(data) != length: continue
                needs_flip = topic in FLIP_TOPICS
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
                    if needs_flip:
                        img = cv2.rotate(img, cv2.ROTATE_180)
                    ok, jpeg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 75])
                    if not ok: continue
                    frame = jpeg.tobytes()
                elif needs_flip:
                    img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
                    if img is None:
                        frame = data
                    else:
                        img = cv2.rotate(img, cv2.ROTATE_180)
                        ok, jpeg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
                        frame = jpeg.tobytes() if ok else data
                else:
                    frame = data
                with cam_frames_lock:
                    cam_frames[cam_key] = frame
        except Exception as e:
            print(f"Multi-cam reader {cam_key} error: {e}", flush=True)
        finally:
            with cam_procs_lock:
                if cam_procs.get(cam_key) == proc:
                    del cam_procs[cam_key]

    threading.Thread(target=reader, daemon=True).start()


def stop_multi_cam(cam_key):
    with cam_procs_lock:
        proc = cam_procs.pop(cam_key, None)
    if proc:
        try:
            subprocess.run(["pkill", "-9", "-P", str(proc.pid)], timeout=3)
            proc.kill()
        except: pass
    with cam_frames_lock:
        cam_frames.pop(cam_key, None)


def stop_all_cams():
    with cam_procs_lock:
        keys = list(cam_procs.keys())
    for k in keys:
        stop_multi_cam(k)


def start_lidar_process():
    global lidar_proc
    stop_lidar_process()
    script_path = write_helper("lidar_sub", LIDAR_STREAMER)

    proc = subprocess.Popen(
        ["bash", "-i", "-c", f"python3 -u {script_path}"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        )
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
        try:
            subprocess.run(["pkill", "-9", "-P", str(lidar_proc.pid)], timeout=3)
            lidar_proc.kill()
        except: pass
        lidar_proc = None


# ── IMU streaming ───────────────────────────────────────────────

imu_data = {}
imu_lock = threading.Lock()
imu_proc = None


def start_imu_process():
    global imu_proc
    stop_imu_process()
    script_path = write_helper("imu_sub", IMU_STREAMER)
    proc = subprocess.Popen(
        ["bash", "-i", "-c", f"python3 -u {script_path}"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        )
    imu_proc = proc

    def reader():
        global imu_data, imu_proc
        try:
            for line in iter(proc.stdout.readline, b''):
                try:
                    d = json.loads(line.decode())
                    with imu_lock:
                        imu_data = d
                except: pass
        except: pass
        finally:
            if imu_proc == proc:
                imu_proc = None
    threading.Thread(target=reader, daemon=True).start()


def stop_imu_process():
    global imu_proc
    if imu_proc:
        try:
            subprocess.run(["pkill", "-9", "-P", str(imu_proc.pid)], timeout=3)
            imu_proc.kill()
        except: pass
        imu_proc = None


# ── Routes ──────────────────────────────────────────────────────

CAMERA_TOPICS = {
    'head_front': '/aima/hal/sensor/rgb_head_front_center/rgb_image/compressed',
    'depth': '/camera/depth/image_raw/compressedDepth',
    'rgbd_front': '/aima/hal/sensor/rgbd_head_front/rgb_image/compressed',
    'stereo_left': '/aima/hal/sensor/stereo_head_front_left/rgb_image/compressed',
    'stereo_right': '/aima/hal/sensor/stereo_head_front_right/rgb_image/compressed',
    'head_rear': '/aima/hal/sensor/rgb_head_rear/rgb_image/compressed',
}


@app.route('/health')
def health():
    return jsonify({"status": "ok", "cam": cam_proc is not None, "lidar": lidar_proc is not None})


@app.route('/camera/start')
def camera_start():
    cam = request.args.get('cam', 'head_front')
    topic = CAMERA_TOPICS.get(cam, CAMERA_TOPICS['head_front'])
    reused = (cam_proc is not None and cam_proc.poll() is None and cam_current_topic == topic)
    start_cam_process(topic)
    return jsonify({"status": "resumed" if reused else "started", "topic": topic})


@app.route('/camera/stop')
def camera_stop():
    pause_cam()  # pause, don't kill — instant restart
    return jsonify({"status": "paused"})


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


@app.route('/cameras/start_all')
def cameras_start_all():
    """Start all cameras simultaneously."""
    started = []
    for key, topic in CAMERA_TOPICS.items():
        start_multi_cam(key, topic)
        started.append(key)
    return jsonify({"status": "started", "cameras": started})


@app.route('/cameras/stop_all')
def cameras_stop_all():
    stop_all_cams()
    return jsonify({"status": "stopped"})


@app.route('/cameras/frame/<cam_key>')
def cameras_frame(cam_key):
    with cam_frames_lock:
        f = cam_frames.get(cam_key)
    if f:
        return Response(f, mimetype='image/jpeg')
    return Response(b'', status=204)


@app.route('/cameras/status')
def cameras_status():
    with cam_procs_lock:
        active = {k: p.poll() is None for k, p in cam_procs.items()}
    with cam_frames_lock:
        has_frame = {k: True for k in cam_frames}
    return jsonify({"active": active, "has_frame": has_frame})


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


@app.route('/imu/start')
def imu_start():
    start_imu_process()
    return jsonify({"status": "started"})


@app.route('/imu/stop')
def imu_stop():
    stop_imu_process()
    return jsonify({"status": "stopped"})


@app.route('/imu/data')
def imu_get_data():
    with imu_lock:
        d = imu_data.copy()
    return jsonify(d)


# ── TTS via piper + SOC2 speaker ────────────────────────────────

PIPER_BIN = os.path.expanduser("~/.local/bin/piper")
PIPER_MODEL = os.path.expanduser("~/stage_v3/data/piper_voices/en_US-hfc_female-medium.onnx")
SOC2_HOST = "agi@10.0.1.42"
SOC2_ALSA = "plug:playback_def"


@app.route('/tts', methods=['POST'])
def tts():
    text = (request.json or {}).get('text', 'Hello')
    text = text.replace('"', '').replace("'", "").replace('\\', '')
    wav = "/tmp/_tts.wav"
    try:
        # Generate WAV with piper
        subprocess.run(
            f'echo "{text}" | {PIPER_BIN} -m {PIPER_MODEL} --output_file {wav}',
            shell=True, capture_output=True, timeout=15)
        if not os.path.exists(wav):
            return jsonify({"error": "piper failed"}), 500
        # Forward to play_bridge's TTS HTTP endpoint (port 8081)
        # play_bridge runs as systemd service with SOC2 network access + ElevenLabs
        import urllib.request
        data = json.dumps({"text": text}).encode()
        req = urllib.request.Request(
            "http://127.0.0.1:8081/", data=data,
            headers={"Content-Type": "application/json"})
        resp = urllib.request.urlopen(req, timeout=30)
        result = json.loads(resp.read().decode())
        if result.get("status") == "ok":
            return jsonify({"status": "ok", "text": text})
        return jsonify({"error": result.get("error", "unknown")}), 500
    except subprocess.TimeoutExpired:
        return jsonify({"error": "timeout"}), 500
    except Exception as e:
        return jsonify({"error": str(e)}), 500


if __name__ == '__main__':
    print("\n  Robot Bridge Server (subprocess mode)")
    print("  http://0.0.0.0:8080\n", flush=True)
    from werkzeug.serving import run_simple
    run_simple('0.0.0.0', 8080, app, threaded=True, use_reloader=False)
