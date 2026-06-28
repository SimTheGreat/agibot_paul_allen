#!/usr/bin/env python3
"""
Agibot X2 Advanced Dashboard
- Robot controls (emoji, motion, TTS, walk, mode)
- 3D robot simulator
- LiDAR point cloud map viewer
- Workflow builder
- n8n webhook connector
- Camera feed

Usage: python3 app.py
Then open http://localhost:5000
"""

import subprocess
import threading
import struct
import time
import base64
import json
import os
import sys
import io
import cv2
import numpy as np
from pathlib import Path
from flask import Flask, render_template, jsonify, request, Response, send_from_directory

app = Flask(__name__, template_folder='templates', static_folder='static')

# ── AI Vision ───────────────────────────────────────────────────
yolo_model = None
yolo_enabled = False
face_rec_enabled = False
known_faces = {}  # name -> encoding
FACES_DIR = Path(__file__).parent / "known_faces"

def load_yolo():
    global yolo_model
    if yolo_model is None:
        try:
            from ultralytics import YOLO
            yolo_model = YOLO("yolo11n.pt")
            print("  YOLO model loaded.")
        except Exception as e:
            print(f"  YOLO load failed: {e}")

def load_known_faces():
    global known_faces
    FACES_DIR.mkdir(exist_ok=True)
    import face_recognition
    for f in FACES_DIR.glob("*.jpg"):
        img = face_recognition.load_image_file(str(f))
        encs = face_recognition.face_encodings(img)
        if encs:
            known_faces[f.stem] = encs[0]
            print(f"  Face loaded: {f.stem}")
    for f in FACES_DIR.glob("*.png"):
        img = face_recognition.load_image_file(str(f))
        encs = face_recognition.face_encodings(img)
        if encs:
            known_faces[f.stem] = encs[0]
            print(f"  Face loaded: {f.stem}")

def flip_and_resize(jpeg_bytes):
    """Flip 180 and resize if needed."""
    img = cv2.imdecode(np.frombuffer(jpeg_bytes, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        return jpeg_bytes
    img = cv2.rotate(img, cv2.ROTATE_180)
    h, w = img.shape[:2]
    if w > 1920:
        scale = 1920 / w
        img = cv2.resize(img, (1920, int(h * scale)))
    ok, out = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
    return out.tobytes() if ok else jpeg_bytes


def process_frame_ai(jpeg_bytes):
    """Run YOLO and/or face recognition on a frame, return annotated JPEG."""
    img = cv2.imdecode(np.frombuffer(jpeg_bytes, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        return jpeg_bytes
    # Always flip 180 (cameras are mounted upside down)
    img = cv2.rotate(img, cv2.ROTATE_180)
    h, w = img.shape[:2]
    if w > 1920:
        scale = 1920 / w
        img = cv2.resize(img, (1920, int(h * scale)))

    if yolo_enabled and yolo_model:
        results = yolo_model(img, verbose=False, conf=0.4)
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                label = f"{yolo_model.names[cls]} {conf:.1%}"
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(img, label, (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    if face_rec_enabled and known_faces:
        import face_recognition
        small = cv2.resize(img, (0, 0), fx=0.25, fy=0.25)
        rgb_small = cv2.cvtColor(small, cv2.COLOR_BGR2RGB)
        locs = face_recognition.face_locations(rgb_small)
        encs = face_recognition.face_encodings(rgb_small, locs)
        known_encs = list(known_faces.values())
        known_names = list(known_faces.keys())
        for (top, right, bottom, left), enc in zip(locs, encs):
            top *= 4; right *= 4; bottom *= 4; left *= 4
            matches = face_recognition.compare_faces(known_encs, enc, tolerance=0.5)
            name = "Unknown"
            if True in matches:
                dists = face_recognition.face_distance(known_encs, enc)
                best = np.argmin(dists)
                if matches[best]:
                    name = known_names[best]
            color = (0, 200, 255) if name != "Unknown" else (0, 0, 255)
            cv2.rectangle(img, (left, top), (right, bottom), color, 2)
            cv2.putText(img, name, (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

    ok, out = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
    return out.tobytes() if ok else jpeg_bytes

# ── Config ──────────────────────────────────────────────────────
ROBOT_HOST = "agi@10.104.218.77"
ROBOT_PASS = "1"
ROBOT_IP = "10.104.218.77"
SETUP_CMD = "source ~/Botifull/SLAM_stack/scripts/setup_env.sh && export FASTRTPS_DEFAULT_PROFILES_FILE=/agibot/data/home/agi/.aima/env/ros_dds_configuration.xml"
SSH_SOCKET = "/tmp/agibot_ssh_ctl"
N8N_WEBHOOK_URL = os.environ.get("N8N_WEBHOOK_URL", "")
ROBOT_SERVER = f"http://{ROBOT_IP}:8080"  # robot_server.py running on robot

# ── State ───────────────────────────────────────────────────────
camera_process = None
camera_lock = threading.Lock()
latest_frame = None
frame_lock = threading.Lock()
walk_process = None
walk_lock = threading.Lock()
walk_fwd = 0.0
walk_ang = 0.0
walk_lat = 0.0
walk_registered = False

# LiDAR map data (accumulated point cloud)
lidar_points = []
lidar_lock = threading.Lock()
lidar_process = None

# Workflows storage
WORKFLOW_FILE = Path(__file__).parent / "workflows.json"

SSH_BASE = [
    "sshpass", "-p", ROBOT_PASS, "ssh",
    "-o", "StrictHostKeyChecking=no",
    "-o", "ConnectTimeout=10",
    "-o", f"ControlPath={SSH_SOCKET}",
    "-o", "ControlMaster=auto",
    "-o", "ControlPersist=300",
    ROBOT_HOST
]


def ensure_ssh_master():
    try:
        r = subprocess.run(
            ["ssh", "-o", f"ControlPath={SSH_SOCKET}", "-O", "check", ROBOT_HOST],
            capture_output=True, timeout=5)
        if r.returncode == 0:
            return True
    except Exception:
        pass
    try:
        subprocess.Popen(
            ["sshpass", "-p", ROBOT_PASS, "ssh",
             "-o", "StrictHostKeyChecking=no",
             "-o", f"ControlPath={SSH_SOCKET}",
             "-o", "ControlMaster=yes",
             "-o", "ControlPersist=300",
             "-N", ROBOT_HOST],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(1)
        print("  SSH master connection established.")
        return True
    except Exception as e:
        print(f"  SSH master failed: {e}")
        return False


def ssh_cmd(cmd, timeout=30):
    full = f'{SETUP_CMD} >/dev/null 2>&1 && {cmd}'
    try:
        r = subprocess.run(
            SSH_BASE + [full],
            capture_output=True, text=True, timeout=timeout)
        out = r.stdout.strip()
        err = r.stderr.strip()
        err_lines = [l for l in err.splitlines()
                     if not any(x in l for x in ['RTPS_WRITER', 'RTPS_READER', 'Warning', 'matched_'])]
        err_clean = '\n'.join(err_lines).strip()
        result = out
        if err_clean:
            result += '\n' + err_clean if result else err_clean
        return result if result else "(no output)"
    except subprocess.TimeoutExpired:
        return "ERROR: Command timed out"
    except Exception as e:
        return f"ERROR: {e}"


# ── Camera ──────────────────────────────────────────────────────

CAMERA_TOPICS = {
    'head_front': '/aima/hal/sensor/rgb_head_front_center/rgb_image/compressed',
    'depth': '/camera/depth/image_raw/compressedDepth',
    'rgbd_front': '/aima/hal/sensor/rgbd_head_front/rgb_image/compressed',
    'stereo_left': '/aima/hal/sensor/stereo_head_front_left/rgb_image/compressed',
    'stereo_right': '/aima/hal/sensor/stereo_head_front_right/rgb_image/compressed',
    'head_rear': '/aima/hal/sensor/rgb_head_rear/rgb_image/compressed',
}

CAMERA_SCRIPT = '''
import rclpy, sys, struct, cv2, numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import CompressedImage

TOPIC = "{topic}"
IS_DEPTH = "compressedDepth" in TOPIC

class S(Node):
    def __init__(self):
        super().__init__("cam_stream")
        for rel in [QoSReliabilityPolicy.RELIABLE, QoSReliabilityPolicy.BEST_EFFORT]:
            qos = QoSProfile(reliability=rel, history=QoSHistoryPolicy.KEEP_LAST,
                              depth=1, durability=QoSDurabilityPolicy.VOLATILE)
            self.create_subscription(CompressedImage, TOPIC, self.cb, qos)
        self.n = 0
        sys.stderr.write("Camera ready on " + TOPIC + "\\n"); sys.stderr.flush()

    def cb(self, msg):
        self.n += 1
        if self.n % 3 != 0: return
        raw = bytes(msg.data)
        try:
            if IS_DEPTH:
                # compressedDepth has a 12-byte header before the PNG/RVL data
                if len(raw) <= 12: return
                png_data = raw[12:]
                img = cv2.imdecode(np.frombuffer(png_data, np.uint8), cv2.IMREAD_UNCHANGED)
                if img is None: return
                # Normalize depth to 8-bit for visualization
                if img.dtype == np.uint16:
                    img = (img / 16).clip(0, 255).astype(np.uint8)
                elif img.dtype == np.float32:
                    img = (img * 50).clip(0, 255).astype(np.uint8)
                img = cv2.applyColorMap(img, cv2.COLORMAP_JET)
                ok, jpeg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if not ok: return
                d = jpeg.tobytes()
            else:
                d = raw
            sys.stdout.buffer.write(struct.pack(">I", len(d)))
            sys.stdout.buffer.write(d)
            sys.stdout.buffer.flush()
        except Exception as e:
            sys.stderr.write(f"Frame error: {e}\\n"); sys.stderr.flush()

rclpy.init()
node = S()
try: rclpy.spin(node)
except: pass
finally:
    node.destroy_node(); rclpy.shutdown()
'''


def camera_stream_worker(topic):
    global camera_process, latest_frame
    script = CAMERA_SCRIPT.replace("{topic}", topic)
    encoded = base64.b64encode(script.encode()).decode()
    cmd = f'{SETUP_CMD} >/dev/null 2>&1; echo {encoded} | base64 -d > /tmp/_cam.py && exec python3 -u /tmp/_cam.py'
    try:
        proc = subprocess.Popen(SSH_BASE + [cmd], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        with camera_lock:
            camera_process = proc
        while proc.poll() is None:
            header = proc.stdout.read(4)
            if len(header) < 4: break
            length = struct.unpack('>I', header)[0]
            if length > 5_000_000 or length == 0: continue
            data = proc.stdout.read(length)
            if len(data) == length:
                with frame_lock:
                    latest_frame = data
    except Exception as e:
        print(f"Camera error: {e}")
    finally:
        with camera_lock:
            if camera_process == proc: camera_process = None


def start_camera(key='rgbd_front'):
    global latest_frame
    stop_camera()
    time.sleep(0.3)
    latest_frame = None
    topic = CAMERA_TOPICS.get(key, CAMERA_TOPICS['rgbd_front'])
    threading.Thread(target=camera_stream_worker, args=(topic,), daemon=True).start()


def stop_camera():
    global camera_process
    with camera_lock:
        if camera_process:
            try: camera_process.kill()
            except: pass
            camera_process = None


# ── Walk ────────────────────────────────────────────────────────

WALK_SCRIPT = '''
import rclpy, sys, json, select, time
from rclpy.node import Node
from aimdk_msgs.msg import McLocomotionVelocity, MessageHeader
from aimdk_msgs.srv import SetMcInputSource

class W(Node):
    def __init__(self):
        super().__init__("walk_ctrl")
        self.pub = self.create_publisher(McLocomotionVelocity, "/aima/mc/locomotion/velocity", 10)
        self.cli = self.create_client(SetMcInputSource, "/aimdk_5Fmsgs/srv/SetMcInputSource")
        self.fwd = 0.0; self.ang = 0.0; self.lat = 0.0
        self.register()
        self.create_timer(0.02, self.tick)
        self.create_timer(0.05, self.read_stdin)
    def register(self):
        self.cli.wait_for_service(timeout_sec=10.0)
        rq = SetMcInputSource.Request()
        rq.action.value = 1001; rq.input_source.name = "node"
        rq.input_source.priority = 40; rq.input_source.timeout = 1000
        for i in range(8):
            rq.request.header.stamp = self.get_clock().now().to_msg()
            ft = self.cli.call_async(rq)
            rclpy.spin_until_future_complete(self, ft, timeout_sec=0.5)
            if ft.done():
                sys.stderr.write("REGISTERED\\n"); sys.stderr.flush(); break
    def tick(self):
        m = McLocomotionVelocity(); m.header = MessageHeader()
        m.header.stamp = self.get_clock().now().to_msg(); m.source = "node"
        m.forward_velocity = self.fwd; m.lateral_velocity = self.lat; m.angular_velocity = self.ang
        self.pub.publish(m)
    def read_stdin(self):
        try:
            while select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline()
                if not line or line.strip() == "QUIT":
                    self.fwd = 0; self.ang = 0; self.lat = 0; self.tick()
                    time.sleep(0.1); rclpy.shutdown(); return
                try:
                    d = json.loads(line)
                    self.fwd = float(d.get("f", 0)); self.ang = float(d.get("a", 0)); self.lat = float(d.get("l", 0))
                except: pass
        except: pass

rclpy.init()
node = W()
try: rclpy.spin(node)
except: pass
finally:
    node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()
'''


def start_walk():
    global walk_process, walk_registered, walk_fwd, walk_ang, walk_lat
    stop_walk()
    walk_fwd = walk_ang = walk_lat = 0.0
    walk_registered = False
    encoded = base64.b64encode(WALK_SCRIPT.encode()).decode()
    cmd = f'{SETUP_CMD} >/dev/null 2>&1; echo {encoded} | base64 -d > /tmp/_walk.py && exec python3 -u /tmp/_walk.py'
    proc = subprocess.Popen(SSH_BASE + [cmd], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    with walk_lock:
        walk_process = proc

    def wait_reg():
        global walk_registered
        try:
            while proc.poll() is None:
                line = proc.stderr.readline()
                if b'REGISTERED' in line:
                    walk_registered = True; print("Walk: registered"); break
        except: pass
    threading.Thread(target=wait_reg, daemon=True).start()


def stop_walk():
    global walk_process, walk_registered
    with walk_lock:
        if walk_process:
            try: walk_process.stdin.write(b'QUIT\n'); walk_process.stdin.flush()
            except: pass
            try: walk_process.wait(timeout=3)
            except:
                try: walk_process.kill()
                except: pass
            walk_process = None
    walk_registered = False


def send_walk_velocity(fwd, ang, lat=0.0):
    global walk_fwd, walk_ang, walk_lat
    walk_fwd, walk_ang, walk_lat = fwd, ang, lat
    with walk_lock:
        if walk_process and walk_process.poll() is None:
            try:
                walk_process.stdin.write((json.dumps({"f": fwd, "a": ang, "l": lat}) + "\n").encode())
                walk_process.stdin.flush()
            except: pass


# ── LiDAR ───────────────────────────────────────────────────────

LIDAR_SCRIPT = '''
import rclpy, sys, json, struct, time
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import PointCloud2
import numpy as np

class L(Node):
    def __init__(self):
        super().__init__("lidar_stream")
        for rel in [QoSReliabilityPolicy.RELIABLE, QoSReliabilityPolicy.BEST_EFFORT]:
            qos = QoSProfile(reliability=rel, history=QoSHistoryPolicy.KEEP_LAST,
                              depth=1, durability=QoSDurabilityPolicy.VOLATILE)
            self.create_subscription(PointCloud2, "/aima/hal/sensor/lidar_chest_front/lidar_pointcloud", self.cb, qos)
        self.n = 0
        sys.stderr.write("LiDAR ready\\n"); sys.stderr.flush()
    def cb(self, msg):
        self.n += 1
        if self.n % 5 != 0: return
        # Extract XYZ from pointcloud - assume float32 x,y,z at offset 0,4,8
        step = msg.point_step
        data = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(-1, step)
        # Downsample to max 2000 points
        if len(data) > 2000:
            idx = np.random.choice(len(data), 2000, replace=False)
            data = data[idx]
        xs = np.frombuffer(data[:, 0:4].tobytes(), dtype=np.float32)
        ys = np.frombuffer(data[:, 4:8].tobytes(), dtype=np.float32)
        zs = np.frombuffer(data[:, 8:12].tobytes(), dtype=np.float32)
        # Filter out NaN/Inf
        mask = np.isfinite(xs) & np.isfinite(ys) & np.isfinite(zs)
        pts = np.column_stack([xs[mask], ys[mask], zs[mask]])
        # Send as JSON line
        out = json.dumps({"pts": pts.tolist()})
        sys.stdout.write(out + "\\n")
        sys.stdout.flush()

rclpy.init()
node = L()
try: rclpy.spin(node)
except: pass
finally:
    node.destroy_node(); rclpy.shutdown()
'''


def lidar_stream_worker():
    global lidar_process, lidar_points
    encoded = base64.b64encode(LIDAR_SCRIPT.encode()).decode()
    cmd = f'{SETUP_CMD} >/dev/null 2>&1; echo {encoded} | base64 -d > /tmp/_lidar.py && exec python3 -u /tmp/_lidar.py'
    try:
        proc = subprocess.Popen(SSH_BASE + [cmd], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        lidar_process = proc
        for line in iter(proc.stdout.readline, b''):
            try:
                d = json.loads(line.decode())
                with lidar_lock:
                    lidar_points = d.get("pts", [])
            except:
                pass
    except Exception as e:
        print(f"LiDAR error: {e}")
    finally:
        lidar_process = None


def start_lidar():
    global lidar_process
    if lidar_process and lidar_process.poll() is None:
        return
    threading.Thread(target=lidar_stream_worker, daemon=True).start()


def stop_lidar():
    global lidar_process
    if lidar_process:
        try: lidar_process.kill()
        except: pass
        lidar_process = None


# ── Workflows ───────────────────────────────────────────────────

def load_workflows():
    if WORKFLOW_FILE.exists():
        return json.loads(WORKFLOW_FILE.read_text())
    return []


def save_workflows(workflows):
    WORKFLOW_FILE.write_text(json.dumps(workflows, indent=2))


def execute_workflow(workflow):
    """Execute a workflow: list of steps with type + params."""
    results = []
    for step in workflow.get("steps", []):
        stype = step.get("type", "")
        params = step.get("params", {})
        delay = step.get("delay", 0)
        if delay:
            time.sleep(delay)

        if stype == "emoji":
            r = ssh_cmd(f"ros2 service call /face_ui_proxy/play_emoji aimdk_msgs/srv/PlayEmoji "
                        f"'{{emotion_id: {params.get('id', 90)}, mode: 1, priority: 10}}'")
        elif stype == "motion":
            mid, aid = params.get("motion_id", 1002), params.get("area_id", 1)
            r = ssh_cmd(f"ros2 service call /aimdk_5Fmsgs/srv/SetMcPresetMotion "
                        f"aimdk_msgs/srv/SetMcPresetMotion "
                        f"'{{motion: {{value: {mid}}}, area: {{value: {aid}}}, interrupt: false}}'")
        elif stype == "tts":
            text = params.get("text", "Hello").replace("'", "").replace('"', '').replace('\\', '')
            encoded = base64.b64encode(TTS_SCRIPT.encode()).decode()
            r = ssh_cmd(
                f'{SETUP_CMD} >/dev/null 2>&1; echo {encoded} | base64 -d > /tmp/_tts_play.py && python3 /tmp/_tts_play.py "{text}"',
                timeout=25)
        elif stype == "mode":
            r = ssh_cmd(f"ros2 service call /aimdk_5Fmsgs/srv/SetMcAction "
                        f"aimdk_msgs/srv/SetMcAction "
                        f"'{{command: {{action_desc: \"{params.get('action', 'STAND_DEFAULT')}\"}}}}'")
        elif stype == "wait":
            time.sleep(params.get("seconds", 1))
            r = f"Waited {params.get('seconds', 1)}s"
        elif stype == "n8n_webhook":
            r = trigger_n8n(params.get("url", ""), params.get("payload", {}))
        elif stype == "shell":
            r = ssh_cmd(params.get("cmd", "echo ok"))
        else:
            r = f"Unknown step type: {stype}"
        results.append({"step": step, "result": r})
    return results


# ── n8n ─────────────────────────────────────────────────────────

def trigger_n8n(webhook_url, payload=None):
    """Send a webhook to n8n."""
    if not webhook_url:
        return "ERROR: No webhook URL"
    try:
        import urllib.request
        data = json.dumps(payload or {}).encode()
        req = urllib.request.Request(webhook_url, data=data,
                                     headers={"Content-Type": "application/json"})
        resp = urllib.request.urlopen(req, timeout=10)
        return f"n8n response: {resp.status} {resp.read().decode()[:200]}"
    except Exception as e:
        return f"n8n error: {e}"


# ── Routes ──────────────────────────────────────────────────────

@app.route('/')
def index():
    return render_template('index.html')


@app.route('/api/cmd', methods=['POST'])
def api_cmd():
    return jsonify({"result": ssh_cmd(request.json.get('cmd', ''))})


@app.route('/api/emoji/<int:eid>')
def api_emoji(eid):
    return jsonify({"result": ssh_cmd(
        f"ros2 service call /face_ui_proxy/play_emoji aimdk_msgs/srv/PlayEmoji "
        f"'{{emotion_id: {eid}, mode: 1, priority: 10}}'")})


@app.route('/api/motion/<action>')
def api_motion(action):
    m = {'wave_left': (1002,1), 'wave_right': (1002,2), 'handshake_left': (1003,1),
         'handshake_right': (1003,2), 'raise_left': (1001,1), 'raise_right': (1001,2),
         'airkiss_left': (1004,1), 'airkiss_right': (1004,2)}
    if action not in m: return jsonify({"error": "unknown"}), 400
    mid, aid = m[action]
    return jsonify({"result": ssh_cmd(
        f"ros2 service call /aimdk_5Fmsgs/srv/SetMcPresetMotion aimdk_msgs/srv/SetMcPresetMotion "
        f"'{{motion: {{value: {mid}}}, area: {{value: {aid}}}, interrupt: false}}'")})


@app.route('/api/preset_motion', methods=['POST'])
def api_preset_motion():
    d = request.json
    mid = d.get('motion_id', 1002)
    aid = d.get('area_id', 1)
    return jsonify({"result": ssh_cmd(
        f"ros2 service call /aimdk_5Fmsgs/srv/SetMcPresetMotion aimdk_msgs/srv/SetMcPresetMotion "
        f"'{{motion: {{value: {mid}}}, area: {{value: {aid}}}, interrupt: false}}'")})


@app.route('/api/mode/<mode>')
def api_mode(mode):
    modes = {'stand':'STAND_DEFAULT','locomotion':'LOCOMOTION_DEFAULT',
             'passive':'PASSIVE_DEFAULT','damping':'DAMPING_DEFAULT','joint':'JOINT_DEFAULT'}
    if mode not in modes: return jsonify({"error": "unknown"}), 400
    return jsonify({"result": ssh_cmd(
        f"ros2 service call /aimdk_5Fmsgs/srv/SetMcAction aimdk_msgs/srv/SetMcAction "
        f"'{{command: {{action_desc: \"{modes[mode]}\"}}}}'")})


TTS_SCRIPT = r'''
import rclpy, subprocess, sys, os, time, wave
from rclpy.node import Node
from aimdk_msgs.srv import PlayAudioFile, RequestAudioFocus

text = sys.argv[1] if len(sys.argv) > 1 else "Hello"
model = "/agibot/data/home/agi/stage_v3/data/piper_voices/en_US-hfc_female-medium.onnx"
wav_path = "/tmp/_tts.wav"

subprocess.run(
    'echo "' + text + '" | /agibot/data/home/agi/.local/bin/piper -m ' + model + ' --output_file ' + wav_path,
    shell=True, capture_output=True, timeout=15)
if not os.path.exists(wav_path):
    print("TTS generation failed"); sys.exit(1)

with wave.open(wav_path, "rb") as wf:
    sr = wf.getframerate()
    ch = wf.getnchannels()

rclpy.init()
node = Node("tts_play")

fc = node.create_client(RequestAudioFocus, "/aimdk_5Fmsgs/srv/RequestAudioFocus")
if fc.wait_for_service(timeout_sec=3.0):
    ft = fc.call_async(RequestAudioFocus.Request())
    rclpy.spin_until_future_complete(node, ft, timeout_sec=3.0)

cli = node.create_client(PlayAudioFile, "/aimdk_5Fmsgs/srv/PlayAudioFile")
if cli.wait_for_service(timeout_sec=5.0):
    req = PlayAudioFile.Request()
    req.file.file_path = wav_path
    req.file.file_name = "_tts.wav"
    req.file.pkg_name = "tts"
    req.file.info.channels = ch
    req.file.info.sample_rate = sr
    req.file.info.sample_format = "S16LE"
    req.file.info.coding_format = "wav"
    req.file.priority = 10
    ft2 = cli.call_async(req)
    rclpy.spin_until_future_complete(node, ft2, timeout_sec=8.0)
    print("Spoke: " + text)
else:
    print("PlayAudioFile service not available")

node.destroy_node()
rclpy.shutdown()
'''


@app.route('/api/tts', methods=['POST'])
def api_tts():
    text = request.json.get('text', 'Hello').replace("'", "").replace('"', '').replace('\\', '')
    encoded = base64.b64encode(TTS_SCRIPT.encode()).decode()
    result = ssh_cmd(
        f'{SETUP_CMD} >/dev/null 2>&1; echo {encoded} | base64 -d > /tmp/_tts_play.py && python3 /tmp/_tts_play.py "{text}"',
        timeout=25)
    return jsonify({"result": result})


@app.route('/api/topics')
def api_topics():
    return jsonify({"result": ssh_cmd("ros2 topic list")})


# Camera — proxy from robot_server.py
@app.route('/api/camera/start')
def api_camera_start():
    cam = request.args.get('cam', 'depth')
    try:
        import urllib.request
        urllib.request.urlopen(f"{ROBOT_SERVER}/camera/start?cam={cam}", timeout=5)
        return jsonify({"status": "started", "cam": cam})
    except Exception as e:
        # Fallback to SSH method
        start_camera(cam)
        return jsonify({"status": "started-ssh", "cam": cam})

@app.route('/api/camera/stop')
def api_camera_stop():
    try:
        import urllib.request
        urllib.request.urlopen(f"{ROBOT_SERVER}/camera/stop", timeout=5)
    except Exception:
        stop_camera()
    return jsonify({"status": "stopped"})

@app.route('/api/camera/frame')
def api_camera_frame():
    frame = None
    # Try robot server first
    try:
        import urllib.request
        resp = urllib.request.urlopen(f"{ROBOT_SERVER}/camera/frame", timeout=2)
        if resp.status == 200:
            frame = resp.read()
    except Exception:
        pass
    # Fallback to local SSH stream
    if not frame:
        with frame_lock:
            frame = latest_frame
    if not frame:
        return Response(b'', status=204)
    # Apply AI processing if enabled, otherwise just flip
    if yolo_enabled or face_rec_enabled:
        frame = process_frame_ai(frame)
    elif motion_detect_enabled:
        frame = process_frame_motion(frame)
    else:
        frame = flip_and_resize(frame)
    return Response(frame, mimetype='image/jpeg')

@app.route('/api/camera/stream')
def api_camera_stream():
    """Direct MJPEG proxy from robot server."""
    try:
        import urllib.request
        resp = urllib.request.urlopen(f"{ROBOT_SERVER}/camera/stream", timeout=30)
        def gen():
            while True:
                chunk = resp.read(65536)
                if not chunk: break
                yield chunk
        return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')
    except Exception:
        return Response(b'', status=503)

# Walk
# ── AI Vision routes ─────────────────────────────────────────────

@app.route('/api/ai/yolo/toggle')
def api_yolo_toggle():
    global yolo_enabled
    yolo_enabled = not yolo_enabled
    if yolo_enabled:
        load_yolo()
    return jsonify({"enabled": yolo_enabled})

@app.route('/api/ai/faces/toggle')
def api_faces_toggle():
    global face_rec_enabled
    face_rec_enabled = not face_rec_enabled
    if face_rec_enabled and not known_faces:
        load_known_faces()
    return jsonify({"enabled": face_rec_enabled, "known_count": len(known_faces)})

@app.route('/api/ai/faces/add', methods=['POST'])
def api_faces_add():
    name = request.form.get('name', '')
    if not name or 'photo' not in request.files:
        return jsonify({"error": "need name and photo"}), 400
    photo = request.files['photo']
    FACES_DIR.mkdir(exist_ok=True)
    path = FACES_DIR / f"{name}.jpg"
    photo.save(str(path))
    import face_recognition
    img = face_recognition.load_image_file(str(path))
    encs = face_recognition.face_encodings(img)
    if encs:
        known_faces[name] = encs[0]
        return jsonify({"status": "added", "name": name, "total": len(known_faces)})
    else:
        path.unlink()
        return jsonify({"error": "no face found in photo"}), 400

@app.route('/api/ai/faces/list')
def api_faces_list():
    return jsonify({"faces": list(known_faces.keys())})

@app.route('/api/ai/status')
def api_ai_status():
    return jsonify({
        "yolo": yolo_enabled,
        "yolo_loaded": yolo_model is not None,
        "faces": face_rec_enabled,
        "known_faces": list(known_faces.keys())
    })

# ── IMU routes ──────────────────────────────────────────────────

@app.route('/api/imu/start')
def api_imu_start():
    try:
        import urllib.request
        urllib.request.urlopen(f"{ROBOT_SERVER}/imu/start", timeout=5)
        return jsonify({"status": "started"})
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/api/imu/stop')
def api_imu_stop():
    try:
        import urllib.request
        urllib.request.urlopen(f"{ROBOT_SERVER}/imu/stop", timeout=5)
    except Exception:
        pass
    return jsonify({"status": "stopped"})

@app.route('/api/imu/data')
def api_imu_data():
    try:
        import urllib.request
        resp = urllib.request.urlopen(f"{ROBOT_SERVER}/imu/data", timeout=2)
        return Response(resp.read(), mimetype='application/json')
    except Exception:
        return jsonify({})

# ── Walk routes ──────────────────────────────────────────────────

@app.route('/api/walk/start')
def api_walk_start():
    start_walk(); return jsonify({"status": "starting"})

@app.route('/api/walk/stop')
def api_walk_stop():
    stop_walk(); return jsonify({"status": "stopped"})

@app.route('/api/walk/vel', methods=['POST'])
def api_walk_vel():
    d = request.json
    send_walk_velocity(d.get('f', 0), d.get('a', 0), d.get('l', 0))
    return jsonify({"status": "ok"})

# LiDAR — proxy from robot_server.py
@app.route('/api/lidar/start')
def api_lidar_start():
    try:
        import urllib.request
        urllib.request.urlopen(f"{ROBOT_SERVER}/lidar/start", timeout=5)
        return jsonify({"status": "started"})
    except Exception:
        start_lidar()
        return jsonify({"status": "started-ssh"})

@app.route('/api/lidar/stop')
def api_lidar_stop():
    try:
        import urllib.request
        urllib.request.urlopen(f"{ROBOT_SERVER}/lidar/stop", timeout=5)
    except Exception:
        stop_lidar()
    return jsonify({"status": "stopped"})

@app.route('/api/lidar/points')
def api_lidar_points():
    try:
        import urllib.request
        resp = urllib.request.urlopen(f"{ROBOT_SERVER}/lidar/points", timeout=3)
        return Response(resp.read(), mimetype='application/json')
    except Exception:
        with lidar_lock:
            pts = lidar_points
        return jsonify({"points": pts})

# Workflows
@app.route('/api/workflows', methods=['GET'])
def api_workflows_list():
    return jsonify(load_workflows())

@app.route('/api/workflows', methods=['POST'])
def api_workflows_save():
    wf = request.json
    workflows = load_workflows()
    # Update or add
    found = False
    for i, w in enumerate(workflows):
        if w.get("id") == wf.get("id"):
            workflows[i] = wf
            found = True
            break
    if not found:
        wf["id"] = str(int(time.time() * 1000))
        workflows.append(wf)
    save_workflows(workflows)
    return jsonify({"status": "saved", "id": wf["id"]})

@app.route('/api/workflows/<wf_id>', methods=['DELETE'])
def api_workflow_delete(wf_id):
    workflows = [w for w in load_workflows() if w.get("id") != wf_id]
    save_workflows(workflows)
    return jsonify({"status": "deleted"})

@app.route('/api/workflows/<wf_id>/run', methods=['POST'])
def api_workflow_run(wf_id):
    workflows = load_workflows()
    wf = next((w for w in workflows if w.get("id") == wf_id), None)
    if not wf: return jsonify({"error": "not found"}), 404
    results = execute_workflow(wf)
    return jsonify({"results": results})

# n8n
@app.route('/api/n8n/trigger', methods=['POST'])
def api_n8n_trigger():
    d = request.json
    return jsonify({"result": trigger_n8n(d.get("url", ""), d.get("payload", {}))})

@app.route('/api/n8n/config', methods=['GET'])
def api_n8n_config():
    return jsonify({"webhook_url": N8N_WEBHOOK_URL})

@app.route('/api/n8n/config', methods=['POST'])
def api_n8n_config_set():
    global N8N_WEBHOOK_URL
    N8N_WEBHOOK_URL = request.json.get("webhook_url", "")
    return jsonify({"status": "ok"})

# Incoming webhook (n8n can call this to trigger robot actions)
@app.route('/api/webhook/robot', methods=['POST'])
def api_webhook_robot():
    """n8n or external services can POST here to control the robot."""
    d = request.json or {}
    action = d.get("action", "")
    params = d.get("params", {})

    if action == "emoji":
        return api_emoji(params.get("id", 90))
    elif action == "tts":
        return api_tts()
    elif action == "motion":
        return api_motion(params.get("type", "wave_left"))
    elif action == "mode":
        return api_mode(params.get("mode", "stand"))
    elif action == "workflow":
        return api_workflow_run(params.get("id", ""))
    else:
        return jsonify({"error": f"Unknown action: {action}"}), 400


# ── URDF API ─────────────────────────────────────────────────────

URDF_DIR = Path(__file__).resolve().parent.parent / "X2_URDF-v1.3.0"

def parse_urdf():
    """Parse URDF file and return joint/link data for 3D visualization."""
    import xml.etree.ElementTree as ET
    urdf_path = URDF_DIR / "x2_ultra.urdf"
    if not urdf_path.exists():
        return {"error": "URDF not found"}
    tree = ET.parse(str(urdf_path))
    root = tree.getroot()

    links = []
    for link in root.findall('link'):
        name = link.get('name')
        visual = link.find('visual')
        mesh = None
        if visual is not None:
            geo = visual.find('geometry')
            if geo is not None:
                m = geo.find('mesh')
                if m is not None:
                    mesh = m.get('filename', '').replace('./meshes/', '')
        inertial = link.find('inertial')
        mass = 0
        if inertial is not None:
            mass_el = inertial.find('mass')
            if mass_el is not None:
                mass = float(mass_el.get('value', 0))
        # Extract RGBA color from material
        color = None
        if visual is not None:
            mat = visual.find('material')
            if mat is not None:
                c = mat.find('color')
                if c is not None:
                    color = [round(float(x), 3) for x in c.get('rgba', '0.5 0.5 0.5 1').split()]
        links.append({"name": name, "mesh": mesh, "mass": round(mass, 3), "color": color})

    joints = []
    for j in root.findall('joint'):
        name = j.get('name')
        jtype = j.get('type')
        parent = j.find('parent').get('link')
        child = j.find('child').get('link')
        origin = j.find('origin')
        xyz = [float(x) for x in (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split()]
        rpy = [float(x) for x in (origin.get('rpy', '0 0 0') if origin is not None else '0 0 0').split()]
        axis_el = j.find('axis')
        axis = [float(x) for x in (axis_el.get('xyz', '0 0 1') if axis_el is not None else '0 0 1').split()]
        limit_el = j.find('limit')
        lower = float(limit_el.get('lower', 0)) if limit_el is not None else 0
        upper = float(limit_el.get('upper', 0)) if limit_el is not None else 0
        joints.append({
            "name": name, "type": jtype, "parent": parent, "child": child,
            "xyz": xyz, "rpy": rpy, "axis": axis,
            "lower": round(lower, 4), "upper": round(upper, 4)
        })

    return {"links": links, "joints": joints, "name": root.get('name', 'x2')}

@app.route('/api/urdf')
def api_urdf():
    return jsonify(parse_urdf())

@app.route('/api/urdf/mesh/<filename>')
def api_urdf_mesh(filename):
    """Serve STL mesh files from URDF directory."""
    meshdir = URDF_DIR / "meshes"
    if not (meshdir / filename).exists():
        return Response(b'', status=404)
    return send_from_directory(str(meshdir), filename)

# ── Motion Detection (YOLO fallback) ────────────────────────────

prev_frame_gray = None

def process_frame_motion(jpeg_bytes):
    """Simple motion detection as YOLO fallback — no extra deps needed."""
    global prev_frame_gray
    img = cv2.imdecode(np.frombuffer(jpeg_bytes, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        return jpeg_bytes
    img = cv2.rotate(img, cv2.ROTATE_180)
    h, w = img.shape[:2]
    if w > 1920:
        scale = 1920 / w
        img = cv2.resize(img, (1920, int(h * scale)))

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (21, 21), 0)

    if prev_frame_gray is not None and prev_frame_gray.shape == gray.shape:
        delta = cv2.absdiff(prev_frame_gray, gray)
        thresh = cv2.threshold(delta, 25, 255, cv2.THRESH_BINARY)[1]
        thresh = cv2.dilate(thresh, None, iterations=2)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            if cv2.contourArea(c) < 500:
                continue
            x, y, bw, bh = cv2.boundingRect(c)
            cv2.rectangle(img, (x, y), (x + bw, y + bh), (0, 255, 255), 2)
            cv2.putText(img, "motion", (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    prev_frame_gray = gray
    ok, out = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
    return out.tobytes() if ok else jpeg_bytes

motion_detect_enabled = False

@app.route('/api/ai/motion/toggle')
def api_motion_detect_toggle():
    global motion_detect_enabled
    motion_detect_enabled = not motion_detect_enabled
    return jsonify({"enabled": motion_detect_enabled})

# ── System status ────────────────────────────────────────────────

@app.route('/api/status')
def api_system_status():
    """Combined system status for the dashboard."""
    ssh_ok = False
    try:
        r = subprocess.run(
            ["ssh", "-o", f"ControlPath={SSH_SOCKET}", "-O", "check", ROBOT_HOST],
            capture_output=True, timeout=3)
        ssh_ok = r.returncode == 0
    except Exception:
        pass

    robot_ok = False
    try:
        import urllib.request
        resp = urllib.request.urlopen(f"{ROBOT_SERVER}/health", timeout=2)
        robot_ok = resp.status == 200
    except Exception:
        pass

    return jsonify({
        "ssh": ssh_ok,
        "robot_server": robot_ok,
        "camera": camera_process is not None,
        "walk": walk_process is not None,
        "lidar": lidar_process is not None,
        "yolo": yolo_enabled,
        "yolo_loaded": yolo_model is not None,
        "motion_detect": motion_detect_enabled,
        "faces": face_rec_enabled,
        "n8n_url": bool(N8N_WEBHOOK_URL),
    })

# ── SSH Terminal ─────────────────────────────────────────────────

@app.route('/api/terminal', methods=['POST'])
def api_terminal():
    """Execute a command on the robot and return output. Supports history."""
    cmd = request.json.get('cmd', '').strip()
    if not cmd:
        return jsonify({"output": ""})
    result = ssh_cmd(cmd, timeout=15)
    return jsonify({"output": result, "cmd": cmd})


if __name__ == '__main__':
    os.makedirs(Path(__file__).parent / 'templates', exist_ok=True)
    os.makedirs(Path(__file__).parent / 'static', exist_ok=True)
    print()
    print("  ╔═══════════════════════════════════════╗")
    print("  ║   Agibot X2 Advanced Dashboard        ║")
    print("  ║   http://localhost:5000                ║")
    print("  ╚═══════════════════════════════════════╝")
    print()
    print("  Establishing SSH master connection...")
    ensure_ssh_master()
    print("  Loading known faces...")
    try:
        load_known_faces()
    except Exception as e:
        print(f"  Face loading skipped: {e}")
    print(f"  Known faces: {list(known_faces.keys()) or 'none'}")
    print("  Ready!\n")
    try:
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    except KeyboardInterrupt:
        stop_camera()
        stop_walk()
        stop_lidar()
        subprocess.run(["ssh", "-o", f"ControlPath={SSH_SOCKET}", "-O", "exit", ROBOT_HOST],
                       capture_output=True, timeout=5)
        print("\nShutdown.")
