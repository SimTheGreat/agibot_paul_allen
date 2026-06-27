#!/usr/bin/env python3
"""
Agibot X2 Robot Control Dashboard
Web UI for controlling the robot via SSH + ROS 2 commands.

Usage: python3 robot_ui.py
Then open http://localhost:5000
"""

import subprocess
import threading
import struct
import time
import base64
import json
import sys
from flask import Flask, render_template_string, jsonify, request, Response

app = Flask(__name__)

ROBOT_HOST = "agi@10.104.218.77"
ROBOT_PASS = "1"
ROBOT_IP = "10.104.218.77"
SETUP_CMD = "source ~/Botifull/SLAM_stack/scripts/setup_env.sh && export FASTRTPS_DEFAULT_PROFILES_FILE=/agibot/data/home/agi/.aima/env/ros_dds_configuration.xml"

# SSH ControlMaster socket for connection reuse
SSH_SOCKET = "/tmp/agibot_ssh_ctl"

# Camera streaming state
camera_process = None
camera_lock = threading.Lock()
latest_frame = None
frame_lock = threading.Lock()

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
    """Start a persistent SSH master connection if not already running."""
    try:
        r = subprocess.run(
            ["ssh", "-o", f"ControlPath={SSH_SOCKET}", "-O", "check", ROBOT_HOST],
            capture_output=True, timeout=5
        )
        if r.returncode == 0:
            return True
    except Exception:
        pass
    # Start master connection
    try:
        subprocess.Popen(
            ["sshpass", "-p", ROBOT_PASS, "ssh",
             "-o", "StrictHostKeyChecking=no",
             "-o", f"ControlPath={SSH_SOCKET}",
             "-o", "ControlMaster=yes",
             "-o", "ControlPersist=300",
             "-N",
             ROBOT_HOST],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        time.sleep(1)
        print("SSH master connection established.")
        return True
    except Exception as e:
        print(f"SSH master failed: {e}")
        return False


def ssh_cmd(cmd, timeout=30):
    """Run a command on the robot via SSH (reuses master connection)."""
    full = f'{SETUP_CMD} >/dev/null 2>&1 && {cmd}'
    try:
        r = subprocess.run(
            SSH_BASE + [full],
            capture_output=True, text=True, timeout=timeout
        )
        out = r.stdout.strip()
        err = r.stderr.strip()
        # Filter DDS noise from stderr
        err_lines = [l for l in err.splitlines()
                     if 'matched_reader_remove' not in l
                     and 'matched_writer_remove' not in l
                     and 'RTPS_WRITER' not in l
                     and 'RTPS_READER' not in l
                     and 'Warning' not in l]
        err_clean = '\n'.join(err_lines).strip()
        result = out
        if err_clean:
            result += '\n' + err_clean if result else err_clean
        return result if result else "(no output)"
    except subprocess.TimeoutExpired:
        return "ERROR: Command timed out (30s)"
    except Exception as e:
        return f"ERROR: {e}"


# ── Camera streaming ────────────────────────────────────────────

CAMERA_TOPICS = {
    'depth': '/camera/depth/image_raw/compressedDepth',
    'rgbd_front': '/aima/hal/sensor/rgbd_head_front/rgb_image/compressed',
    'stereo_left': '/aima/hal/sensor/stereo_head_front_left/rgb_image/compressed',
    'stereo_right': '/aima/hal/sensor/stereo_head_front_right/rgb_image/compressed',
    'head_front': '/aima/hal/sensor/rgb_head_front_center/rgb_image/compressed',
    'head_rear': '/aima/hal/sensor/rgb_head_rear/rgb_image/compressed',
}

CAMERA_SCRIPT_TEMPLATE = '''
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
                if len(raw) <= 12: return
                png_data = raw[12:]
                img = cv2.imdecode(np.frombuffer(png_data, np.uint8), cv2.IMREAD_UNCHANGED)
                if img is None: return
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
    """SSH into robot, run camera streamer, pipe JPEG frames back."""
    global camera_process, latest_frame

    script = CAMERA_SCRIPT_TEMPLATE.format(topic=topic)
    encoded = base64.b64encode(script.encode()).decode()

    # Write script to temp file, source env (suppress output), run script
    # This avoids: (1) setup_env output corrupting binary stream
    #              (2) pipe-based python3 consuming stdin
    cmd = (
        f'{SETUP_CMD} >/dev/null 2>&1; '
        f'echo {encoded} | base64 -d > /tmp/_cam_stream.py && '
        f'exec python3 -u /tmp/_cam_stream.py'
    )

    try:
        proc = subprocess.Popen(
            SSH_BASE + [cmd],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )
        with camera_lock:
            camera_process = proc

        while proc.poll() is None:
            header = proc.stdout.read(4)
            if len(header) < 4:
                # Check if process died
                if proc.poll() is not None:
                    err = proc.stderr.read().decode(errors='ignore')
                    if err.strip():
                        print(f"Camera stderr: {err.strip()}")
                break
            length = struct.unpack('>I', header)[0]
            if length > 5_000_000 or length == 0:
                # Corrupted frame, try to resync by reading one byte at a time
                continue
            data = proc.stdout.read(length)
            if len(data) == length:
                with frame_lock:
                    latest_frame = data
    except Exception as e:
        print(f"Camera stream error: {e}")
    finally:
        with camera_lock:
            if camera_process == proc:
                camera_process = None


def start_camera(camera_key='rgbd_front'):
    global latest_frame
    stop_camera()
    time.sleep(0.3)
    latest_frame = None
    topic = CAMERA_TOPICS.get(camera_key, CAMERA_TOPICS['rgbd_front'])
    print(f"Starting camera: {camera_key} -> {topic}")
    t = threading.Thread(target=camera_stream_worker, args=(topic,), daemon=True)
    t.start()


def stop_camera():
    global camera_process
    with camera_lock:
        if camera_process:
            try:
                camera_process.kill()
            except Exception:
                pass
            camera_process = None


# ── Walk control ────────────────────────────────────────────────

walk_process = None
walk_lock = threading.Lock()
walk_fwd = 0.0
walk_ang = 0.0
walk_lat = 0.0
walk_registered = False

WALK_SCRIPT = '''
import rclpy, sys, json, select, time, os
from rclpy.node import Node
from aimdk_msgs.msg import McLocomotionVelocity, MessageHeader
from aimdk_msgs.srv import SetMcInputSource

class W(Node):
    def __init__(self):
        super().__init__("walk_ctrl")
        self.pub = self.create_publisher(McLocomotionVelocity, "/aima/mc/locomotion/velocity", 10)
        self.cli = self.create_client(SetMcInputSource, "/aimdk_5Fmsgs/srv/SetMcInputSource")
        self.fwd = 0.0
        self.ang = 0.0
        self.lat = 0.0
        self.register()
        self.tmr = self.create_timer(0.02, self.tick)
        self.stdin_tmr = self.create_timer(0.05, self.read_stdin)

    def register(self):
        self.cli.wait_for_service(timeout_sec=10.0)
        rq = SetMcInputSource.Request()
        rq.action.value = 1001
        rq.input_source.name = "node"
        rq.input_source.priority = 40
        rq.input_source.timeout = 1000
        for i in range(8):
            rq.request.header.stamp = self.get_clock().now().to_msg()
            ft = self.cli.call_async(rq)
            rclpy.spin_until_future_complete(self, ft, timeout_sec=0.5)
            if ft.done():
                sys.stderr.write("REGISTERED\\n")
                sys.stderr.flush()
                break

    def tick(self):
        m = McLocomotionVelocity()
        m.header = MessageHeader()
        m.header.stamp = self.get_clock().now().to_msg()
        m.source = "node"
        m.forward_velocity = self.fwd
        m.lateral_velocity = self.lat
        m.angular_velocity = self.ang
        self.pub.publish(m)

    def read_stdin(self):
        try:
            while select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline()
                if not line:
                    # EOF - stdin closed, stop
                    self.fwd = 0.0
                    self.ang = 0.0
                    self.lat = 0.0
                    self.tick()
                    time.sleep(0.1)
                    rclpy.shutdown()
                    return
                line = line.strip()
                if line == "QUIT":
                    self.fwd = 0.0
                    self.ang = 0.0
                    self.lat = 0.0
                    self.tick()
                    time.sleep(0.1)
                    rclpy.shutdown()
                    return
                try:
                    d = json.loads(line)
                    self.fwd = float(d.get("f", 0.0))
                    self.ang = float(d.get("a", 0.0))
                    self.lat = float(d.get("l", 0.0))
                except:
                    pass
        except Exception:
            pass

rclpy.init()
node = W()
try:
    rclpy.spin(node)
except:
    pass
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
'''


def start_walk():
    global walk_process, walk_registered, walk_fwd, walk_ang, walk_lat
    stop_walk()
    walk_fwd = 0.0
    walk_ang = 0.0
    walk_lat = 0.0
    walk_registered = False

    encoded = base64.b64encode(WALK_SCRIPT.encode()).decode()

    # Write to temp file so stdin remains available for velocity commands
    cmd = (
        f'{SETUP_CMD} >/dev/null 2>&1; '
        f'echo {encoded} | base64 -d > /tmp/_walk_ctrl.py && '
        f'exec python3 -u /tmp/_walk_ctrl.py'
    )

    proc = subprocess.Popen(
        SSH_BASE + [cmd],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE
    )
    with walk_lock:
        walk_process = proc

    # Watch stderr for REGISTERED signal
    def wait_reg():
        global walk_registered
        try:
            while proc.poll() is None:
                line = proc.stderr.readline()
                if not line:
                    break
                text = line.decode(errors='ignore').strip()
                if 'REGISTERED' in text:
                    walk_registered = True
                    print("Walk controller: registered input source")
                    break
                elif text:
                    print(f"Walk stderr: {text}")
        except Exception as e:
            print(f"Walk reg watcher error: {e}")
    t = threading.Thread(target=wait_reg, daemon=True)
    t.start()
    print("Walk controller: starting...")


def stop_walk():
    global walk_process, walk_registered
    with walk_lock:
        if walk_process:
            try:
                walk_process.stdin.write(b'QUIT\n')
                walk_process.stdin.flush()
            except Exception:
                pass
            try:
                walk_process.wait(timeout=3)
            except Exception:
                try:
                    walk_process.kill()
                except Exception:
                    pass
            walk_process = None
    walk_registered = False


def send_walk_velocity(fwd, ang, lat=0.0):
    global walk_fwd, walk_ang, walk_lat
    walk_fwd = fwd
    walk_ang = ang
    walk_lat = lat
    with walk_lock:
        if walk_process and walk_process.poll() is None:
            try:
                msg = json.dumps({"f": fwd, "a": ang, "l": lat}) + "\n"
                walk_process.stdin.write(msg.encode())
                walk_process.stdin.flush()
            except Exception as e:
                print(f"Walk send error: {e}")


# ── Routes ──────────────────────────────────────────────────────

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)


@app.route('/api/cmd', methods=['POST'])
def api_cmd():
    cmd = request.json.get('cmd', '')
    result = ssh_cmd(cmd)
    return jsonify({"result": result})


@app.route('/api/emoji/<int:emoji_id>')
def api_emoji(emoji_id):
    result = ssh_cmd(
        "ros2 service call /face_ui_proxy/play_emoji "
        "aimdk_msgs/srv/PlayEmoji "
        f"'{{emotion_id: {emoji_id}, mode: 1, priority: 10}}'")
    return jsonify({"result": result})


@app.route('/api/motion/<action>')
def api_motion(action):
    motion_map = {
        'wave_left':       (1002, 1),
        'wave_right':      (1002, 2),
        'handshake_left':  (1003, 1),
        'handshake_right': (1003, 2),
        'raise_left':      (1001, 1),
        'raise_right':     (1001, 2),
        'airkiss_left':    (1004, 1),
        'airkiss_right':   (1004, 2),
    }
    if action not in motion_map:
        return jsonify({"error": "unknown motion"}), 400
    mid, aid = motion_map[action]
    result = ssh_cmd(
        "ros2 service call /aimdk_5Fmsgs/srv/SetMcPresetMotion "
        "aimdk_msgs/srv/SetMcPresetMotion "
        f"'{{motion: {{value: {mid}}}, area: {{value: {aid}}}, interrupt: false}}'")
    return jsonify({"result": result})


@app.route('/api/mode/<mode>')
def api_mode(mode):
    modes = {
        'stand': 'STAND_DEFAULT',
        'locomotion': 'LOCOMOTION_DEFAULT',
        'passive': 'PASSIVE_DEFAULT',
        'damping': 'DAMPING_DEFAULT',
        'joint': 'JOINT_DEFAULT',
    }
    if mode not in modes:
        return jsonify({"error": "unknown mode"}), 400
    action = modes[mode]
    result = ssh_cmd(
        "ros2 service call /aimdk_5Fmsgs/srv/SetMcAction "
        "aimdk_msgs/srv/SetMcAction "
        f"'{{command: {{action_desc: \"{action}\"}}}}'")
    return jsonify({"result": result})


@app.route('/api/tts', methods=['POST'])
def api_tts():
    text = request.json.get('text', 'Hello')
    text = text.replace("'", "").replace('"', '')
    result = ssh_cmd(
        "ros2 service call /aimdk_5Fmsgs/srv/PlayTts "
        "aimdk_msgs/srv/PlayTts "
        f"'{{tts_req: {{text: \"{text}\", domain: demo, trace_id: test, "
        f"is_interrupted: true, priority_weight: 0, priority_level: {{value: 6}}}}}}'")
    return jsonify({"result": result})


@app.route('/api/topics')
def api_topics():
    result = ssh_cmd("ros2 topic list")
    return jsonify({"result": result})


@app.route('/api/camera/start')
def api_camera_start():
    cam = request.args.get('cam', 'rgbd_front')
    start_camera(cam)
    return jsonify({"status": "starting", "camera": cam})


@app.route('/api/camera/stop')
def api_camera_stop():
    stop_camera()
    return jsonify({"status": "stopped"})


@app.route('/api/camera/frame')
def api_camera_frame():
    with frame_lock:
        frame = latest_frame
    if frame:
        return Response(frame, mimetype='image/jpeg')
    return Response(b'', status=204)


@app.route('/api/walk/start')
def api_walk_start():
    start_walk()
    return jsonify({"status": "starting"})


@app.route('/api/walk/stop')
def api_walk_stop():
    stop_walk()
    return jsonify({"status": "stopped"})


@app.route('/api/walk/vel', methods=['POST'])
def api_walk_vel():
    d = request.json
    send_walk_velocity(d.get('f', 0), d.get('a', 0), d.get('l', 0))
    return jsonify({"status": "ok", "f": walk_fwd, "a": walk_ang, "l": walk_lat})


@app.route('/api/walk/status')
def api_walk_status():
    with walk_lock:
        running = walk_process is not None and walk_process.poll() is None
    return jsonify({
        "running": running,
        "registered": walk_registered,
        "f": walk_fwd, "a": walk_ang, "l": walk_lat
    })


# ── HTML Template ───────────────────────────────────────────────

HTML_TEMPLATE = r"""
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Agibot X2 Dashboard</title>
<style>
* { margin: 0; padding: 0; box-sizing: border-box; }
body {
    font-family: 'Segoe UI', system-ui, -apple-system, sans-serif;
    background: #0a0a1a; color: #e0e0e0; min-height: 100vh;
}
.header {
    background: linear-gradient(135deg, #1a1a3e 0%, #0d0d2b 100%);
    padding: 20px 30px; border-bottom: 2px solid #2a2a5a;
    display: flex; align-items: center; gap: 15px;
}
.header h1 {
    font-size: 24px;
    background: linear-gradient(90deg, #00d4ff, #7b2ff7);
    -webkit-background-clip: text; -webkit-text-fill-color: transparent;
}
.header .status {
    margin-left: auto; padding: 6px 16px; border-radius: 20px;
    font-size: 13px; font-weight: 600;
}
.status.online { background: #0a3d0a; color: #4eff4e; border: 1px solid #2a7a2a; }

.dashboard {
    display: grid; grid-template-columns: 1fr 1fr; gap: 20px;
    padding: 20px; max-width: 1400px; margin: 0 auto;
}
.panel {
    background: #12122a; border: 1px solid #2a2a5a;
    border-radius: 12px; padding: 20px;
}
.panel h2 {
    font-size: 16px; color: #8888cc; margin-bottom: 15px;
    text-transform: uppercase; letter-spacing: 1px;
    border-bottom: 1px solid #2a2a5a; padding-bottom: 8px;
}
.btn-grid {
    display: grid; grid-template-columns: repeat(auto-fill, minmax(120px, 1fr)); gap: 10px;
}
.btn {
    padding: 12px 8px; border: 1px solid #3a3a6a; border-radius: 8px;
    background: #1a1a3e; color: #c0c0ff; font-size: 13px;
    cursor: pointer; transition: all 0.2s; text-align: center;
    font-weight: 500; user-select: none;
}
.btn:hover { background: #2a2a5e; border-color: #5a5aaa; transform: translateY(-1px); }
.btn:active { transform: translateY(1px); }
.btn.active { background: #2a4a6a; border-color: #4a8abb; color: #80ccff; }
.btn.danger { border-color: #6a3a3a; color: #ff8888; }
.btn.danger:hover { background: #3a1a1a; border-color: #aa5a5a; }
.btn.success { border-color: #3a6a3a; color: #88ff88; }
.btn.success:hover { background: #1a3a1a; border-color: #5aaa5a; }
.btn.warning { border-color: #6a6a3a; color: #ffff88; }
.btn.warning:hover { background: #3a3a1a; border-color: #aaaa5a; }

.emoji-grid { grid-template-columns: repeat(auto-fill, minmax(90px, 1fr)); }
.emoji-btn { font-size: 12px; padding: 10px 6px; }

.camera-container {
    position: relative; background: #000; border-radius: 8px;
    overflow: hidden; min-height: 300px;
    display: flex; align-items: center; justify-content: center;
}
.camera-container img { max-width: 100%; max-height: 400px; }
.camera-placeholder { color: #555; font-size: 14px; }
.camera-controls { display: flex; gap: 10px; margin-top: 10px; }

.tts-row { display: flex; gap: 10px; margin-top: 10px; }
.tts-input {
    flex: 1; padding: 10px 14px; border: 1px solid #3a3a6a;
    border-radius: 8px; background: #1a1a3e; color: #e0e0e0;
    font-size: 14px; outline: none;
}
.tts-input:focus { border-color: #5a5aaa; }

.log-box {
    background: #0a0a1a; border: 1px solid #2a2a5a; border-radius: 8px;
    padding: 12px; margin-top: 10px; max-height: 200px; overflow-y: auto;
    font-family: 'Fira Code', 'Consolas', monospace; font-size: 12px;
    color: #888; white-space: pre-wrap; word-break: break-all;
}

.walk-panel { display: flex; gap: 20px; align-items: flex-start; }
.walk-controls {
    display: grid;
    grid-template-areas: ".    up    ." "left stop  right" ".    down  .";
    grid-template-columns: 70px 70px 70px; gap: 8px;
}
.walk-controls .btn { padding: 18px 8px; font-size: 16px; }
.walk-up { grid-area: up; } .walk-down { grid-area: down; }
.walk-left { grid-area: left; } .walk-right { grid-area: right; }
.walk-stop { grid-area: stop; }

.walk-info { flex: 1; display: flex; flex-direction: column; gap: 10px; }
.walk-status {
    padding: 10px; border-radius: 8px; background: #0a0a1a;
    border: 1px solid #2a2a5a; font-family: monospace; font-size: 13px;
}
.walk-status .val { color: #00d4ff; }

.full-width { grid-column: 1 / -1; }

@media (max-width: 800px) {
    .dashboard { grid-template-columns: 1fr; }
    .walk-panel { flex-direction: column; align-items: center; }
}
</style>
</head>
<body>

<div class="header">
    <h1>AGIBOT X2</h1>
    <span>Robot Control Dashboard</span>
    <span class="status online" id="statusBadge">10.255.198.77</span>
</div>

<div class="dashboard">

    <!-- Camera Panel -->
    <div class="panel">
        <h2>Camera Feed</h2>
        <div class="camera-container">
            <span class="camera-placeholder" id="cameraPlaceholder">Click START to begin streaming</span>
            <img id="cameraImg" style="display:none" />
        </div>
        <div class="camera-controls">
            <button class="btn success" onclick="startCamera()">START</button>
            <button class="btn danger" onclick="stopCamera()">STOP</button>
            <select class="tts-input" id="cameraSelect" style="flex:1">
                <option value="rgbd_front">RGBD Front</option>
                <option value="stereo_left">Stereo Left</option>
                <option value="stereo_right">Stereo Right</option>
                <option value="head_front">Head Front</option>
                <option value="head_rear">Head Rear</option>
            </select>
        </div>
    </div>

    <!-- Arm Motions + Mode -->
    <div class="panel">
        <h2>Arm Motions</h2>
        <div class="btn-grid">
            <button class="btn" onclick="motion('wave_left')">Wave Left</button>
            <button class="btn" onclick="motion('wave_right')">Wave Right</button>
            <button class="btn" onclick="motion('handshake_left')">Shake Left</button>
            <button class="btn" onclick="motion('handshake_right')">Shake Right</button>
            <button class="btn" onclick="motion('raise_left')">Raise Left</button>
            <button class="btn" onclick="motion('raise_right')">Raise Right</button>
            <button class="btn" onclick="motion('airkiss_left')">Kiss Left</button>
            <button class="btn" onclick="motion('airkiss_right')">Kiss Right</button>
        </div>
        <h2 style="margin-top:20px">Robot Mode</h2>
        <div class="btn-grid">
            <button class="btn success" onclick="setMode('stand')">Stand</button>
            <button class="btn active" onclick="setMode('locomotion')">Locomotion</button>
            <button class="btn" onclick="setMode('joint')">Joint Lock</button>
            <button class="btn" onclick="setMode('damping')">Damping</button>
            <button class="btn danger" onclick="setMode('passive')">Passive</button>
        </div>
    </div>

    <!-- Emoji + TTS -->
    <div class="panel">
        <h2>Face Emoji</h2>
        <div class="btn-grid emoji-grid">
            <button class="btn emoji-btn" onclick="emoji(1)">Blink</button>
            <button class="btn emoji-btn" onclick="emoji(60)">Bored</button>
            <button class="btn emoji-btn" onclick="emoji(70)">Abnormal</button>
            <button class="btn emoji-btn" onclick="emoji(80)">Sleeping</button>
            <button class="btn emoji-btn" onclick="emoji(90)">Happy</button>
            <button class="btn emoji-btn" onclick="emoji(100)">Sad</button>
            <button class="btn emoji-btn" onclick="emoji(110)">Confused</button>
            <button class="btn emoji-btn" onclick="emoji(120)">Surprised</button>
            <button class="btn emoji-btn" onclick="emoji(130)">Shy</button>
            <button class="btn emoji-btn" onclick="emoji(140)">Thinking</button>
            <button class="btn emoji-btn" onclick="emoji(150)">Angry</button>
            <button class="btn emoji-btn" onclick="emoji(160)">Laughing</button>
            <button class="btn emoji-btn" onclick="emoji(170)">Wink</button>
            <button class="btn emoji-btn" onclick="emoji(180)">Crying</button>
            <button class="btn emoji-btn" onclick="emoji(190)">Furious</button>
            <button class="btn emoji-btn" onclick="emoji(200)">Adore</button>
        </div>
        <h2 style="margin-top:20px">Text to Speech</h2>
        <div class="tts-row">
            <input type="text" class="tts-input" id="ttsInput"
                   placeholder="Type text to speak..." value="Hello I am Agibot X2" />
            <button class="btn success" onclick="speak()">Speak</button>
        </div>
    </div>

    <!-- Walk Control -->
    <div class="panel">
        <h2>Walk Control</h2>
        <div class="walk-panel">
            <div class="walk-controls">
                <button class="btn walk-up" onmousedown="wk('f',0.3)" onmouseup="wkRelease()">W</button>
                <button class="btn walk-left" onmousedown="wk('a',0.2)" onmouseup="wkRelease()">A</button>
                <button class="btn walk-stop danger" onclick="wkStop()">STOP</button>
                <button class="btn walk-right" onmousedown="wk('a',-0.2)" onmouseup="wkRelease()">D</button>
                <button class="btn walk-down" onmousedown="wk('f',-0.3)" onmouseup="wkRelease()">S</button>
            </div>
            <div class="walk-info">
                <div class="walk-status">
                    Forward: <span class="val" id="wFwd">0.0</span> m/s<br>
                    Turn: <span class="val" id="wAng">0.0</span> rad/s<br>
                    Strafe: <span class="val" id="wLat">0.0</span> m/s
                </div>
                <div style="display:flex; gap:8px; flex-wrap:wrap">
                    <button class="btn success" onclick="walkInit()">Enable Walk</button>
                    <button class="btn danger" onclick="walkShutdown()">Disable Walk</button>
                    <button class="btn warning" onclick="setMode('locomotion')">Set Locomotion</button>
                </div>
                <p style="color:#666;font-size:11px">
                    1. Set Locomotion Mode &rarr; 2. Enable Walk &rarr; 3. WASD / arrows / buttons
                </p>
            </div>
        </div>
    </div>

    <!-- Log Panel -->
    <div class="panel full-width">
        <h2>Command Log</h2>
        <div class="log-box" id="logBox">Dashboard ready.
</div>
    </div>
</div>

<script>
function log(msg) {
    const box = document.getElementById('logBox');
    const ts = new Date().toLocaleTimeString();
    box.textContent += `[${ts}] ${msg}\n`;
    box.scrollTop = box.scrollHeight;
}

async function api(url, opts) {
    try { const r = await fetch(url, opts); return await r.json(); }
    catch(e) { log('Network error: ' + e.message); return {error: e.message}; }
}

async function emoji(id) {
    log(`Emoji ${id}...`);
    const r = await api(`/api/emoji/${id}`);
    log(r.result || r.error);
}

async function motion(action) {
    log(`Motion: ${action}...`);
    const r = await api(`/api/motion/${action}`);
    log(r.result || r.error);
}

async function setMode(mode) {
    log(`Mode: ${mode}...`);
    const r = await api(`/api/mode/${mode}`);
    log(r.result || r.error);
}

async function speak() {
    const text = document.getElementById('ttsInput').value;
    if (!text) return;
    log(`TTS: "${text}"`);
    const r = await api('/api/tts', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({text})
    });
    log(r.result || r.error);
}
document.getElementById('ttsInput').addEventListener('keydown', e => {
    if (e.key === 'Enter') { speak(); e.preventDefault(); e.stopPropagation(); }
});

// Camera
let camInterval = null;
function startCamera() {
    if (camInterval) clearInterval(camInterval);
    const cam = document.getElementById('cameraSelect').value;
    log(`Camera: starting ${cam}...`);
    fetch('/api/camera/start?cam=' + cam);
    const img = document.getElementById('cameraImg');
    const ph = document.getElementById('cameraPlaceholder');
    camInterval = setInterval(() => {
        const tmp = new Image();
        tmp.onload = () => { img.src = tmp.src; img.style.display = 'block'; ph.style.display = 'none'; };
        tmp.src = '/api/camera/frame?' + Date.now();
    }, 150);
}
function stopCamera() {
    if (camInterval) { clearInterval(camInterval); camInterval = null; }
    fetch('/api/camera/stop');
    document.getElementById('cameraImg').style.display = 'none';
    document.getElementById('cameraPlaceholder').style.display = 'block';
    log('Camera stopped.');
}

// Walk
let wF = 0, wA = 0, wL = 0, walkRunning = false;
function updateWalkUI() {
    document.getElementById('wFwd').textContent = wF.toFixed(1);
    document.getElementById('wAng').textContent = wA.toFixed(1);
    document.getElementById('wLat').textContent = wL.toFixed(1);
}
async function walkInit() {
    log('Enabling walk controller...');
    await api('/api/walk/start');
    walkRunning = true;
    log('Walk starting (wait a few seconds for input source registration)...');
}
async function walkShutdown() {
    log('Disabling walk...');
    await api('/api/walk/stop');
    walkRunning = false; wF = 0; wA = 0; wL = 0; updateWalkUI();
    log('Walk stopped.');
}
function wk(axis, delta) {
    if (!walkRunning) { log('Click "Enable Walk" first!'); return; }
    if (axis === 'f') wF = Math.max(-1, Math.min(1, wF + delta));
    else if (axis === 'a') wA = Math.max(-1, Math.min(1, wA + delta));
    else if (axis === 'l') wL = Math.max(-1, Math.min(1, wL + delta));
    updateWalkUI();
    api('/api/walk/vel', {
        method: 'POST', headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({f: wF, a: wA, l: wL})
    });
}
function wkRelease() {}
function wkStop() {
    wF = 0; wA = 0; wL = 0; updateWalkUI();
    if (walkRunning) {
        api('/api/walk/vel', {
            method: 'POST', headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({f: 0, a: 0, l: 0})
        });
    }
    log('Walk: STOP');
}

// Keyboard
document.addEventListener('keydown', e => {
    if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
    if (e.repeat) return;
    switch(e.key) {
        case 'w': case 'ArrowUp':    wk('f', 0.3); e.preventDefault(); break;
        case 's': case 'ArrowDown':  wk('f', -0.3); e.preventDefault(); break;
        case 'a': case 'ArrowLeft':  wk('a', 0.2); e.preventDefault(); break;
        case 'd': case 'ArrowRight': wk('a', -0.2); e.preventDefault(); break;
        case 'q': wk('l', 0.2); break;
        case 'e': wk('l', -0.2); break;
        case ' ': wkStop(); e.preventDefault(); break;
    }
});
</script>
</body>
</html>
"""


if __name__ == '__main__':
    print()
    print("  +---------------------------------+")
    print("  |   Agibot X2 Control Dashboard   |")
    print("  |   http://localhost:5000          |")
    print("  +---------------------------------+")
    print()
    print("  Establishing SSH master connection...")
    ensure_ssh_master()
    print("  Ready!\n")
    try:
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    except KeyboardInterrupt:
        stop_camera()
        stop_walk()
        # Kill SSH master
        subprocess.run(
            ["ssh", "-o", f"ControlPath={SSH_SOCKET}", "-O", "exit", ROBOT_HOST],
            capture_output=True, timeout=5
        )
        print("\nShutdown.")
