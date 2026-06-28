# Agibot X2 Ultra - Web Dashboard & Automation Platform

A full-featured web control dashboard, 3D URDF simulator, and n8n automation bridge for the [Agibot X2 Ultra](https://store.agibot.com/products/x2-ultra) humanoid robot. Built on top of the official AimDK SDK (v0.9.0).

The system lets you control every aspect of the robot from a browser — arm motions, walking, camera feeds, LiDAR mapping, text-to-speech, face expressions — and automate sequences via n8n workflows or direct API calls.

## Architecture

```
 ┌──────────────┐       SSH / HTTP        ┌──────────────────────┐
 │  Your Laptop │ ◄──────────────────────► │  Agibot X2 Ultra     │
 │              │                          │  (ARM64, Ubuntu 22)  │
 │  dashboard/  │    ┌─────────────┐       │                      │
 │  app.py      │◄───│ robot_      │──────►│  ROS 2 Humble        │
 │  :5000       │    │ server.py   │       │  aimdk_msgs          │
 │              │    │ :8080       │       │  sensors / actuators │
 └──────┬───────┘    └─────────────┘       └──────────────────────┘
        │
        │ webhooks
        ▼
 ┌──────────────┐
 │  n8n         │
 │  :5678       │
 │  (Docker)    │
 └──────────────┘
```

**Dashboard (app.py, port 5000)** runs on your laptop. It connects to the robot via SSH multiplexing for ROS 2 commands and via HTTP to the robot server for sensor streams.

**Robot Server (robot_server.py, port 8080)** runs on the robot itself. It subscribes to ROS 2 camera, LiDAR, and IMU topics via subprocess-spawned Python nodes, then serves the data over HTTP to the dashboard. It also handles text-to-speech via piper, forwarding audio to the SOC2 speaker board through a local play_bridge service (port 8081).

**n8n (port 5678)** runs in Docker on your laptop. It triggers robot actions via webhooks to the dashboard API.

## Features

### Web Dashboard (7 tabs)

| Tab | What it does |
|-----|-------------|
| **Controls** | Camera feeds (6 cameras), 20+ arm motions, 10 face emojis, text-to-speech (piper TTS engine), WASD walk control, YOLO / motion detection / face recognition toggles |
| **URDF Sim** | Real-time 3D visualizer loading actual X2 Ultra STL meshes from the URDF. 31 joint sliders grouped by body part (collapsible). Preset poses: stand, crouch, sit, T-pose, bow, hands-up. Animations: wave, walk, dance, look around |
| **LiDAR** | Live 3D point cloud map from the chest-mounted LiDAR. Accumulates points over time with height-based coloring |
| **Workflows** | Visual workflow builder — chain emoji, TTS, motions, waits, shell commands, and n8n webhooks into reusable sequences |
| **IMU** | Real-time 3D orientation visualization from chest IMU, quaternion/angular velocity/acceleration readouts, live acceleration graph |
| **n8n** | Configure outgoing/incoming webhooks, test payloads, see API docs for external integrations |
| **Terminal** | SSH terminal to the robot with command history (up/down arrows), quick-access buttons for common ROS 2 commands, system stats |

### Robot Preset Motions (from AimDK SDK)

| Motion | ID | Area | Motion | ID | Area |
|--------|----|------|--------|----|------|
| Wave | 1002 | 1=L, 2=R | Raise hand | 1001 | 1=L, 2=R |
| Handshake | 1003 | 1=L, 2=R | Air kiss | 1004 | 1=L, 2=R |
| Salute | 1013 | 1=L, 2=R | High-five | 1008 | 1=L, 2=R |
| Heart gesture | 1007 | 1=L, 2=R, 3=both | Raise both | 1010 | 1=L, 2=R, 3=both |
| Clap | 3017 | 11 | Hug | 3008 | 11 |
| Cheer | 3011 | 11 | Wave goodbye | 3031 | 11 |
| Cross arms | 3009 | 11 | Bow | 3001 | 11 |
| Scratch head | 3024 | 11 | Light wave | 3007 | 11 |

### Robot Modes

| Mode | action_desc | Description |
|------|-------------|-------------|
| Stand | `STAND_DEFAULT` | Active balancing, required for motions |
| Locomotion | `LOCOMOTION_DEFAULT` | Walking/running |
| Joint Lock | `JOINT_DEFAULT` | Position-controlled stand |
| Damping | `DAMPING_DEFAULT` | Joints have damping resistance |
| Passive | `PASSIVE_DEFAULT` | Zero torque, free state |

### Camera Topics

| Camera | ROS 2 Topic |
|--------|------------|
| Head Front Center | `/aima/hal/sensor/rgb_head_front_center/rgb_image/compressed` (auto-flipped 180) |
| RGBD Front | `/aima/hal/sensor/rgbd_head_front/rgb_image/compressed` |
| Depth | `/camera/depth/image_raw/compressedDepth` (auto-flipped 180) |
| Stereo Left | `/aima/hal/sensor/stereo_head_front_left/rgb_image/compressed` |
| Stereo Right | `/aima/hal/sensor/stereo_head_front_right/rgb_image/compressed` |
| Head Rear | `/aima/hal/sensor/rgb_head_rear/rgb_image/compressed` (auto-flipped 180) |

## Setup

### Prerequisites

- Python 3.8+
- `sshpass` (for SSH to robot)
- `opencv-python`, `numpy`, `flask` Python packages
- Docker + Docker Compose (for n8n, optional)
- [piper](https://github.com/rhasspy/piper) TTS engine installed on the robot (for text-to-speech)
- The robot must be on the same network as your machine

### 1. Install dependencies

```bash
pip install flask opencv-python numpy
sudo apt install sshpass   # Ubuntu/Debian
```

### 2. Configure robot IP

Edit `dashboard/app.py` — update these lines with your robot's IP:

```python
ROBOT_HOST = "agi@<ROBOT_IP>"
ROBOT_PASS = "1"
ROBOT_IP = "<ROBOT_IP>"
```

### 3. Deploy the robot server (on the robot)

```bash
sshpass -p 1 scp robot_server.py agi@<ROBOT_IP>:~/
sshpass -p 1 ssh agi@<ROBOT_IP> "bash -i -c 'nohup python3 ~/robot_server.py &'"
```

This runs the sensor bridge on port 8080 on the robot. It subscribes to ROS 2 camera/LiDAR/IMU topics and serves them over HTTP. It also provides the `/tts` endpoint using piper for speech synthesis.

### 4. Start the dashboard

```bash
cd dashboard
python3 app.py
```

Open http://localhost:5000 in your browser.

### 5. Deploy n8n (optional)

```bash
cd n8n
./deploy.sh
```

This starts n8n on port 5678 and imports 3 pre-built workflows. Open http://localhost:5678 to manage them.

## n8n Workflows

Three ready-to-import workflows are included in `n8n/`:

### Greet Visitor

Triggered via webhook. Plays happy emoji, speaks a personalized greeting, waves.

```bash
curl -X POST http://localhost:5678/webhook-test/agibot-greet \
  -H 'Content-Type: application/json' \
  -d '{"name": "Alice"}'
```

### Scheduled Demo Loop

Runs every 5 minutes. Sets stand mode, speaks an introduction, performs a random motion and emoji. Ideal for trade shows and booth demos.

### Slack Command Bridge

Parses natural language commands and routes them to the robot:

```bash
curl -X POST http://localhost:5678/webhook-test/agibot-slack \
  -H 'Content-Type: application/json' \
  -d '{"text": "say Hello from Slack!"}'
```

Supported commands: `say <text>`, `wave [left|right]`, `shake [left|right]`, `emoji <id>`, `mode <mode>`, `bow`, `cheer`

## CLI Tool

The `robot.sh` script provides quick terminal access:

```bash
./robot.sh emoji 90          # Happy face
./robot.sh wave right         # Wave right arm
./robot.sh handshake          # Offer handshake (right hand)
./robot.sh tts "Hello world"  # Text to speech
./robot.sh mode stand         # Switch to stand mode
./robot.sh mode locomotion    # Enable walking
./robot.sh topics             # List all ROS 2 topics
./robot.sh services           # List all ROS 2 services
./robot.sh raw topic echo /aima/hal/imu/chest/state --once
```

## REST API Reference

The dashboard exposes a REST API on port 5000:

### Robot Control

```
GET  /api/emoji/<id>              Play face emoji
GET  /api/motion/<action>         Arm motion (wave_left, handshake_right, etc.)
GET  /api/mode/<mode>             Set mode (stand, locomotion, passive, etc.)
POST /api/tts                     {"text": "Hello"} — via piper on robot server
POST /api/preset_motion           {"motion_id": 3017, "area_id": 11}
POST /api/walk/vel                {"f": 0.3, "a": 0.0, "l": 0.0}
```

### Sensors

```
GET  /api/camera/start?cam=depth  Start camera stream
GET  /api/camera/frame            Get latest JPEG frame
GET  /api/lidar/start             Start LiDAR point cloud
GET  /api/lidar/points            Get current points
GET  /api/imu/start               Start IMU stream
GET  /api/imu/data                Get orientation/acceleration
```

### Robot Server (port 8080)

The robot server also exposes its own endpoints directly:

```
POST /tts                         {"text": "Hello"} — piper TTS via play_bridge
GET  /cam?topic=...               Start camera subscription
GET  /frame                       Get latest JPEG frame
GET  /lidar/start                 Start LiDAR stream
GET  /lidar/points                Get LiDAR point cloud
GET  /imu/start                   Start IMU stream
GET  /imu/data                    Get IMU orientation/acceleration
```

### AI Vision

```
GET  /api/ai/yolo/toggle          Toggle YOLO object detection
GET  /api/ai/motion/toggle        Toggle motion detection
GET  /api/ai/faces/toggle         Toggle face recognition
GET  /api/ai/status               Get AI module status
```

### URDF

```
GET  /api/urdf                    Get parsed URDF joint/link data as JSON
GET  /api/urdf/mesh/<file>.STL    Serve STL mesh file for 3D viewer
```

### Automation

```
POST /api/webhook/robot           Incoming webhook for n8n / external services
POST /api/terminal                {"cmd": "ros2 topic list"}
GET  /api/status                  System status (SSH, robot, sensors, AI)
POST /api/workflows               Save workflow
POST /api/workflows/<id>/run      Execute saved workflow
POST /api/n8n/trigger             {"url": "...", "payload": {...}}
```

### Webhook Payload Format

External services (n8n, Slack, etc.) can control the robot by POSTing to `/api/webhook/robot`:

```json
{"action": "tts",    "params": {"text": "Hello!"}}
{"action": "emoji",  "params": {"id": 90}}
{"action": "motion", "params": {"type": "wave_right"}}
{"action": "mode",   "params": {"mode": "stand"}}
```

## URDF Simulator

The simulator loads the actual X2 Ultra URDF (`X2_URDF-v1.3.0/x2_ultra.urdf`) with all 45 STL mesh files directly in the browser using Three.js. The kinematic chain is built from the URDF joint definitions:

- **41 links**, **40 joints** (31 revolute, 9 fixed)
- Joint sliders respect real limits from the URDF
- Coordinate transform: URDF (X=forward, Y=left, Z=up) to Three.js (X=forward, Y=up, Z=back)
- Preset pose keyframes use smooth easeInOutQuad interpolation

## Project Structure

```
.
├── dashboard/
│   ├── app.py                  # Main Flask dashboard (port 5000)
│   ├── templates/index.html    # Single-page UI (Three.js, vanilla JS)
│   ├── known_faces/            # Face recognition reference photos
│   └── workflows.json          # Saved workflow sequences
├── robot_server.py             # Sensor bridge — runs on the robot (port 8080)
├── robot.sh                    # CLI tool
├── n8n/
│   ├── docker-compose.yml      # One-command n8n deployment
│   ├── deploy.sh               # Deploy + import workflows
│   ├── workflow_greet_visitor.json
│   ├── workflow_scheduled_demo.json
│   └── workflow_slack_robot.json
├── X2_URDF-v1.3.0/
│   ├── x2_ultra.urdf           # Robot description (31 DOF)
│   ├── meshes/*.STL            # 45 mesh files for 3D visualization
│   └── visual/*.png            # Joint diagrams
├── src/                        # AimDK SDK packages (C++/Python examples)
│   ├── aimdk_msgs/             # Custom ROS 2 message/service definitions
│   ├── examples/               # C++ example nodes
│   ├── py_examples/            # Python example nodes
│   └── ruckig/                 # Trajectory generation library
└── docs/                       # Official AimDK documentation (HTML)
```

## Official Documentation

- [AimDK Online Docs](https://x2-aimdk.agibot.com) (SDK reference, examples, API)
- [Agibot X2 Ultra Product Page](https://store.agibot.com/products/x2-ultra)
- [AimDK Preset Motion Reference](https://x2-aimdk.agibot.com/en/latest/Interface/control_mod/preset_motion.html)
- [AimDK Mode Switch Reference](https://x2-aimdk.agibot.com/en/latest/Interface/control_mod/modeswitch.html)

## License

The AimDK SDK and URDF files are provided by Agibot. The dashboard, robot server, n8n workflows, and automation tooling in this repository are open source.
