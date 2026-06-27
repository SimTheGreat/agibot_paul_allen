#!/bin/bash
# Agibot X2 Robot Controller
# Usage: ./robot.sh <command> [args]

ROBOT="agi@10.104.218.77"
CTL="/tmp/agibot_ssh_ctl"
SSH="sshpass -p 1 ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 -o ControlPath=$CTL -o ControlMaster=auto -o ControlPersist=300"
SETUP="source ~/Botifull/SLAM_stack/scripts/setup_env.sh >/dev/null 2>&1 && export FASTRTPS_DEFAULT_PROFILES_FILE=/agibot/data/home/agi/.aima/env/ros_dds_configuration.xml"

rcmd() {
    $SSH "$ROBOT" "$SETUP && $1"
}

CMD="$1"
shift

case "$CMD" in
  emoji)
    ID="${1:-90}"
    echo "Playing emoji $ID..."
    rcmd "ros2 service call /face_ui_proxy/play_emoji aimdk_msgs/srv/PlayEmoji '{emotion_id: $ID, mode: 1, priority: 10}'"
    ;;
  wave)
    AREA=1; [[ "$1" == "right" ]] && AREA=2
    echo "Waving ${1:-left} arm..."
    rcmd "ros2 service call /aimdk_5Fmsgs/srv/SetMcPresetMotion aimdk_msgs/srv/SetMcPresetMotion '{motion: {value: 1002}, area: {value: $AREA}, interrupt: false}'"
    ;;
  handshake)
    AREA=2; [[ "$1" == "left" ]] && AREA=1
    echo "Handshake ${1:-right} arm..."
    rcmd "ros2 service call /aimdk_5Fmsgs/srv/SetMcPresetMotion aimdk_msgs/srv/SetMcPresetMotion '{motion: {value: 1003}, area: {value: $AREA}, interrupt: false}'"
    ;;
  airkiss)
    AREA=1; [[ "$1" == "right" ]] && AREA=2
    echo "Air kiss ${1:-left} arm..."
    rcmd "ros2 service call /aimdk_5Fmsgs/srv/SetMcPresetMotion aimdk_msgs/srv/SetMcPresetMotion '{motion: {value: 1004}, area: {value: $AREA}, interrupt: false}'"
    ;;
  raise)
    AREA=1; [[ "$1" == "right" ]] && AREA=2
    echo "Raising ${1:-left} arm..."
    rcmd "ros2 service call /aimdk_5Fmsgs/srv/SetMcPresetMotion aimdk_msgs/srv/SetMcPresetMotion '{motion: {value: 1001}, area: {value: $AREA}, interrupt: false}'"
    ;;
  tts)
    TEXT="${1:-Hello I am Agibot X2}"
    echo "Speaking: $TEXT"
    rcmd "ros2 service call /aimdk_5Fmsgs/srv/PlayTts aimdk_msgs/srv/PlayTts '{tts_req: {text: \"$TEXT\", domain: demo, trace_id: test, is_interrupted: true, priority_weight: 0, priority_level: {value: 6}}}'"
    ;;
  mode)
    case "$1" in
      stand)       ACTION="STAND_DEFAULT" ;;
      locomotion)  ACTION="LOCOMOTION_DEFAULT" ;;
      passive)     ACTION="PASSIVE_DEFAULT" ;;
      damping)     ACTION="DAMPING_DEFAULT" ;;
      joint)       ACTION="JOINT_DEFAULT" ;;
      *)           echo "Modes: stand, locomotion, passive, damping, joint"; exit 1 ;;
    esac
    echo "Setting mode: $ACTION"
    rcmd "ros2 service call /aimdk_5Fmsgs/srv/SetMcAction aimdk_msgs/srv/SetMcAction '{command: {action_desc: \"$ACTION\"}}'"
    ;;
  topics)
    rcmd "ros2 topic list"
    ;;
  services)
    rcmd "ros2 service list"
    ;;
  raw)
    rcmd "ros2 $*"
    ;;
  ui)
    echo "Starting web dashboard..."
    python3 "$(dirname "$0")/robot_ui.py"
    ;;
  *)
    cat << 'EOF'
Agibot X2 Robot Controller

Usage: ./robot.sh <command> [args]

Commands:
  emoji <id>          Emoji (1=blink 60=bored 80=sleep 90=happy 200=adore)
  wave [left|right]   Wave arm
  handshake [left]    Handshake (default: right)
  airkiss [right]     Air kiss (default: left)
  raise [left|right]  Raise arm
  tts "text"          Text to speech
  mode <mode>         Mode: stand, locomotion, passive, damping, joint
  topics              List ROS topics
  services            List ROS services
  raw <ros2 cmd>      Run any ros2 command
  ui                  Launch web dashboard
EOF
    ;;
esac
