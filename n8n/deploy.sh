#!/bin/bash
# Deploy n8n + import Agibot X2 workflows
# Usage: ./deploy.sh [DASHBOARD_IP]

set -e

DASHBOARD_IP="${1:-$(hostname -I | awk '{print $1}')}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo ""
echo "  ╔═══════════════════════════════════════╗"
echo "  ║   Agibot X2 - n8n Workflow Deploy     ║"
echo "  ╚═══════════════════════════════════════╝"
echo ""
echo "  Dashboard IP: $DASHBOARD_IP:5000"
echo ""

# Step 1: Start n8n
echo "  [1/3] Starting n8n with Docker Compose..."
cd "$SCRIPT_DIR"
docker compose up -d 2>/dev/null || docker-compose up -d

echo "  [2/3] Waiting for n8n to be ready..."
for i in $(seq 1 30); do
    if curl -sf http://localhost:5678/healthz > /dev/null 2>&1; then
        echo "  n8n is ready!"
        break
    fi
    sleep 2
    printf "."
done
echo ""

# Step 3: Import workflows (replace placeholder IP)
echo "  [3/3] Importing workflows..."
for wf in "$SCRIPT_DIR"/workflow_*.json; do
    name=$(basename "$wf" .json)
    echo "    Importing: $name"
    # Replace host.docker.internal with actual IP for non-Docker-Desktop environments
    tmpfile=$(mktemp)
    sed "s|host.docker.internal|$DASHBOARD_IP|g" "$wf" > "$tmpfile"

    curl -sf -X POST "http://localhost:5678/api/v1/workflows" \
        -H "Content-Type: application/json" \
        -d @"$tmpfile" > /dev/null 2>&1 && echo "      OK" || echo "      (may need manual import - open n8n UI)"
    rm -f "$tmpfile"
done

echo ""
echo "  Done! Open n8n at: http://localhost:5678"
echo ""
echo "  Workflows imported:"
echo "    1. Greet Visitor  - POST http://localhost:5678/webhook/agibot-greet"
echo "       Body: {\"name\": \"Alice\"}"
echo ""
echo "    2. Scheduled Demo - Runs every 5 min (activate in n8n UI)"
echo ""
echo "    3. Slack Bridge   - POST http://localhost:5678/webhook/agibot-slack"
echo "       Body: {\"text\": \"say Hello world\"}"
echo "       Commands: say, wave, shake, emoji, mode, bow, cheer"
echo ""
echo "  Test: curl -X POST http://localhost:5678/webhook-test/agibot-greet \\"
echo "         -H 'Content-Type: application/json' -d '{\"name\":\"Demo\"}'"
echo ""
