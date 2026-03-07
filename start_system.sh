#!/bin/bash

# ==============================================================================
# Hawkeye ROS2 System Launcher
# ==============================================================================

set -eo pipefail

# Change to the script's directory so all relative paths are correct
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
WORKDIR="$SCRIPT_DIR"

echo "============================================================"
echo "Hawkeye ROS2 System - Starting All Components"
echo "============================================================"
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# ==============================================================================
# Detect docker compose command
# ==============================================================================

if docker compose version &>/dev/null; then
    DC="docker compose"
elif command -v docker-compose &>/dev/null; then
    DC="docker-compose"
else
    echo -e "${RED}Error: Neither 'docker compose' nor 'docker-compose' found${NC}"
    exit 1
fi

# ==============================================================================
# Step 1: Check Prerequisites
# ==============================================================================

echo -e "${YELLOW}[1/5] Checking prerequisites...${NC}"

# Check if Docker is running
if ! docker info > /dev/null 2>&1; then
    echo -e "${RED}Error: Docker is not running. Please start Docker first.${NC}"
    exit 1
fi
echo -e "${GREEN}✓ Docker is running${NC}"

# Check if test_images exists and has images
if [ ! -d "test_images" ] || [ -z "$(ls -A test_images/*.jpg 2>/dev/null)" ]; then
    echo -e "${YELLOW}Warning: test_images/ folder is empty or missing${NC}"
    echo "Creating test_images folder..."
    mkdir -p test_images
    echo -e "${YELLOW}Please add .jpg images to test_images/ folder and run again${NC}"
    echo "You can download test images:"
    echo "  curl -L https://picsum.photos/1920/1080 -o test_images/test1.jpg"
    exit 1
fi
echo -e "${GREEN}✓ Test images found${NC}"

# Check if mock_gcom.py exists
if [ ! -f "mock_gcom.py" ]; then
    echo -e "${RED}Error: mock_gcom.py not found in ${WORKDIR}${NC}"
    exit 1
fi
echo -e "${GREEN}✓ mock_gcom.py found${NC}"

# Set up Python virtual environment
VENV_DIR="$WORKDIR/venv"
if [ ! -f "$VENV_DIR/bin/activate" ]; then
    echo "  Creating Python virtual environment..."
    python3 -m venv "$VENV_DIR"
fi
# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"
# Prevent colcon (inside Docker) from scanning the venv when the entire
# repo root is bind-mounted into the container (see docker-compose.override.yml)
touch "$VENV_DIR/COLCON_IGNORE"

# Check Python dependencies against requirements.txt
if ! python3 -c "import websockets, aiortc, cv2" 2>/dev/null; then
    echo -e "${YELLOW}Warning: Python dependencies missing${NC}"
    echo "Installing dependencies from requirements.txt..."
    pip install -r requirements.txt
fi
echo -e "${GREEN}✓ Python venv ready${NC}"

echo ""

# ==============================================================================
# Step 2: Start Docker Container
# ==============================================================================

echo -e "${YELLOW}[2/5] Starting Docker container...${NC}"
$DC down 2>/dev/null || true
$DC up -d

# Wait for container to be actually running (up to 30 seconds)
echo "  Waiting for container to be ready..."
WAITED=0
MAX_WAIT=30
until docker inspect --format='{{.State.Running}}' hawkeye-ros 2>/dev/null | grep -q true; do
    if [ "$WAITED" -ge "$MAX_WAIT" ]; then
        echo -e "${RED}Error: Container 'hawkeye-ros' failed to start within ${MAX_WAIT}s${NC}"
        $DC logs --tail=20
        exit 1
    fi
    sleep 1
    WAITED=$((WAITED + 1))
done
echo -e "${GREEN}✓ Docker container started${NC}"
echo ""

# ==============================================================================
# Step 3: Wait for ROS Workspace Build
# ==============================================================================

echo -e "${YELLOW}[3/5] Waiting for ROS workspace build...${NC}"
echo "  (colcon build --symlink-install is running inside the container)"
WAITED=0
MAX_WAIT=300  # colcon can take a while on first build
until $DC exec -T ros2_workspace test -f /ros2_ws/install/setup.bash 2>/dev/null; do
    if [ "$WAITED" -ge "$MAX_WAIT" ]; then
        echo -e "${RED}Error: ROS workspace build did not finish within ${MAX_WAIT}s${NC}"
        echo "Check container logs:"
        $DC logs --tail=30
        exit 1
    fi
    sleep 2
    WAITED=$((WAITED + 2))
    echo -ne "  Waited ${WAITED}s...\r"
done
echo -e "${GREEN}✓ ROS workspace built${NC}"
echo ""

# ==============================================================================
# Step 4: Open Terminal Windows
# ==============================================================================

echo -e "${YELLOW}[4/5] Opening terminal windows...${NC}"

# Open a named tmux window. Falls back to background log file if tmux is missing.
open_terminal() {
    local title="$1"
    local cmd="$2"

    if command -v tmux &>/dev/null; then
        if ! tmux has-session -t hawkeye 2>/dev/null; then
            tmux new-session -d -s hawkeye -n "$title" "bash -c '$cmd; exec bash'"
        else
            tmux new-window -t hawkeye -n "$title" "bash -c '$cmd; exec bash'"
        fi
        echo "  → tmux window: $title"
    else
        mkdir -p "$WORKDIR/logs"
        local logfile="$WORKDIR/logs/${title// /_}.log"
        bash -c "$cmd" > "$logfile" 2>&1 &
        echo -e "  ${YELLOW}→ tmux not found. '$title' running in background → $logfile${NC}"
    fi
}

# echo "  → Starting GCOM server..."
# open_terminal "GCOM Server" "cd '$WORKDIR' && '$VENV_DIR/bin/python3' mock_gcom.py"
# sleep 1

echo "  → Start Streaming NOde..."
open_terminal "Object Detection" "cd '$WORKDIR' && $DC exec ros2_workspace bash -c \"source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run streaming streaming\""
sleep 1

echo "  → Starting Object Detection..."
open_terminal "Object Detection" "cd '$WORKDIR' && $DC exec ros2_workspace bash -c \"source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run orchestrator mock_object_detection\""
sleep 1

# echo "  → Starting Orchestrator..."
# open_terminal "Orchestrator" "cd '$WORKDIR' && $DC exec ros2_workspace bash -c \"source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run orchestrator orchestrator\""

echo -e "${GREEN}✓ Terminal windows opened${NC}"
echo ""

# ==============================================================================
# Step 5: Display Instructions
# ==============================================================================

echo "============================================================"
echo -e "${GREEN}SUCCESS! System is starting...${NC}"
echo "============================================================"
echo ""
echo -e "${YELLOW}Three tmux windows started in session 'hawkeye':${NC}"
echo "  Attach with: tmux attach -t hawkeye"
echo "  1. GCOM Server (Host)"
echo "  2. Object Detection (Docker)"
echo "  3. Orchestrator (Docker)"
echo ""
echo -e "${YELLOW}Watch the windows for status messages.${NC}"
echo ""
echo "Expected flow:"
echo "  1. GCOM server waits for connections"
echo "  2. Object Detection starts publishing images (30 FPS)"
echo "  3. Orchestrator connects to GCOM"
echo "  4. GCOM sends 'start_stream' command (after 3s)"
echo "  5. Video stream starts!"
echo ""
echo "Check results in: received_stream/"
echo ""
echo "To stop:"
echo "  1. Close all terminal windows (or Ctrl+C in each)"
echo "  2. Run: ./stop_system.sh"
echo "============================================================"
echo ""
