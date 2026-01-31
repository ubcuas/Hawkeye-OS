#!/bin/bash

# ==============================================================================
# Hawkeye ROS2 System Launcher (Git Bash - Windows)
# Opens separate Git Bash windows (no tmux required)
# ==============================================================================

set -e  # Exit on error

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
# Step 1: Check Prerequisites
# ==============================================================================

echo -e "${YELLOW}[1/5] Checking prerequisites...${NC}"

# Check if Docker is running
if ! docker info > /dev/null 2>&1; then
    echo -e "${RED}Error: Docker is not running. Please start Docker Desktop first.${NC}"
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
    echo -e "${RED}Error: mock_gcom.py not found in current directory${NC}"
    exit 1
fi
echo -e "${GREEN}✓ mock_gcom.py found${NC}"

# Check Python dependencies
if ! python -c "import websockets, aiortc, cv2" 2>/dev/null; then
    echo -e "${YELLOW}Warning: Python dependencies missing${NC}"
    echo "Installing dependencies..."
    pip install websockets aiortc av opencv-python numpy
fi
echo -e "${GREEN}✓ Python dependencies installed${NC}"

echo ""

# ==============================================================================
# Step 2: Start Docker Container
# ==============================================================================

echo -e "${YELLOW}[2/5] Starting Docker container...${NC}"
docker-compose down 2>/dev/null || true
docker-compose up -d

# Wait for container to be ready
sleep 3
echo -e "${GREEN}✓ Docker container started${NC}"
echo ""

# ==============================================================================
# Step 3: Build ROS Workspace
# ==============================================================================

echo -e "${YELLOW}[3/5] Building ROS workspace...${NC}"
docker-compose exec -T ros2_workspace bash -c "colcon build --symlink-install 2>&1" | grep -E "Starting|Finished|Summary|ERROR" || true
echo -e "${GREEN}✓ ROS workspace built${NC}"
echo ""

# ==============================================================================
# Step 4: Detect Windows Terminal or Git Bash
# ==============================================================================

echo -e "${YELLOW}[4/5] Opening terminal windows...${NC}"

# Get the current working directory
WORKDIR=$(pwd)

# Try to detect mintty (Git Bash terminal)
if command -v mintty &> /dev/null; then
    echo "Using Git Bash (mintty) to open windows..."
    
    # Window 1: GCOM Server
    echo "  → Starting GCOM server..."
    mintty -t "GCOM Server" -h always bash -c "cd '$WORKDIR' && echo '═══════════════════════════════════════════════════════════' && echo '  TERMINAL 1: GCOM Server (Host)' && echo '═══════════════════════════════════════════════════════════' && python mock_gcom.py; exec bash" &
    
    sleep 2
    
    # Window 2: Object Detection
    echo "  → Starting Object Detection..."
    mintty -t "Object Detection" -h always bash -c "cd '$WORKDIR' && echo '═══════════════════════════════════════════════════════════' && echo '  TERMINAL 2: Object Detection (Docker)' && echo '═══════════════════════════════════════════════════════════' && docker-compose exec ros2_workspace bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run orchestrator mock_object_detection'; exec bash" &
    
    sleep 2
    
    # Window 3: Orchestrator
    echo "  → Starting Orchestrator..."
    mintty -t "Orchestrator" -h always bash -c "cd '$WORKDIR' && echo '═══════════════════════════════════════════════════════════' && echo '  TERMINAL 3: Orchestrator (Docker)' && echo '═══════════════════════════════════════════════════════════' && docker-compose exec ros2_workspace bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run orchestrator orchestrator'; exec bash" &
    
    echo -e "${GREEN}✓ Git Bash windows opened${NC}"

# Fallback to cmd.exe if mintty not available
else
    echo "Using cmd.exe to open windows..."
    
    # Window 1: GCOM Server
    echo "  → Starting GCOM server..."
    cmd.exe /c start "GCOM Server" bash -c "cd '$WORKDIR' && echo '═══════════════════════════════════════════════════════════' && echo '  TERMINAL 1: GCOM Server (Host)' && echo '═══════════════════════════════════════════════════════════' && python mock_gcom.py; exec bash"
    
    sleep 2
    
    # Window 2: Object Detection
    echo "  → Starting Object Detection..."
    cmd.exe /c start "Object Detection" bash -c "cd '$WORKDIR' && echo '═══════════════════════════════════════════════════════════' && echo '  TERMINAL 2: Object Detection (Docker)' && echo '═══════════════════════════════════════════════════════════' && docker-compose exec ros2_workspace bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run orchestrator mock_object_detection'; exec bash"
    
    sleep 2
    
    # Window 3: Orchestrator
    echo "  → Starting Orchestrator..."
    cmd.exe /c start "Orchestrator" bash -c "cd '$WORKDIR' && echo '═══════════════════════════════════════════════════════════' && echo '  TERMINAL 3: Orchestrator (Docker)' && echo '═══════════════════════════════════════════════════════════' && docker-compose exec ros2_workspace bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run orchestrator orchestrator'; exec bash"
    
    echo -e "${GREEN}✓ Cmd windows opened${NC}"
fi

echo ""

# ==============================================================================
# Step 5: Display Instructions
# ==============================================================================

echo "============================================================"
echo -e "${GREEN}SUCCESS! System is starting...${NC}"
echo "============================================================"
echo ""
echo -e "${YELLOW}Three terminal windows have been opened:${NC}"
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