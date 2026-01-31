#!/bin/bash

# ==============================================================================
# Hawkeye ROS2 System - Stop Script (Git Bash - Windows)
# ==============================================================================

echo "============================================================"
echo "Stopping Hawkeye ROS2 System"
echo "============================================================"
echo ""

# Stop GCOM server (Python process)
echo "Stopping GCOM server..."
# Find and kill python processes running mock_gcom.py
pkill -f "python.*mock_gcom.py" 2>/dev/null || echo "  No GCOM process found"

# Stop Docker containers
echo "Stopping Docker containers..."
docker-compose down

# Optional: Kill any remaining ros2 processes
echo "Cleaning up ROS processes..."
pkill -f "ros2 run" 2>/dev/null || true

echo ""
echo "============================================================"
echo "System stopped"
echo "============================================================"
echo ""
echo "All terminal windows can be closed manually if still open."
echo ""
echo "To start again: ./start_system_gitbash.sh"
echo ""