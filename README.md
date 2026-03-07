# Hawkeye-OS

## Repository Structure 
```bash 
Hawkeye-OS/
├── .dockerignore 
├── .gitignore 
├── docker-compose.yml 
├── Dockerfile
├── mock_gcom.py 
├── README.md
├── start_system.sh
├── stop_system.sh 
├── test-hawkeye-os.sh 
├── received_stream/
└── src/
    └── orchestrator
        ├── package.xml
        ├── setup.cfg
        ├── setup.py
        ├── orchestrator/
            ├── __init__.py
            ├── mock_object_detection.py
            └── orchestrator.py 
        └── resource/
            └── orchestrator
└── test_images
    ├── anpanman_wooddadandan_hero.jpg
    └── test_video.mp4
```

### Installing Dependencies 
Install `docker, docker-compose, tmux`.

// I think we can ignore this since we have the requirements.txt
Install required Python Packages: 
```bash
pip install websockets aiortc av opencv-python numpy
```

### Building the Image for the First Time (Manual)
Build the image (in project root)
```bash
docker-compose build
```

Run the container
```bash
docker-compose up -d
```

Get the bash (On each terminal to test)
```bash
docker-compose exec ros2_workspace bash
```

Inside the terminal, build the workplace
```bash
colcon build
```

### Testing (Manual) 
Make sure you're in the project's root directory (../Hawkeye-OS)

In a Normal Terminal: 
```bash 
python3 mock_gcom.py
```

Inside the Docker Workspace (see previous section for setup): 
```bash 
ros2 run orchestrator orchestrator
```

For testing, mock queues are available 
```bash
ros2 run orchestrator mock_object_detection ("on another terminal")
```

### Unit Tests

Set up the virtual environment (first time only):
```bash
python -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

Run all tests:
```bash
source venv/bin/activate
pytest
```

Run with coverage:
```bash
pytest --cov=src --cov-report=term-missing
```

Run a specific file (example):
```bash
pytest tests/streaming/test_streaming.py
```

### Automated Build/Test
For bash shells, there are files you can run to automate the test setups. 

To run the script, make sure you make it executable with:
```bash
chmod +x start_system.sh stop_system.sh
```

Run the script (in project root):
```bash
./start_system.sh 
```

To stop the script: 
```bash 
./stop_system.sh 
```