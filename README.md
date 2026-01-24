# Hawkeye-OS

## Repository Structure 
```bash 
ros2/
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

Install required Python Packages: 
```bash
pip install websockets aiortc av opencv-python numpy
```

### Building the Image for the First Time (Manual)
Build the image 
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
py


```bash 
ros2 run orchestrator orchestrator
```

For testing, mock queues are available
```bash
ros2 run orchestrator mock_image_capture ("on one terminal")
ros2 run orchestrator mock_object_detection ("on another terminal")
```

# Automating Build with Script

To run the script, make sure you make it executable with:
```bash
chmod +x test-hawkeye-os.sh
```
Run the script:
```bash
./test-hawkeye-os.sh
```