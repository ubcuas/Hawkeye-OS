# Hawkeye-OS

## Repository Structure 
```bash 
ros2_pubsub/
├── Dockerfile
├── README.md
├── docker-compose.yml
├── test-hawkeye-os.sh
└── src/
    ├── py_pubsub/
    │   ├── package.xml
    │   ├── setup.py
    │   ├── setup.cfg
    │   ├── resource/
    │   │   └── py_pubsub
    │   └── py_pubsub/
    │       ├── __init__.py
    │       ├── publisher.py
    │       └── subscriber.py
    └── cpp_pubsub/
        ├── package.xml
        ├── CMakeLists.txt
        └── src/
            ├── publisher.cpp
            └── subscriber.cpp
```

## Getting Started 

Install `docker, docker-compose, tmux`.

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

# Orchestrator 

Inside the terminal, after the workspace is built, run the orchestrator
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

# Others 
There are sample files called `py_pubsub, cpp_pubsub` which show basics of ROS. They'll be removed but feel free to browse for now.
