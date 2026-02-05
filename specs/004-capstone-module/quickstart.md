# Module 4: Vision-Language-Action (VLA) Capstone - Quickstart Guide

## Prerequisites
- Complete Modules 1-3
- Python 3.8+
- ROS 2 Humble Hawksbill
- Access to GPU (recommended) or sufficient CPU resources
- Microphone for voice input (for testing)

## Setup Instructions

### 1. Install VLA Dependencies
```bash
# Navigate to your workspace
cd ~/humanoid_robot_ws

# Install VLA-specific packages
pip install vosk openai speechrecognition transformers torch torchvision torchaudio
```

### 2. Download VLA Models
```bash
# Create models directory
mkdir -p ~/humanoid_robot_ws/models/vla

# For offline speech recognition
pip install vosk
# Download a model from https://alphacephei.com/vosk/models
# Extract to ~/humanoid_robot_ws/models/vla/vosk-model-small-en-us

# For transformer models
python -c "from transformers import AutoTokenizer, AutoModel; \
AutoTokenizer.from_pretrained('distilbert-base-uncased'); \
AutoModel.from_pretrained('distilbert-base-uncased')"
```

### 3. Launch VLA System
```bash
# Source your ROS 2 environment
source /opt/ros/humble/setup.bash
source ~/humanoid_robot_ws/install/setup.bash

# Launch the VLA capstone system
ros2 launch vla_capstone vla_system.launch.py
```

## Basic Commands

### Voice Interaction
1. Say "Robot, listen" to activate voice recognition
2. Give a command like "Navigate to the kitchen" or "Describe your surroundings"
3. Listen for voice feedback confirming the action

### Programmatic Interface
```bash
# Send a text command to the VLA system
ros2 topic pub /vla/text_command std_msgs/String "data: 'move forward 1 meter'"

# Monitor the VLA status
ros2 topic echo /vla/status

# Check the generated action plan
ros2 topic echo /vla/action_plan
```

## Testing the System

### 1. Voice-to-Action Test
```bash
# Start the voice recognition node
ros2 run vla_capstone voice_recognition_node

# Speak a simple command like "stop" or "move forward"
# Observe the robot's response
```

### 2. LLM Planning Test
```bash
# Publish a high-level task
ros2 topic pub /vla/high_level_task vla_msgs/HighLevelTask "task: 'fetch water bottle from kitchen'"
```

### 3. Capstone Scenario
```bash
# Run the complete capstone scenario
ros2 run vla_capstone capstone_demo_node
```

## Expected Behavior
- Voice commands should be recognized and converted to robot actions
- LLM should generate appropriate action sequences for complex tasks
- Robot should integrate perception, planning, and execution
- All safety constraints should be maintained

## Troubleshooting
- If voice recognition is not working, check microphone permissions
- If LLM responses are slow, verify internet connectivity or switch to offline mode
- If actions are not executing, check ROS 2 connections with `ros2 topic list`
- For model loading errors, ensure sufficient disk space and correct paths

## Next Steps
After completing this quickstart:
1. Experiment with different voice commands
2. Modify the LLM prompt templates for different behaviors
3. Create custom capstone scenarios
4. Integrate with your own robotic platform