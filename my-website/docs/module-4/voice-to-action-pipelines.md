---
sidebar_position: 2
title: "Voice-to-Action Pipelines"
---

# Voice-to-Action Pipelines

## Learning Objectives

After completing this chapter, you will be able to:
- Implement voice command processing pipelines using speech-to-text technologies
- Integrate Whisper API for accurate speech recognition in robotics applications
- Parse and interpret natural language commands for robot execution
- Design robust command validation and error handling systems
- Connect voice processing to ROS 2 action execution

## Introduction

Voice-to-action pipelines form the foundation of natural human-robot interaction, enabling users to control robots through spoken commands. In the context of Vision-Language-Action (VLA) systems, voice processing serves as the primary input modality that translates human intentions into executable robot actions. This chapter explores the implementation of voice processing systems that can accurately recognize speech and convert it into meaningful robotic commands.

## Speech Recognition with Whisper

### Overview of Whisper Technology

Whisper is a general-purpose speech recognition model developed by OpenAI that demonstrates robust performance across multiple languages and accents. For robotics applications, Whisper provides several key advantages:

- High accuracy across diverse speaking conditions
- Multi-language support
- Robustness to background noise
- Real-time processing capabilities

### Whisper Integration in ROS 2

To integrate Whisper with ROS 2, we create a speech recognition node that processes audio input and publishes recognized text:

```python
import rclpy
from rclpy.node import Node
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String
import whisper
import tempfile
import wave
import numpy as np

class WhisperSpeechRecognitionNode(Node):
    def __init__(self):
        super().__init__('whisper_speech_recognition')

        # Initialize Whisper model
        self.model = whisper.load_model("base")  # Use "small" or "medium" for better accuracy

        # Subscribe to audio data
        self.audio_sub = self.create_subscription(
            AudioData,
            'audio_input',
            self.audio_callback,
            10
        )

        # Publish recognized text
        self.text_pub = self.create_publisher(
            String,
            'recognized_text',
            10
        )

        # Parameter for confidence threshold
        self.declare_parameter('confidence_threshold', 0.7)
        self.confidence_threshold = self.get_parameter('confidence_threshold').value

        self.get_logger().info("Whisper Speech Recognition Node Initialized")

    def audio_callback(self, msg):
        """Process incoming audio data and perform speech recognition"""
        try:
            # Convert audio data to WAV format temporarily
            audio_array = np.frombuffer(msg.data, dtype=np.int16)

            # Create temporary WAV file for Whisper processing
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_wav:
                # Write audio data to temporary file
                wf = wave.open(temp_wav.name, 'wb')
                wf.setnchannels(1)  # Mono
                wf.setsampwidth(2)  # 16-bit
                wf.setframerate(16000)  # Standard rate
                wf.writeframes(msg.data)
                wf.close()

                # Process with Whisper
                result = self.model.transcribe(temp_wav.name)
                recognized_text = result['text'].strip()

                # Check confidence and publish if above threshold
                if len(recognized_text) > 0:
                    self.get_logger().info(f"Recognized: {recognized_text}")

                    # Publish recognized text
                    text_msg = String()
                    text_msg.data = recognized_text
                    self.text_pub.publish(text_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing audio: {e}")

def main(args=None):
    rclpy.init(args=args)

    speech_node = WhisperSpeechRecognitionNode()

    try:
        rclpy.spin(speech_node)
    except KeyboardInterrupt:
        pass
    finally:
        speech_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Whisper Model Selection

Different Whisper models offer trade-offs between accuracy and performance:

```python
# Model options and their characteristics:
MODELS = {
    'tiny': {
        'size': '75 MB',
        'relative_speed': '32x',
        'relative_accuracy': 'Lowest',
        'use_case': 'Real-time applications with limited resources'
    },
    'base': {
        'size': '145 MB',
        'relative_speed': '16x',
        'relative_accuracy': 'Low',
        'use_case': 'Basic applications with good performance'
    },
    'small': {
        'size': '485 MB',
        'relative_speed': '6x',
        'relative_accuracy': 'Medium',
        'use_case': 'Balanced performance and accuracy'
    },
    'medium': {
        'size': '1.5 GB',
        'relative_speed': '2x',
        'relative_accuracy': 'High',
        'use_case': 'Applications requiring high accuracy'
    },
    'large': {
        'size': '3.0 GB',
        'relative_speed': '1x',
        'relative_accuracy': 'Highest',
        'use_case': 'Applications where accuracy is critical'
    }
}
```

For robotics applications, the "base" or "small" models typically provide the best balance of accuracy and performance.

### Audio Preprocessing

Proper audio preprocessing is crucial for accurate speech recognition:

```python
import numpy as np
import librosa
from scipy import signal

class AudioPreprocessor:
    def __init__(self):
        self.sample_rate = 16000  # Standard for Whisper
        self.frame_size = 1024
        self.hop_length = 512

    def preprocess_audio(self, audio_data, original_sample_rate):
        """Preprocess audio for optimal Whisper performance"""
        # Resample to standard rate
        if original_sample_rate != self.sample_rate:
            audio_data = librosa.resample(
                audio_data.astype(np.float32),
                orig_sr=original_sample_rate,
                target_sr=self.sample_rate
            )

        # Normalize audio
        audio_data = audio_data / np.max(np.abs(audio_data))

        # Apply pre-emphasis filter to boost high frequencies
        audio_data = np.append(audio_data[0], audio_data[1:] - 0.97 * audio_data[:-1])

        # Convert back to int16 for Whisper
        audio_data = (audio_data * 32767).astype(np.int16)

        return audio_data

    def detect_speech_activity(self, audio_data, threshold=0.01):
        """Detect speech activity to reduce processing of silence"""
        # Calculate energy of audio frames
        frames = self._frame_audio(audio_data)
        energies = [np.mean(frame ** 2) for frame in frames]

        # Return indices of frames with significant energy
        active_indices = [i for i, energy in enumerate(energies) if energy > threshold]
        return active_indices

    def _frame_audio(self, audio_data):
        """Create overlapping frames from audio data"""
        frames = []
        for i in range(0, len(audio_data) - self.frame_size, self.hop_length):
            frames.append(audio_data[i:i + self.frame_size])
        return frames
```

## Natural Language Command Parsing

### Command Structure Analysis

Natural language commands for robots typically follow specific patterns that can be parsed and understood:

```python
import re
from typing import Dict, List, Tuple
from dataclasses import dataclass

@dataclass
class ParsedCommand:
    """Structure to hold parsed command information"""
    action: str  # The main action (move, pick, place, etc.)
    target: str  # The target object or location
    attributes: Dict[str, str]  # Additional attributes (color, size, etc.)
    confidence: float  # Confidence in parsing accuracy

class CommandParser:
    def __init__(self):
        # Define command patterns with regex
        self.patterns = [
            # Movement commands
            {
                'pattern': r'(?:go to|move to|navigate to|walk to)\s+(.+)',
                'action': 'navigate',
                'target_group': 1
            },
            {
                'pattern': r'(?:move|go)\s+(forward|backward|left|right|up|down)\s*(\d*\.?\d+)\s*(meters|meter|m)?',
                'action': 'move_direction',
                'target_group': 1,
                'distance_group': 2
            },
            {
                'pattern': r'(?:pick up|grab|take|lift)\s+(?:the\s+)?(.+)',
                'action': 'pick_up',
                'target_group': 1
            },
            {
                'pattern': r'(?:place|put|set|drop)\s+(?:the\s+)?(.+)\s+(?:on|at|in)\s+(.+)',
                'action': 'place',
                'target_group': 1,
                'location_group': 2
            },
            {
                'pattern': r'(?:look at|face|turn toward)\s+(.+)',
                'action': 'look_at',
                'target_group': 1
            },
            {
                'pattern': r'(?:follow|track)\s+(.+)',
                'action': 'follow',
                'target_group': 1
            }
        ]

        # Attribute patterns
        self.attribute_patterns = {
            'color': r'(red|blue|green|yellow|orange|purple|pink|brown|black|white)',
            'size': r'(small|large|big|medium|tiny|huge)',
            'shape': r'(cube|sphere|cylinder|box|rectangle)',
            'direction': r'(front|back|left|right|up|down)'
        }

    def parse_command(self, text: str) -> ParsedCommand:
        """Parse natural language command into structured format"""
        text = text.lower().strip()

        # Extract attributes first
        attributes = self._extract_attributes(text)

        # Try to match command patterns
        for pattern_info in self.patterns:
            match = re.search(pattern_info['pattern'], text)
            if match:
                action = pattern_info['action']

                # Extract target
                target = ''
                if 'target_group' in pattern_info:
                    target = match.group(pattern_info['target_group'])

                # Extract additional information
                additional_info = {}
                if 'distance_group' in pattern_info:
                    distance = match.group(pattern_info['distance_group'])
                    if distance:
                        additional_info['distance'] = float(distance)

                if 'location_group' in pattern_info:
                    location = match.group(pattern_info['location_group'])
                    additional_info['location'] = location

                # Calculate confidence based on pattern match strength
                confidence = self._calculate_confidence(text, match)

                return ParsedCommand(
                    action=action,
                    target=target,
                    attributes={**attributes, **additional_info},
                    confidence=confidence
                )

        # If no pattern matches, return generic command
        return ParsedCommand(
            action='unknown',
            target=text,
            attributes=attributes,
            confidence=0.1
        )

    def _extract_attributes(self, text: str) -> Dict[str, str]:
        """Extract object attributes from text"""
        attributes = {}

        for attr_type, pattern in self.attribute_patterns.items():
            match = re.search(pattern, text)
            if match:
                attributes[attr_type] = match.group(1)

        return attributes

    def _calculate_confidence(self, text: str, match) -> float:
        """Calculate confidence in parsing result"""
        # Simple confidence based on match length vs text length
        match_length = len(match.group(0))
        text_length = len(text)

        # Prefer longer, more specific matches
        base_confidence = min(match_length / text_length, 1.0)

        # Boost confidence for well-defined commands
        if any(keyword in text for keyword in ['please', 'now', 'immediately']):
            base_confidence *= 1.1

        return min(base_confidence, 1.0)

# Example usage
parser = CommandParser()
command = parser.parse_command("Please pick up the red cube and place it on the table")
print(f"Action: {command.action}")
print(f"Target: {command.target}")
print(f"Attributes: {command.attributes}")
print(f"Confidence: {command.confidence:.2f}")
```

### Intent Classification with LLMs

For more sophisticated command understanding, we can use LLMs to classify intents:

```python
import openai
from typing import Optional
import json

class LLMIntentClassifier:
    def __init__(self, api_key: str):
        self.client = openai.OpenAI(api_key=api_key)

        # Define robot command schema
        self.command_schema = {
            "type": "object",
            "properties": {
                "action": {
                    "type": "string",
                    "enum": [
                        "navigate", "move_direction", "pick_up", "place",
                        "look_at", "follow", "grasp", "release", "wave",
                        "speak", "take_photo", "find_object", "stop"
                    ]
                },
                "target": {"type": "string"},
                "location": {"type": "string"},
                "attributes": {
                    "type": "object",
                    "properties": {
                        "color": {"type": "string"},
                        "size": {"type": "string"},
                        "shape": {"type": "string"},
                        "distance": {"type": "number"}
                    }
                },
                "confidence": {"type": "number", "minimum": 0, "maximum": 1}
            },
            "required": ["action", "confidence"]
        }

    def classify_intent(self, command_text: str) -> Optional[ParsedCommand]:
        """Use LLM to classify intent and extract parameters"""
        try:
            response = self.client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[
                    {
                        "role": "system",
                        "content": "You are a robot command parser. Extract structured information from natural language commands. Respond only with valid JSON matching the provided schema."
                    },
                    {
                        "role": "user",
                        "content": f"Parse this robot command: '{command_text}'"
                    }
                ],
                functions=[{
                    "name": "parse_robot_command",
                    "description": "Parse a robot command into structured format",
                    "parameters": self.command_schema
                }],
                function_call={"name": "parse_robot_command"}
            )

            # Extract the function arguments
            message = response.choices[0].message
            if message.function_call:
                args = json.loads(message.function_call.arguments)

                return ParsedCommand(
                    action=args.get('action', 'unknown'),
                    target=args.get('target', ''),
                    attributes=args.get('attributes', {}),
                    confidence=args.get('confidence', 0.5)
                )

        except Exception as e:
            print(f"Error calling LLM: {e}")
            return None

        return None
```

## ROS 2 Action Integration

### Voice Command to ROS 2 Actions

Connecting voice processing to ROS 2 actions requires a bridge node:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from typing import Dict, Any

class VoiceActionBridge(Node):
    def __init__(self):
        super().__init__('voice_action_bridge')

        # Subscribe to parsed commands
        self.command_sub = self.create_subscription(
            String,
            'parsed_command',
            self.command_callback,
            10
        )

        # Publishers for different action types
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Action clients
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        # Initialize command handlers
        self.command_handlers = {
            'navigate': self.handle_navigate,
            'move_direction': self.handle_move_direction,
            'pick_up': self.handle_pick_up,
            'place': self.handle_place,
            'look_at': self.handle_look_at,
            'follow': self.handle_follow
        }

        self.get_logger().info("Voice Action Bridge Initialized")

    def command_callback(self, msg):
        """Process parsed command and execute corresponding action"""
        try:
            # Assuming the message contains JSON with command structure
            import json
            command_data = json.loads(msg.data)

            action = command_data.get('action')
            target = command_data.get('target', '')
            attributes = command_data.get('attributes', {})
            confidence = command_data.get('confidence', 0.5)

            # Check confidence threshold
            if confidence < 0.5:
                self.get_logger().warn(f"Command confidence too low: {confidence}")
                return

            # Execute command if handler exists
            if action in self.command_handlers:
                self.command_handlers[action](target, attributes)
            else:
                self.get_logger().warn(f"No handler for action: {action}")

        except json.JSONDecodeError:
            self.get_logger().error("Invalid JSON in command message")
        except Exception as e:
            self.get_logger().error(f"Error processing command: {e}")

    def handle_navigate(self, target: str, attributes: Dict[str, Any]):
        """Handle navigation commands"""
        self.get_logger().info(f"Navigating to: {target}")

        # In a real implementation, this would look up the location
        # and send a navigation goal to move_base
        goal = MoveBaseGoal()

        # Example: Convert target description to coordinates
        # This would require a map of named locations
        if target == "kitchen":
            goal.target_pose.pose.position.x = 5.0
            goal.target_pose.pose.position.y = 3.0
            goal.target_pose.pose.orientation.w = 1.0
        elif target == "living room":
            goal.target_pose.pose.position.x = 2.0
            goal.target_pose.pose.position.y = 1.0
            goal.target_pose.pose.orientation.w = 1.0
        else:
            self.get_logger().warn(f"Unknown location: {target}")
            return

        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = self.get_clock().now().to_msg()

        # Send goal to navigation system
        self.move_base_client.send_goal(goal)

    def handle_move_direction(self, direction: str, attributes: Dict[str, Any]):
        """Handle directional movement commands"""
        twist = Twist()

        distance = attributes.get('distance', 1.0)  # Default 1 meter

        if direction in ['forward', 'front']:
            twist.linear.x = 0.5  # Move forward at 0.5 m/s
        elif direction in ['backward', 'back']:
            twist.linear.x = -0.5
        elif direction in ['left', 'port']:
            twist.angular.z = 0.5  # Turn left
        elif direction in ['right', 'starboard']:
            twist.angular.z = -0.5  # Turn right
        elif direction == 'up':
            twist.linear.z = 0.5  # Move up (for flying robots)
        elif direction == 'down':
            twist.linear.z = -0.5  # Move down (for flying robots)

        # Move for specified distance (simplified)
        duration = distance / 0.5  # Assuming 0.5 m/s
        self.execute_movement(twist, duration)

    def execute_movement(self, twist: Twist, duration: float):
        """Execute movement command for specified duration"""
        self.cmd_vel_pub.publish(twist)

        # Stop after duration
        timer = self.create_timer(duration, lambda: self.stop_robot())

    def stop_robot(self):
        """Stop robot movement"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def handle_pick_up(self, target: str, attributes: Dict[str, Any]):
        """Handle pick up commands"""
        self.get_logger().info(f"Attempting to pick up: {target}")

        # In a real implementation, this would:
        # 1. Locate the object using perception
        # 2. Plan grasp trajectory
        # 3. Execute manipulation action
        # 4. Verify grasp success
        pass

    def handle_place(self, target: str, attributes: Dict[str, Any]):
        """Handle place commands"""
        self.get_logger().info(f"Attempting to place {target}")

        # In a real implementation, this would:
        # 1. Locate placement location
        # 2. Plan placement trajectory
        # 3. Execute manipulation action
        # 4. Release object
        pass

    def handle_look_at(self, target: str, attributes: Dict[str, Any]):
        """Handle look at commands"""
        self.get_logger().info(f"Looking at: {target}")

        # In a real implementation, this would:
        # 1. Locate target in visual space
        # 2. Calculate pan/tilt angles
        # 3. Control head/pan-tilt mechanism
        pass

    def handle_follow(self, target: str, attributes: Dict[str, Any]):
        """Handle follow commands"""
        self.get_logger().info(f"Following: {target}")

        # In a real implementation, this would:
        # 1. Track target using perception
        # 2. Maintain following distance
        # 3. Navigate around obstacles
        pass

def main(args=None):
    rclpy.init(args=args)

    bridge = VoiceActionBridge()

    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Voice Command Validation and Error Handling

### Confidence-Based Validation

Implementing validation based on confidence scores:

```python
class VoiceCommandValidator:
    def __init__(self):
        self.min_confidence = 0.6
        self.similarity_threshold = 0.8

    def validate_command(self, parsed_command: ParsedCommand) -> bool:
        """Validate if command should be executed"""
        # Check confidence threshold
        if parsed_command.confidence < self.min_confidence:
            return False

        # Check if command is meaningful
        if not self.is_meaningful_command(parsed_command):
            return False

        # Check for potentially dangerous commands
        if self.is_potentially_dangerous(parsed_command):
            return False

        return True

    def is_meaningful_command(self, command: ParsedCommand) -> bool:
        """Check if command has meaningful content"""
        if command.action == 'unknown':
            return False

        if command.action == 'speak' and len(command.target.strip()) == 0:
            return False

        return True

    def is_potentially_dangerous(self, command: ParsedCommand) -> bool:
        """Check if command might be dangerous"""
        dangerous_keywords = [
            'attack', 'hurt', 'break', 'destroy', 'damage',
            'jump off', 'fall', 'explode', 'fire'
        ]

        command_text = f"{command.action} {command.target}".lower()

        for keyword in dangerous_keywords:
            if keyword in command_text:
                return True

        return False

    def suggest_correction(self, command_text: str) -> str:
        """Suggest corrections for misunderstood commands"""
        # Common corrections for similar-sounding words
        corrections = {
            'tree': 'three',
            'for': 'four',
            'to': 'two',
            'won': 'one',
            'too': 'two',
            'there': 'three',
            'toe': 'two'
        }

        corrected = command_text
        for wrong, right in corrections.items():
            corrected = corrected.replace(wrong, right)

        return corrected
```

### Feedback and Confirmation System

Providing feedback to users about command execution:

```python
class VoiceFeedbackSystem:
    def __init__(self):
        self.confirmation_required_actions = ['pick_up', 'place', 'navigate', 'follow']

    def provide_feedback(self, command: ParsedCommand, node):
        """Provide feedback about command processing"""
        if command.confidence < 0.7:
            node.get_logger().warn(f"Low confidence command: {command.confidence:.2f}")

        if command.action in self.confirmation_required_actions:
            self.request_confirmation(command, node)
        else:
            self.acknowledge_command(command, node)

    def request_confirmation(self, command: ParsedCommand, node):
        """Request user confirmation for critical commands"""
        feedback_msg = f"I understood: {self.format_command(command)}. Should I proceed?"
        node.get_logger().info(feedback_msg)

        # In a real system, this would use text-to-speech
        # and wait for user confirmation

    def acknowledge_command(self, command: ParsedCommand, node):
        """Acknowledge command receipt"""
        feedback_msg = f"Executing: {self.format_command(command)}"
        node.get_logger().info(feedback_msg)

        # In a real system, this would use text-to-speech

    def format_command(self, command: ParsedCommand) -> str:
        """Format command for user feedback"""
        if command.attributes:
            attrs = ', '.join([f"{k}={v}" for k, v in command.attributes.items()])
            return f"{command.action} {command.target} ({attrs})"
        else:
            return f"{command.action} {command.target}"
```

## Performance Optimization

### Real-Time Processing Considerations

For real-time voice processing in robotics:

```python
import threading
import queue
from collections import deque

class RealTimeVoiceProcessor:
    def __init__(self):
        self.audio_queue = queue.Queue(maxsize=10)
        self.command_queue = queue.Queue(maxsize=5)

        # Buffer for continuous audio processing
        self.audio_buffer = deque(maxlen=48000)  # 3 seconds at 16kHz

        # Processing thread
        self.processing_thread = threading.Thread(target=self.process_audio_stream)
        self.processing_running = True

    def start_processing(self):
        """Start real-time audio processing"""
        self.processing_thread.start()

    def stop_processing(self):
        """Stop real-time audio processing"""
        self.processing_running = False
        self.processing_thread.join()

    def process_audio_stream(self):
        """Continuously process audio stream"""
        while self.processing_running:
            try:
                # Get audio chunk from queue
                audio_chunk = self.audio_queue.get(timeout=0.1)

                # Add to buffer
                self.audio_buffer.extend(audio_chunk)

                # Check if we have enough audio for processing
                if len(self.audio_buffer) >= 16000:  # 1 second of audio
                    # Process the buffered audio
                    self.process_audio_batch(list(self.audio_buffer))

                    # Clear buffer to avoid accumulation
                    self.audio_buffer.clear()

            except queue.Empty:
                continue
            except Exception as e:
                print(f"Error in audio processing: {e}")

    def process_audio_batch(self, audio_data):
        """Process a batch of audio data"""
        # This would call Whisper or other speech recognition
        # and then command parsing
        pass
```

## Practice Exercise

Create a complete voice-to-action pipeline that:
1. Records audio from a microphone
2. Processes it with Whisper for speech recognition
3. Parses the recognized text into structured commands
4. Validates commands for safety and meaning
5. Executes appropriate ROS 2 actions
6. Provides feedback to the user

Test your pipeline with various voice commands and evaluate its accuracy and responsiveness.

## Summary

Voice-to-action pipelines form a critical component of Vision-Language-Action systems, enabling natural human-robot interaction. By combining robust speech recognition with intelligent command parsing and safe action execution, we can create intuitive interfaces that allow users to control robots through natural language. The integration with ROS 2 provides a standardized framework for connecting voice processing to robot actions, making it easier to develop and deploy voice-controlled robotic systems.