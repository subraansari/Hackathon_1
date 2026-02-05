---
sidebar_position: 3
title: "LLM-Based Cognitive Planning"
---

# LLM-Based Cognitive Planning

## Learning Objectives

After completing this chapter, you will be able to:
- Integrate Large Language Models (LLMs) with ROS 2 for cognitive planning
- Translate natural language instructions into ROS 2 action sequences
- Implement task decomposition algorithms for complex robot behaviors
- Design safe and reliable LLM-based planning systems
- Validate and verify LLM-generated action plans
- Handle errors and uncertainties in LLM-based planning

## Introduction

Large Language Models (LLMs) have emerged as powerful tools for cognitive planning in robotics, bridging the gap between high-level natural language instructions and low-level robot actions. In the context of Vision-Language-Action (VLA) systems, LLMs serve as the cognitive layer that interprets human intentions, decomposes complex tasks into manageable subtasks, and generates executable action sequences for robots.

This chapter explores how to effectively integrate LLMs with ROS 2 to create intelligent cognitive planning systems for humanoid robots. We'll cover the architecture, implementation patterns, and best practices for building reliable LLM-based planning systems.

## LLM Integration Architecture

### Cognitive Planning Pipeline

The LLM-based cognitive planning system follows this pipeline:

```
Natural Language Instruction
         ↓
    LLM Processing
         ↓
  Task Decomposition
         ↓
 Action Sequence Generation
         ↓
   Plan Validation
         ↓
   ROS 2 Execution
```

### System Components

#### 1. Language Understanding Module
- Interprets natural language instructions
- Extracts key information (objects, locations, actions)
- Determines intent and context

#### 2. Task Decomposition Engine
- Breaks complex tasks into primitive actions
- Plans subtask dependencies and sequencing
- Handles resource allocation and constraints

#### 3. Action Mapping System
- Maps high-level actions to ROS 2 action servers
- Generates appropriate ROS 2 messages and parameters
- Handles action preconditions and postconditions

#### 4. Plan Validation Module
- Validates action sequences for safety and feasibility
- Checks for conflicts and constraints
- Ensures plan executability

### ROS 2 Integration Patterns

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
import openai
import json
from typing import List, Dict, Any
from dataclasses import dataclass

@dataclass
class CognitivePlan:
    """Structure representing a cognitive plan from LLM"""
    original_instruction: str
    decomposed_tasks: List[Dict[str, Any]]
    estimated_duration: float
    confidence: float
    dependencies: Dict[str, List[str]]  # task_id -> list of prerequisite task_ids

@dataclass
class Task:
    """Individual task in the plan"""
    task_id: str
    action_type: str  # 'navigation', 'manipulation', 'perception', 'communication'
    parameters: Dict[str, Any]
    preconditions: List[str]
    postconditions: List[str]
    estimated_time: float

class LLMCognitivePlannerNode(Node):
    def __init__(self):
        super().__init__('llm_cognitive_planner')

        # LLM client setup
        self.llm_client = openai.OpenAI(api_key=self.get_parameter_or('openai_api_key', '').value)

        # Subscribe to natural language instructions
        self.instruction_sub = self.create_subscription(
            String,
            'natural_language_instruction',
            self.instruction_callback,
            10
        )

        # Publisher for cognitive plans
        self.plan_pub = self.create_publisher(
            String,  # Will publish JSON-serialized plans
            'cognitive_plan',
            10
        )

        # Action client for plan execution
        self.action_clients = {}  # Will store clients for different action types

        # Configuration parameters
        self.declare_parameter('max_plan_length', 10)
        self.declare_parameter('confidence_threshold', 0.7)
        self.declare_parameter('llm_model', 'gpt-4-turbo')

        self.max_plan_length = self.get_parameter('max_plan_length').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.llm_model = self.get_parameter('llm_model').value

        self.get_logger().info("LLM Cognitive Planner Node Initialized")

    def instruction_callback(self, msg):
        """Process natural language instruction and generate cognitive plan"""
        instruction = msg.data

        self.get_logger().info(f"Processing instruction: {instruction}")

        try:
            # Generate cognitive plan using LLM
            cognitive_plan = self.generate_cognitive_plan(instruction)

            if cognitive_plan and cognitive_plan.confidence >= self.confidence_threshold:
                # Validate the plan
                if self.validate_plan(cognitive_plan):
                    # Publish the plan
                    plan_json = json.dumps({
                        'original_instruction': cognitive_plan.original_instruction,
                        'tasks': [self.task_to_dict(task) for task in cognitive_plan.decomposed_tasks],
                        'estimated_duration': cognitive_plan.estimated_duration,
                        'confidence': cognitive_plan.confidence,
                        'dependencies': cognitive_plan.dependencies
                    })

                    plan_msg = String()
                    plan_msg.data = plan_json
                    self.plan_pub.publish(plan_msg)

                    self.get_logger().info(f"Published cognitive plan with {len(cognitive_plan.decomposed_tasks)} tasks")
                else:
                    self.get_logger().error("Generated plan failed validation")
            else:
                self.get_logger().warn(f"Plan confidence too low: {cognitive_plan.confidence if cognitive_plan else 'None'}")

        except Exception as e:
            self.get_logger().error(f"Error processing instruction: {e}")

    def generate_cognitive_plan(self, instruction: str) -> CognitivePlan:
        """Generate cognitive plan from natural language instruction using LLM"""
        try:
            # Define the expected response format
            response_format = {
                "type": "json_object",
                "schema": {
                    "type": "object",
                    "properties": {
                        "decomposed_tasks": {
                            "type": "array",
                            "items": {
                                "type": "object",
                                "properties": {
                                    "task_id": {"type": "string"},
                                    "action_type": {"type": "string", "enum": ["navigation", "manipulation", "perception", "communication"]},
                                    "parameters": {"type": "object"},
                                    "preconditions": {"type": "array", "items": {"type": "string"}},
                                    "postconditions": {"type": "array", "items": {"type": "string"}},
                                    "estimated_time": {"type": "number"}
                                },
                                "required": ["task_id", "action_type", "parameters"]
                            }
                        },
                        "estimated_duration": {"type": "number"},
                        "confidence": {"type": "number"},
                        "dependencies": {"type": "object", "additionalProperties": {"type": "array", "items": {"type": "string"}}}
                    },
                    "required": ["decomposed_tasks", "estimated_duration", "confidence"]
                }
            }

            response = self.client.chat.completions.create(
                model=self.llm_model,
                messages=[
                    {
                        "role": "system",
                        "content": """You are a cognitive planning assistant for humanoid robots.
                        Decompose natural language instructions into executable robot tasks.
                        Each task should be a primitive action that can be executed by a humanoid robot.
                        Consider the robot's capabilities and limitations.
                        Return the plan as valid JSON."""
                    },
                    {
                        "role": "user",
                        "content": f"Decompose this instruction into robot tasks: '{instruction}'. "
                                  "Consider that this is for a humanoid robot with navigation, manipulation, perception, and communication capabilities."
                    }
                ],
                response_format=response_format,
                temperature=0.3
            )

            plan_data = json.loads(response.choices[0].message.content)

            # Convert to CognitivePlan object
            tasks = []
            for task_data in plan_data['decomposed_tasks']:
                task = Task(
                    task_id=task_data['task_id'],
                    action_type=task_data['action_type'],
                    parameters=task_data.get('parameters', {}),
                    preconditions=task_data.get('preconditions', []),
                    postconditions=task_data.get('postconditions', []),
                    estimated_time=task_data.get('estimated_time', 1.0)
                )
                tasks.append(task)

            return CognitivePlan(
                original_instruction=instruction,
                decomposed_tasks=tasks,
                estimated_duration=plan_data['estimated_duration'],
                confidence=plan_data['confidence'],
                dependencies=plan_data.get('dependencies', {})
            )

        except Exception as e:
            self.get_logger().error(f"Error generating cognitive plan: {e}")
            return None

    def validate_plan(self, plan: CognitivePlan) -> bool:
        """Validate the cognitive plan for safety and feasibility"""
        # Check plan length
        if len(plan.decomposed_tasks) > self.max_plan_length:
            self.get_logger().error(f"Plan too long: {len(plan.decomposed_tasks)} tasks (max: {self.max_plan_length})")
            return False

        # Check for circular dependencies
        if self.has_circular_dependencies(plan.dependencies):
            self.get_logger().error("Plan has circular dependencies")
            return False

        # Check task feasibility
        for task in plan.decomposed_tasks:
            if not self.is_task_feasible(task):
                self.get_logger().error(f"Task not feasible: {task.task_id}")
                return False

        # Check safety constraints
        for task in plan.decomposed_tasks:
            if not self.is_task_safe(task):
                self.get_logger().error(f"Task not safe: {task.task_id}")
                return False

        return True

    def has_circular_dependencies(self, dependencies: Dict[str, List[str]]) -> bool:
        """Check if there are circular dependencies in the task graph"""
        visited = set()
        rec_stack = set()

        def dfs(node):
            visited.add(node)
            rec_stack.add(node)

            for neighbor in dependencies.get(node, []):
                if neighbor not in visited:
                    if dfs(neighbor):
                        return True
                elif neighbor in rec_stack:
                    return True

            rec_stack.remove(node)
            return False

        for node in dependencies:
            if node not in visited:
                if dfs(node):
                    return True

        return False

    def is_task_feasible(self, task: Task) -> bool:
        """Check if a task is feasible given robot capabilities"""
        # Check if action type is supported
        supported_actions = ['navigation', 'manipulation', 'perception', 'communication']
        if task.action_type not in supported_actions:
            return False

        # Check parameter validity
        if task.action_type == 'navigation':
            if 'target_pose' not in task.parameters:
                return False
        elif task.action_type == 'manipulation':
            if 'object_name' not in task.parameters or 'action' not in task.parameters:
                return False
        elif task.action_type == 'perception':
            if 'target' not in task.parameters:
                return False

        return True

    def is_task_safe(self, task: Task) -> bool:
        """Check if a task is safe to execute"""
        # Check for dangerous actions
        dangerous_keywords = ['attack', 'hurt', 'damage', 'destroy']

        if task.action_type == 'manipulation':
            action = task.parameters.get('action', '').lower()
            for keyword in dangerous_keywords:
                if keyword in action:
                    return False

        # Check for navigation to dangerous locations
        if task.action_type == 'navigation':
            target_location = task.parameters.get('target_location', '').lower()
            dangerous_locations = ['edge', 'cliff', 'dangerous_area']
            for location in dangerous_locations:
                if location in target_location:
                    return False

        return True

    def task_to_dict(self, task: Task) -> Dict[str, Any]:
        """Convert Task object to dictionary for JSON serialization"""
        return {
            'task_id': task.task_id,
            'action_type': task.action_type,
            'parameters': task.parameters,
            'preconditions': task.preconditions,
            'postconditions': task.postconditions,
            'estimated_time': task.estimated_time
        }
```

## Task Decomposition Algorithms

### Hierarchical Task Networks (HTNs)

HTNs provide a structured approach to task decomposition:

```python
from typing import Union, Callable
from enum import Enum

class PrimitiveTask:
    """A primitive task that can be executed directly by the robot"""
    def __init__(self, action_type: str, parameters: Dict[str, Any]):
        self.action_type = action_type
        self.parameters = parameters

class CompoundTask:
    """A compound task that needs to be decomposed further"""
    def __init__(self, name: str, subtasks: List[Union[PrimitiveTask, 'CompoundTask']]):
        self.name = name
        self.subtasks = subtasks

class TaskDecomposer:
    def __init__(self):
        # Define task decomposition rules
        self.decomposition_rules = {
            'clean_room': self.decompose_clean_room,
            'prepare_meal': self.decompose_prepare_meal,
            'assist_person': self.decompose_assist_person,
            'navigate_to_location': self.decompose_navigate_to_location,
            'pick_and_place': self.decompose_pick_and_place
        }

    def decompose_task(self, task_name: str, context: Dict[str, Any]) -> List[PrimitiveTask]:
        """Decompose a high-level task into primitive tasks"""
        if task_name in self.decomposition_rules:
            return self.decomposition_rules[task_name](context)
        else:
            # Use LLM for unknown tasks
            return self.decompose_with_llm(task_name, context)

    def decompose_clean_room(self, context: Dict[str, Any]) -> List[PrimitiveTask]:
        """Decompose 'clean_room' task"""
        tasks = []

        # Find dirty objects
        tasks.append(PrimitiveTask('perception', {
            'target': 'dirty_objects',
            'area': context.get('room_area', 'all')
        }))

        # Navigate to each dirty object
        for obj in context.get('dirty_objects', []):
            tasks.append(PrimitiveTask('navigation', {
                'target_pose': obj['pose']
            }))

            # Clean the object
            tasks.append(PrimitiveTask('manipulation', {
                'action': 'clean',
                'object_name': obj['name']
            }))

        # Return to home position
        tasks.append(PrimitiveTask('navigation', {
            'target_pose': context.get('home_pose', [0, 0, 0])
        }))

        return tasks

    def decompose_prepare_meal(self, context: Dict[str, Any]) -> List[PrimitiveTask]:
        """Decompose 'prepare_meal' task"""
        tasks = []

        # Get meal recipe
        recipe = context.get('recipe', [])

        for step in recipe:
            if step['action'] == 'fetch_ingredient':
                # Navigate to ingredient location
                tasks.append(PrimitiveTask('navigation', {
                    'target_pose': step['location']
                }))

                # Pick up ingredient
                tasks.append(PrimitiveTask('manipulation', {
                    'action': 'pick_up',
                    'object_name': step['ingredient']
                }))

                # Navigate to preparation area
                tasks.append(PrimitiveTask('navigation', {
                    'target_pose': context.get('prep_area', [0, 0, 0])
                }))

                # Place ingredient
                tasks.append(PrimitiveTask('manipulation', {
                    'action': 'place',
                    'object_name': step['ingredient'],
                    'location': context.get('prep_area', [0, 0, 0])
                }))

            elif step['action'] == 'cook':
                # Cook the ingredient
                tasks.append(PrimitiveTask('manipulation', {
                    'action': 'cook',
                    'object_name': step['ingredient'],
                    'method': step['method']
                }))

        return tasks

    def decompose_with_llm(self, task_name: str, context: Dict[str, Any]) -> List[PrimitiveTask]:
        """Use LLM to decompose unknown tasks"""
        # This would call the LLM to generate a decomposition
        # For demonstration, returning a generic task list
        return [
            PrimitiveTask('perception', {'target': task_name}),
            PrimitiveTask('navigation', {'target_pose': [0, 0, 0]}),
            PrimitiveTask('manipulation', {'action': 'generic', 'object_name': task_name})
        ]

# Example usage
decomposer = TaskDecomposer()

context = {
    'dirty_objects': [
        {'name': 'cup', 'pose': [1.0, 2.0, 0.0]},
        {'name': 'plate', 'pose': [1.5, 2.5, 0.0]}
    ],
    'room_area': 'kitchen',
    'home_pose': [0, 0, 0]
}

primitive_tasks = decomposer.decompose_task('clean_room', context)
print(f"Decomposed into {len(primitive_tasks)} primitive tasks")
```

### Sequential vs Parallel Task Execution

Consider when tasks can be executed in parallel:

```python
import asyncio
from typing import Set

class TaskScheduler:
    def __init__(self):
        self.running_tasks = set()
        self.completed_tasks = set()
        self.task_dependencies = {}  # task_id -> set of prerequisite task_ids

    def add_task_with_dependencies(self, task_id: str, dependencies: Set[str] = None):
        """Add a task with its dependencies"""
        if dependencies is None:
            dependencies = set()
        self.task_dependencies[task_id] = dependencies

    def get_ready_tasks(self) -> Set[str]:
        """Get tasks that are ready to execute (dependencies satisfied)"""
        ready_tasks = set()

        for task_id, deps in self.task_dependencies.items():
            if task_id not in self.completed_tasks and task_id not in self.running_tasks:
                if deps.issubset(self.completed_tasks):
                    ready_tasks.add(task_id)

        return ready_tasks

    async def execute_plan_parallel(self, tasks: List[Task], executor_func: Callable):
        """Execute tasks with parallel execution where possible"""
        # Initialize scheduler
        for task in tasks:
            self.add_task_with_dependencies(task.task_id, set(task.preconditions))

        # Execute tasks
        while len(self.completed_tasks) < len(tasks):
            ready_tasks = self.get_ready_tasks()

            if not ready_tasks:
                # Deadlock - no tasks can run
                raise RuntimeError("Task deadlock detected - circular dependencies?")

            # Execute ready tasks in parallel
            running_coroutines = []
            for task_id in ready_tasks:
                self.running_tasks.add(task_id)
                task = next(t for t in tasks if t.task_id == task_id)

                coroutine = self.execute_single_task(task, executor_func)
                running_coroutines.append(coroutine)

            # Wait for all currently running tasks to complete
            results = await asyncio.gather(*running_coroutines, return_exceptions=True)

            # Update completed tasks
            for i, task_id in enumerate(ready_tasks):
                if not isinstance(results[i], Exception):
                    self.completed_tasks.add(task_id)
                self.running_tasks.remove(task_id)

                if isinstance(results[i], Exception):
                    self.get_logger().error(f"Task {task_id} failed: {results[i]}")

    async def execute_single_task(self, task: Task, executor_func: Callable):
        """Execute a single task using the executor function"""
        try:
            # Wait for preconditions
            for precondition in task.preconditions:
                while precondition not in self.completed_tasks:
                    await asyncio.sleep(0.1)

            # Execute the task
            await executor_func(task)

            # Mark postconditions as satisfied
            for postcondition in task.postconditions:
                # In a real system, this would update the world state
                pass

        except Exception as e:
            raise e
```

## Natural Language to ROS 2 Action Mapping

### Action Mapping System

Map high-level language concepts to specific ROS 2 actions:

```python
class ActionMapper:
    def __init__(self):
        # Define mappings from natural language to ROS 2 actions
        self.action_mappings = {
            # Navigation actions
            'go_to': {
                'ros_action': 'move_base_msgs/MoveBase',
                'parameter_mapping': {
                    'location': 'target_pose.pose.position',
                    'orientation': 'target_pose.pose.orientation'
                }
            },
            'navigate_to': {
                'ros_action': 'nav2_msgs/NavigateToPose',
                'parameter_mapping': {
                    'location': 'pose.pose.position',
                    'orientation': 'pose.pose.orientation'
                }
            },
            'move_forward': {
                'ros_action': 'geometry_msgs/Twist',
                'parameter_mapping': {
                    'distance': 'linear.x',
                    'speed': 'linear.x'
                }
            },

            # Manipulation actions
            'pick_up': {
                'ros_action': 'control_msgs/FollowJointTrajectory',
                'parameter_mapping': {
                    'object_name': 'object_name',
                    'grasp_pose': 'trajectory.points[-1].positions'
                }
            },
            'place': {
                'ros_action': 'control_msgs/FollowJointTrajectory',
                'parameter_mapping': {
                    'object_name': 'object_name',
                    'placement_pose': 'trajectory.points[-1].positions'
                }
            },
            'grasp': {
                'ros_action': 'control_msgs/GripperCommand',
                'parameter_mapping': {
                    'width': 'command.position',
                    'effort': 'command.max_effort'
                }
            },

            # Perception actions
            'look_at': {
                'ros_action': 'control_msgs/PointHead',
                'parameter_mapping': {
                    'target': 'target_point',
                    'frame': 'pointing_frame'
                }
            },
            'detect': {
                'ros_action': 'object_detection_msgs/DetectObjects',
                'parameter_mapping': {
                    'object_class': 'object_classes',
                    'confidence_threshold': 'min_confidence'
                }
            },

            # Communication actions
            'speak': {
                'ros_action': 'sound_play/SoundRequest',
                'parameter_mapping': {
                    'text': 'sound_request.arg2',
                    'volume': 'sound_request.volume'
                }
            }
        }

        # Synonym mappings
        self.synonyms = {
            'go_to': ['navigate_to', 'move_to', 'travel_to', 'go_to_location'],
            'pick_up': ['grasp', 'take', 'pickup', 'catch'],
            'place': ['put', 'set_down', 'release', 'position'],
            'look_at': ['face', 'turn_to', 'orient_to'],
            'detect': ['find', 'locate', 'identify', 'spot']
        }

    def map_language_to_action(self, action_word: str, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """Map natural language action to ROS 2 action with parameters"""
        # Normalize action word
        normalized_action = self.normalize_action_word(action_word)

        if normalized_action not in self.action_mappings:
            return None

        mapping = self.action_mappings[normalized_action]

        # Map parameters according to the mapping
        ros_parameters = {}
        for param_name, ros_path in mapping['parameter_mapping'].items():
            if param_name in parameters:
                # This is a simplified parameter mapping
                # In practice, you'd need more sophisticated path resolution
                ros_parameters[param_name] = parameters[param_name]

        return {
            'ros_action': mapping['ros_action'],
            'parameters': ros_parameters,
            'original_action': action_word,
            'normalized_action': normalized_action
        }

    def normalize_action_word(self, action_word: str) -> str:
        """Normalize action word by checking synonyms"""
        action_word_lower = action_word.lower()

        for canonical, synonyms in self.synonyms.items():
            if action_word_lower in synonyms or action_word_lower == canonical:
                return canonical

        return action_word_lower

    def validate_action_parameters(self, action_type: str, parameters: Dict[str, Any]) -> bool:
        """Validate that required parameters are present for the action"""
        required_params = self.get_required_parameters(action_type)

        for param in required_params:
            if param not in parameters:
                return False

        return True

    def get_required_parameters(self, action_type: str) -> List[str]:
        """Get required parameters for an action type"""
        required_params = {
            'move_base_msgs/MoveBase': ['target_pose'],
            'nav2_msgs/NavigateToPose': ['pose'],
            'geometry_msgs/Twist': ['linear', 'angular'],
            'control_msgs/FollowJointTrajectory': ['trajectory'],
            'control_msgs/GripperCommand': ['command'],
            'control_msgs/PointHead': ['target_point', 'pointing_frame'],
            'object_detection_msgs/DetectObjects': ['object_classes'],
            'sound_play/SoundRequest': ['sound_request']
        }

        return required_params.get(action_type, [])
```

### Safety and Validation Layer

Implement safety checks for LLM-generated plans:

```python
class SafetyValidator:
    def __init__(self):
        # Define safety rules
        self.safety_rules = [
            self.no_dangerous_actions,
            self.no_invalid_coordinates,
            self.no_resource_conflicts,
            self.no_unreachable_locations,
            self.no_self_collision
        ]

        # Dangerous action keywords
        self.dangerous_actions = [
            'attack', 'hurt', 'damage', 'destroy', 'break',
            'jump_off', 'fall', 'explode', 'ignite', 'electrocute'
        ]

        # Known safe zones
        self.safe_zones = set()  # Would be populated from environment map

    def validate_plan(self, plan: CognitivePlan) -> tuple[bool, List[str]]:
        """Validate a cognitive plan against safety rules"""
        violations = []

        for rule in self.safety_rules:
            rule_passed, rule_violations = rule(plan)
            if not rule_passed:
                violations.extend(rule_violations)

        return len(violations) == 0, violations

    def no_dangerous_actions(self, plan: CognitivePlan) -> tuple[bool, List[str]]:
        """Check for dangerous actions"""
        violations = []

        for task in plan.decomposed_tasks:
            action_lower = task.action_type.lower()

            if action_lower in self.dangerous_actions:
                violations.append(f"Dangerous action detected: {task.action_type}")

            # Check parameters for dangerous keywords
            for param_value in task.parameters.values():
                if isinstance(param_value, str):
                    for danger_word in self.dangerous_actions:
                        if danger_word in param_value.lower():
                            violations.append(f"Dangerous keyword in parameter: {param_value}")

        return len(violations) == 0, violations

    def no_invalid_coordinates(self, plan: CognitivePlan) -> tuple[bool, List[str]]:
        """Check for invalid coordinates"""
        violations = []

        for task in plan.decomposed_tasks:
            if task.action_type == 'navigation':
                pose = task.parameters.get('target_pose')
                if pose:
                    # Check if coordinates are reasonable
                    if isinstance(pose, (list, tuple)) and len(pose) >= 2:
                        x, y = pose[0], pose[1]

                        # Check for extremely large coordinates (likely invalid)
                        if abs(x) > 1000 or abs(y) > 1000:
                            violations.append(f"Invalid coordinates in navigation task: {pose}")

                        # Check for NaN or infinity
                        if any(not (float('-inf') < val < float('inf')) for val in pose if isinstance(val, (int, float))):
                            violations.append(f"Invalid coordinate values (NaN/Inf) in navigation task: {pose}")

        return len(violations) == 0, violations

    def no_resource_conflicts(self, plan: CognitivePlan) -> tuple[bool, List[str]]:
        """Check for resource conflicts in the plan"""
        violations = []

        # Check for simultaneous use of shared resources
        used_resources = {}

        for task in plan.decomposed_tasks:
            # Example: check if multiple manipulation tasks use the same arm simultaneously
            if task.action_type == 'manipulation':
                arm = task.parameters.get('arm', 'both')
                time_slot = task.parameters.get('time_slot', 0)

                if (arm, time_slot) in used_resources:
                    violations.append(f"Resource conflict: {arm} arm requested at time slot {time_slot}")
                else:
                    used_resources[(arm, time_slot)] = task.task_id

        return len(violations) == 0, violations

    def no_unreachable_locations(self, plan: CognitivePlan) -> tuple[bool, List[str]]:
        """Check for unreachable locations"""
        violations = []

        # In a real system, this would check against the robot's map
        # and kinematic constraints
        for task in plan.decomposed_tasks:
            if task.action_type == 'navigation':
                target = task.parameters.get('target_pose')
                if target:
                    # This would normally check against occupancy grid or navigation map
                    # For now, just a placeholder check
                    pass

        return len(violations) == 0, violations

    def no_self_collision(self, plan: CognitivePlan) -> tuple[bool, List[str]]:
        """Check for potential self-collision in manipulation tasks"""
        violations = []

        # In a real system, this would check joint limits and collision models
        for task in plan.decomposed_tasks:
            if task.action_type == 'manipulation':
                joint_positions = task.parameters.get('joint_positions', [])

                # Check for joint limit violations
                for i, pos in enumerate(joint_positions):
                    # This would check against actual robot joint limits
                    if abs(pos) > 3.14:  # Example: joint limit violation
                        violations.append(f"Potential joint limit violation in manipulation task: joint {i} at {pos}")

        return len(violations) == 0, violations
```

## Prompt Engineering for Robotics

### Effective Prompts for Robot Planning

Crafting effective prompts is crucial for reliable LLM-based planning:

```python
class PromptEngineer:
    def __init__(self):
        self.system_prompts = {
            'basic_planning': """You are a cognitive planning assistant for humanoid robots.
            Decompose high-level instructions into sequences of primitive robot actions.
            Consider the robot's physical capabilities and environmental constraints.
            Each action should be executable by a humanoid robot with navigation,
            manipulation, perception, and communication abilities.""",

            'safety_aware': """You are a safety-aware cognitive planning assistant for humanoid robots.
            Generate action sequences that are not only effective but also safe.
            Avoid dangerous actions and consider potential risks in the environment.
            If uncertain about safety, provide a conservative plan.""",

            'context_aware': """You are a context-aware cognitive planning assistant for humanoid robots.
            Use environmental context and robot state to generate appropriate action sequences.
            Consider the current location, available objects, and task requirements."""
        }

        self.task_templates = {
            'navigation': """Generate a navigation plan to {target_location}.
            Consider obstacles, preferred pathways, and safety constraints.
            Include intermediate waypoints if necessary.""",

            'manipulation': """Generate a manipulation plan to {action} the {object}.
            Consider grasp poses, approach directions, and safety constraints.
            Include pre- and post-manipulation navigation if needed.""",

            'perception': """Generate a perception plan to {task_description}.
            Consider sensor placement, viewing angles, and environmental lighting.
            Specify what to look for and how to verify the results."""
        }

    def create_planning_prompt(self, instruction: str, context: Dict[str, Any] = None,
                              safety_mode: bool = True, context_mode: bool = True) -> str:
        """Create an effective prompt for cognitive planning"""
        prompt_parts = []

        # Select appropriate system prompt
        system_prompt_key = 'basic_planning'
        if safety_mode:
            system_prompt_key = 'safety_aware'
        if context_mode and context:
            system_prompt_key = 'context_aware'

        prompt_parts.append(self.system_prompts[system_prompt_key])

        # Add context information
        if context:
            prompt_parts.append(f"\nContext information:")
            for key, value in context.items():
                prompt_parts.append(f"- {key}: {value}")

        # Add the main instruction
        prompt_parts.append(f"\nInstruction: {instruction}")

        # Add output format requirements
        prompt_parts.append("""
        Output format: Return only valid JSON with the following structure:
        {
            "decomposed_tasks": [
                {
                    "task_id": "unique_identifier",
                    "action_type": "navigation|manipulation|perception|communication",
                    "parameters": {...},
                    "preconditions": [...],
                    "postconditions": [...],
                    "estimated_time": number_seconds
                }
            ],
            "estimated_duration": total_time_seconds,
            "confidence": 0.0_to_1.0,
            "dependencies": {"task_id": ["prerequisite_task_ids"]}
        }
        """)

        return "\n".join(prompt_parts)

    def create_refinement_prompt(self, original_plan: Dict[str, Any],
                                feedback: str, context: Dict[str, Any] = None) -> str:
        """Create a prompt to refine an existing plan based on feedback"""
        prompt_parts = [
            self.system_prompts['safety_aware'],
            f"\nOriginal plan: {json.dumps(original_plan, indent=2)}",
            f"\nFeedback to address: {feedback}"
        ]

        if context:
            prompt_parts.append(f"\nAdditional context: {json.dumps(context, indent=2)}")

        prompt_parts.append("""
        Please refine the plan to address the feedback while maintaining the original intent.
        Return only valid JSON in the same format as the original plan.
        """)

        return "\n".join(prompt_parts)

    def create_verification_prompt(self, plan: Dict[str, Any],
                                 instruction: str) -> str:
        """Create a prompt to verify plan correctness"""
        prompt_parts = [
            """You are a verification assistant for robot cognitive plans.
            Check if the plan correctly addresses the original instruction
            and is feasible for a humanoid robot. Identify any issues.""",
            f"\nOriginal instruction: {instruction}",
            f"\nProposed plan: {json.dumps(plan, indent=2)}",
            """
            Verify the following aspects:
            1. Does the plan address the original instruction?
            2. Are the actions feasible for a humanoid robot?
            3. Are there any safety concerns?
            4. Are the task dependencies logical?
            5. Are the parameters reasonable?

            Return only valid JSON:
            {
                "valid": true/false,
                "issues": ["list of issues if any"],
                "suggestions": ["improvement suggestions if any"],
                "confidence": 0.0_to_1.0
            }
            """
        ]

        return "\n".join(prompt_parts)
```

## Error Handling and Uncertainty Management

### Managing LLM Uncertainty

LLMs can produce uncertain or incorrect outputs that need to be managed:

```python
import numpy as np
from typing import Optional

class UncertaintyManager:
    def __init__(self):
        self.confidence_threshold = 0.7
        self.uncertainty_strategies = {
            'replan': self.handle_replan,
            'ask_user': self.handle_ask_user,
            'fallback': self.handle_fallback,
            'decompose_further': self.handle_decompose_further
        }

    def assess_uncertainty(self, plan: CognitivePlan) -> Dict[str, float]:
        """Assess various types of uncertainty in a plan"""
        uncertainty_metrics = {
            'overall_confidence': plan.confidence,
            'task_complexity': self.calculate_task_complexity(plan.decomposed_tasks),
            'ambiguity_score': self.calculate_ambiguity_score(plan.original_instruction),
            'dependency_risk': self.calculate_dependency_risk(plan.dependencies),
            'action_feasibility': self.calculate_action_feasibility_score(plan.decomposed_tasks)
        }

        return uncertainty_metrics

    def calculate_task_complexity(self, tasks: List[Task]) -> float:
        """Calculate task complexity score (higher = more complex/uncertain)"""
        if not tasks:
            return 0.0

        complexity_factors = []
        for task in tasks:
            # Complexity increases with:
            # - Unknown action types
            # - Complex parameters
            # - Many dependencies

            action_complexity = 0.0
            if task.action_type not in ['navigation', 'manipulation', 'perception', 'communication']:
                action_complexity = 0.8  # High uncertainty for unknown action types
            else:
                action_complexity = 0.2  # Lower uncertainty for known types

            # Parameter complexity
            param_complexity = min(len(task.parameters) * 0.1, 0.5)

            # Dependency complexity
            dep_complexity = min(len(task.preconditions) * 0.05, 0.3)

            complexity_factors.append((action_complexity + param_complexity + dep_complexity) / 3)

        return np.mean(complexity_factors) if complexity_factors else 0.0

    def calculate_ambiguity_score(self, instruction: str) -> float:
        """Calculate ambiguity score for natural language instruction"""
        # Ambiguity indicators
        ambiguity_indicators = [
            'maybe', 'perhaps', 'possibly', 'could', 'might',
            'some', 'certain', 'specific', 'the', 'a'
        ]

        instruction_lower = instruction.lower()
        ambiguity_count = sum(1 for indicator in ambiguity_indicators if indicator in instruction_lower)

        # Also check for vague spatial/temporal references
        vague_refs = ['somewhere', 'anywhere', 'nowhere', 'soon', 'later', 'a bit']
        vague_count = sum(1 for ref in vague_refs if ref in instruction_lower)

        # Calculate normalized ambiguity score
        total_words = len(instruction.split())
        ambiguity_score = (ambiguity_count + vague_count) / max(total_words, 1)

        return min(ambiguity_score, 1.0)  # Clamp to [0, 1]

    def calculate_dependency_risk(self, dependencies: Dict[str, List[str]]) -> float:
        """Calculate risk from task dependencies"""
        if not dependencies:
            return 0.0

        # Risk increases with:
        # - Many dependencies per task
        # - Complex dependency graph
        total_deps = sum(len(deps) for deps in dependencies.values())
        avg_deps_per_task = total_deps / len(dependencies) if dependencies else 0.0

        # Normalize to [0, 1]
        return min(avg_deps_per_task * 0.2, 1.0)

    def calculate_action_feasibility_score(self, tasks: List[Task]) -> float:
        """Calculate feasibility score for actions"""
        if not tasks:
            return 1.0

        feasibility_scores = []
        for task in tasks:
            # Check if action parameters seem reasonable
            if task.action_type == 'navigation':
                target_pose = task.parameters.get('target_pose')
                if target_pose and len(target_pose) >= 2:
                    x, y = target_pose[0], target_pose[1]
                    # Check if coordinates are extremely far (might be wrong)
                    if abs(x) > 100 or abs(y) > 100:
                        feasibility_scores.append(0.2)
                    else:
                        feasibility_scores.append(0.9)
                else:
                    feasibility_scores.append(0.5)  # Unknown, medium risk
            else:
                # For other action types, assume medium feasibility
                feasibility_scores.append(0.7)

        return 1.0 - np.mean(feasibility_scores) if feasibility_scores else 0.5

    def handle_uncertainty(self, plan: CognitivePlan, context: Dict[str, Any]) -> Optional[CognitivePlan]:
        """Handle uncertainty in the plan using appropriate strategy"""
        uncertainty_metrics = self.assess_uncertainty(plan)

        # Overall uncertainty score
        overall_uncertainty = np.mean(list(uncertainty_metrics.values()))

        if overall_uncertainty > (1 - self.confidence_threshold):
            # High uncertainty - need to handle it
            strategy = self.select_strategy(uncertainty_metrics, context)
            return self.uncertainty_strategies[strategy](plan, context, uncertainty_metrics)
        else:
            # Low uncertainty - return original plan
            return plan

    def select_strategy(self, uncertainty_metrics: Dict[str, float],
                       context: Dict[str, Any]) -> str:
        """Select appropriate uncertainty handling strategy"""
        # Choose strategy based on which uncertainty metric is highest
        max_uncertainty_metric = max(uncertainty_metrics, key=uncertainty_metrics.get)

        if uncertainty_metrics['overall_confidence'] < self.confidence_threshold:
            return 'replan'
        elif uncertainty_metrics['ambiguity_score'] > 0.5:
            return 'ask_user'
        elif uncertainty_metrics['task_complexity'] > 0.7:
            return 'decompose_further'
        else:
            return 'fallback'

    def handle_replan(self, plan: CognitivePlan, context: Dict[str, Any],
                     uncertainty_metrics: Dict[str, float]) -> CognitivePlan:
        """Handle uncertainty by replanning"""
        self.get_logger().info("Replanning due to high uncertainty")

        # Try to generate a new plan with a more specific prompt
        # This would call the LLM again with refined instructions
        return plan  # Placeholder - in practice, would call LLM again

    def handle_ask_user(self, plan: CognitivePlan, context: Dict[str, Any],
                       uncertainty_metrics: Dict[str, float]) -> CognitivePlan:
        """Handle uncertainty by asking for clarification"""
        self.get_logger().info("Requesting user clarification due to ambiguity")

        # In a real system, this would prompt the user for clarification
        # For now, return a conservative plan
        return self.create_conservative_plan(plan.original_instruction)

    def handle_fallback(self, plan: CognitivePlan, context: Dict[str, Any],
                       uncertainty_metrics: Dict[str, float]) -> CognitivePlan:
        """Handle uncertainty with a fallback plan"""
        self.get_logger().info("Using fallback plan due to uncertainty")

        # Create a simple, safe fallback plan
        return self.create_fallback_plan(plan.original_instruction)

    def handle_decompose_further(self, plan: CognitivePlan, context: Dict[str, Any],
                               uncertainty_metrics: Dict[str, float]) -> CognitivePlan:
        """Handle uncertainty by further decomposing complex tasks"""
        self.get_logger().info("Further decomposing complex tasks")

        # This would break down complex tasks into simpler subtasks
        return plan  # Placeholder

    def create_conservative_plan(self, instruction: str) -> CognitivePlan:
        """Create a conservative plan for ambiguous instructions"""
        return CognitivePlan(
            original_instruction=instruction,
            decomposed_tasks=[
                Task(
                    task_id="verify_instruction",
                    action_type="communication",
                    parameters={"text": f"Could you clarify: {instruction}?"},
                    preconditions=[],
                    postconditions=["user_clarification_received"],
                    estimated_time=10.0
                )
            ],
            estimated_duration=10.0,
            confidence=0.9,  # High confidence in conservative approach
            dependencies={}
        )

    def create_fallback_plan(self, instruction: str) -> CognitivePlan:
        """Create a fallback plan for uncertain situations"""
        return CognitivePlan(
            original_instruction=instruction,
            decomposed_tasks=[
                Task(
                    task_id="safe_standby",
                    action_type="communication",
                    parameters={"text": f"I'm not sure how to execute: {instruction}. Please provide more specific instructions."},
                    preconditions=[],
                    postconditions=["awaiting_new_instruction"],
                    estimated_time=5.0
                )
            ],
            estimated_duration=5.0,
            confidence=0.95,  # High confidence in safe fallback
            dependencies={}
        )
```

## Practice Exercise

Create a complete LLM-based cognitive planning system that:
1. Takes natural language instructions as input
2. Uses an LLM to decompose tasks into primitive actions
3. Validates the plan for safety and feasibility
4. Maps the actions to ROS 2 action servers
5. Executes the plan with proper error handling
6. Provides feedback on plan execution

Test your system with various natural language instructions and evaluate its performance in terms of accuracy, safety, and user satisfaction.

## Summary

LLM-based cognitive planning represents a significant advancement in human-robot interaction, enabling natural language communication between humans and robots. By carefully designing the integration architecture, implementing robust validation and safety measures, and properly handling uncertainty, we can create intelligent planning systems that translate high-level human intentions into executable robot actions. The key to success lies in balancing the expressive power of LLMs with the reliability and safety requirements of robotic systems.