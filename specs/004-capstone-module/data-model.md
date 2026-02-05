# Module 4: Vision-Language-Action (VLA) Capstone - Data Model

## Overview
This document defines the data structures and models used in the Vision-Language-Action (VLA) capstone module.

## Voice Command Data Model
```yaml
VoiceCommand:
  id: string
  timestamp: datetime
  audio_data: bytes
  transcription: string
  intent_classification: string
  confidence_score: float
  parsed_parameters: dict
  execution_status: enum['pending', 'processing', 'success', 'failed']
```

## LLM Request/Response Model
```yaml
LLMRequest:
  id: string
  context: dict
  query: string
  available_actions: list
  timestamp: datetime

LLMResponse:
  id: string
  generated_plan: list
  confidence_score: float
  reasoning_trace: string
  estimated_execution_time: float
  risk_assessment: dict
```

## Action Sequence Model
```yaml
ActionSequence:
  id: string
  steps: list<ActionStep>
  priority: int
  estimated_duration: float
  required_resources: list<string>
  preconditions: list<Condition>
  postconditions: list<Condition>

ActionStep:
  id: string
  action_type: enum['navigation', 'manipulation', 'perception', 'communication']
  parameters: dict
  timeout: float
  success_criteria: Condition
```

## Perception Context Model
```yaml
PerceptionContext:
  timestamp: datetime
  objects_detected: list<ObjectInfo>
  environment_state: EnvironmentState
  robot_state: RobotState
  task_context: TaskContext

ObjectInfo:
  id: string
  class: string
  position: Vector3
  confidence: float
  properties: dict

EnvironmentState:
  occupancy_grid: OccupancyGrid
  lighting_conditions: LightingInfo
  obstacles: list<Obstacle>
```

## Voice Feedback Model
```yaml
VoiceFeedback:
  id: string
  message: string
  emotion: enum['neutral', 'positive', 'cautious', 'error']
  priority: int
  timestamp: datetime
  spoken: boolean
```

## Capstone Scenario Model
```yaml
CapstoneScenario:
  id: string
  name: string
  description: string
  objectives: list<ScenarioObjective>
  initial_conditions: dict
  success_criteria: list<Condition>
  duration_limit: float

ScenarioObjective:
  id: string
  description: string
  success_criteria: Condition
  weight: float
```

## Integration Models
These models connect with previous modules:
- Inherits RobotState from Module 3 (Isaac AI Brain)
- Uses Navigation goals from Module 3
- Incorporates ROS 2 message formats from Module 1
- Leverages simulation environments from Module 2