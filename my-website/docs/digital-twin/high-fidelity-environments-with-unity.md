---
sidebar_position: 3
title: "High-Fidelity Environments with Unity"
---

# High-Fidelity Environments with Unity

## Learning Objectives

After completing this chapter, you will be able to:
- Create high-fidelity environments in Unity for robot simulation
- Understand the advantages of Unity for visual realism and HRI
- Implement Human-Robot Interaction (HRI) features in Unity
- Connect Unity simulations to ROS 2 for comprehensive robot development
- Optimize Unity environments for real-time performance

## Introduction

Unity is a powerful game engine that excels at creating high-fidelity, visually impressive environments for robot simulation. While Gazebo provides excellent physics simulation, Unity complements it by offering superior visual quality, advanced rendering capabilities, and sophisticated Human-Robot Interaction (HRI) interfaces. This chapter explores how to leverage Unity for creating photorealistic environments that enhance the digital twin experience for humanoid robots.

## Unity for Robotics: Why Unity?

### Visual Realism
Unity offers industry-leading rendering capabilities that can create photorealistic environments:

- Physically Based Rendering (PBR) materials
- Advanced lighting systems (real-time/global illumination)
- Post-processing effects
- High-resolution textures and detailed models

### Human-Robot Interaction (HRI)
Unity provides tools for creating intuitive HRI interfaces:

- Advanced UI/UX systems
- VR/AR integration capabilities
- Gesture recognition interfaces
- Interactive 3D environments

### Cross-Platform Support
Unity environments can run on various platforms:

- Desktop (Windows, Mac, Linux)
- VR/AR headsets
- Mobile devices
- Web browsers (with Unity WebGL)

## Unity Robotics Simulation Setup

### Installing Unity for Robotics

To set up Unity for robotics applications:

1. Download Unity Hub from unity.com
2. Install Unity 2022.3 LTS or later
3. Install the Unity Robotics Package
4. Import the ROS-TCP-Connector package

### Unity Robotics Package Features

The Unity Robotics Package provides essential tools:

- **ROS-TCP-Connector**: Enables communication with ROS/ROS2
- **Robotics-Infrared-Range-Sensor**: Simulates distance sensors
- **Robotics-Library**: Contains useful robotics utilities
- **Robotics-NavMesh-Components**: For navigation mesh generation

### Basic Unity Scene Setup

```csharp
using UnityEngine;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;

public class RobotController : MonoBehaviour
{
    private ROSConnection ros;
    public string robotTopic = "robot/cmd_vel";

    void Start()
    {
        // Connect to ROS
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>(robotTopic);
    }

    void Update()
    {
        // Example: Send velocity commands
        if (Input.GetKeyDown(KeyCode.Space))
        {
            var twist = new TwistMsg();
            twist.linear.Set(0.5f, 0, 0); // Move forward
            twist.angular.Set(0, 0, 0.5f); // Rotate
            ros.Publish(robotTopic, twist);
        }
    }
}
```

## Creating High-Fidelity Environments

### Environmental Assets

#### Terrain Generation
Unity's terrain system allows for realistic outdoor environments:

```csharp
using UnityEngine;

public class TerrainGenerator : MonoBehaviour
{
    public int terrainWidth = 1000;
    public int terrainHeight = 1000;
    public float terrainStrength = 10f;

    void Start()
    {
        Terrain terrain = GetComponent<Terrain>();

        // Create heightmap
        float[,] heights = new float[terrainWidth, terrainHeight];

        // Generate procedural terrain
        for (int x = 0; x < terrainWidth; x++)
        {
            for (int y = 0; y < terrainHeight; y++)
            {
                heights[x, y] = Mathf.PerlinNoise(
                    x * 0.01f,
                    y * 0.01f
                ) * terrainStrength;
            }
        }

        terrain.terrainData.SetHeights(0, 0, heights);
    }
}
```

#### Vegetation and Details
Add realistic vegetation with Unity's built-in systems:

```csharp
using UnityEngine;

public class VegetationSpawner : MonoBehaviour
{
    public GameObject[] plantPrefabs;
    public int plantCount = 1000;
    public Terrain terrain;

    void Start()
    {
        SpawnVegetation();
    }

    void SpawnVegetation()
    {
        for (int i = 0; i < plantCount; i++)
        {
            Vector3 pos = new Vector3(
                Random.Range(0, terrain.terrainData.size.x),
                0,
                Random.Range(0, terrain.terrainData.size.z)
            );

            // Get terrain height at position
            pos.y = terrain.SampleHeight(pos);

            GameObject plant = Instantiate(
                plantPrefabs[Random.Range(0, plantPrefabs.Length)],
                pos,
                Quaternion.identity
            );

            // Random rotation and scale
            plant.transform.rotation = Quaternion.Euler(0, Random.Range(0, 360), 0);
            float scale = Random.Range(0.8f, 1.2f);
            plant.transform.localScale = new Vector3(scale, scale, scale);
        }
    }
}
```

### Lighting Systems

#### Realistic Day/Night Cycle
Create dynamic lighting for different times of day:

```csharp
using UnityEngine;

public class DayNightCycle : MonoBehaviour
{
    public Light sun;
    public float secondsInFullDay = 120f;
    [Range(0, 1)] public float currentTimeOfDay = 0f;

    float timeSpeedMultiplier = 1f;

    void Update()
    {
        currentTimeOfDay += (Time.deltaTime / secondsInFullDay) * timeSpeedMultiplier;
        if (currentTimeOfDay > 1) currentTimeOfDay = 0;

        RotateSun();
        SetLightIntensity();
    }

    void RotateSun()
    {
        sun.transform.localRotation = Quaternion.Euler(
            (currentTimeOfDay * 360f) - 90,
            170,
            0
        );
    }

    void SetLightIntensity()
    {
        float intensityMultiplier = 1f;
        if (currentTimeOfDay <= 0.23f || currentTimeOfDay >= 0.77f)
        {
            intensityMultiplier = 0f;
        }
        else if (currentTimeOfDay <= 0.25f)
        {
            intensityMultiplier = Mathf.Clamp01(
                (currentTimeOfDay - 0.23f) * (1 / 0.02f)
            );
        }
        else if (currentTimeOfDay >= 0.75f)
        {
            intensityMultiplier = Mathf.Clamp01(
                (0.77f - currentTimeOfDay) * (1 / 0.02f)
            );
        }

        sun.intensity = 1f * intensityMultiplier;
    }
}
```

#### Atmospheric Scattering
Enhance realism with atmospheric effects:

```csharp
using UnityEngine;

[ExecuteInEditMode]
public class AtmosphericScattering : MonoBehaviour
{
    public Color scatteringColor = Color.cyan;
    public float scatteringIntensity = 1f;
    public float scatteringDistance = 50f;

    void Update()
    {
        RenderSettings.fog = true;
        RenderSettings.fogColor = scatteringColor;
        RenderSettings.fogDensity = scatteringIntensity / scatteringDistance;
        RenderSettings.fogMode = FogMode.ExponentialSquared;
    }
}
```

## Human-Robot Interaction (HRI) Features

### Interactive UI System
Create intuitive interfaces for robot control and monitoring:

```csharp
using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class RobotUIManager : MonoBehaviour
{
    [Header("Robot Status")]
    public TextMeshProUGUI batteryText;
    public Slider batterySlider;
    public TextMeshProUGUI statusText;

    [Header("Controls")]
    public Button moveForwardButton;
    public Button moveBackwardButton;
    public Button rotateLeftButton;
    public Button rotateRightButton;

    [Header("Camera Controls")]
    public Toggle thirdPersonToggle;
    public Slider cameraZoomSlider;

    void Start()
    {
        SetupButtons();
        SetupToggles();
        StartCoroutine(UpdateRobotStatus());
    }

    void SetupButtons()
    {
        moveForwardButton.onClick.AddListener(() => SendCommand("MOVE_FORWARD"));
        moveBackwardButton.onClick.AddListener(() => SendCommand("MOVE_BACKWARD"));
        rotateLeftButton.onClick.AddListener(() => SendCommand("ROTATE_LEFT"));
        rotateRightButton.onClick.AddListener(() => SendCommand("ROTATE_RIGHT"));
    }

    void SetupToggles()
    {
        thirdPersonToggle.onValueChanged.AddListener((bool isThirdPerson) =>
        {
            // Switch camera perspective
            SwitchCameraView(isThirdPerson);
        });
    }

    IEnumerator UpdateRobotStatus()
    {
        while (true)
        {
            // Simulate getting robot status
            float batteryLevel = Random.Range(20f, 100f);
            batterySlider.value = batteryLevel;
            batteryText.text = $"Battery: {batteryLevel:F1}%";

            statusText.text = "Status: Active";

            yield return new WaitForSeconds(1f);
        }
    }

    void SendCommand(string command)
    {
        Debug.Log($"Sending command: {command}");
        // Send command via ROS connection
    }

    void SwitchCameraView(bool isThirdPerson)
    {
        Debug.Log($"Switching to {(isThirdPerson ? "Third Person" : "First Person")} view");
        // Implement camera switch logic
    }
}
```

### Gesture Recognition Interface
Implement gesture-based controls:

```csharp
using UnityEngine;
using UnityEngine.XR;

public class GestureRecognizer : MonoBehaviour
{
    public Camera mainCamera;
    public GameObject gestureIndicator;

    private Vector3 gestureStartPos;
    private bool isGestureActive = false;

    void Update()
    {
        HandleMouseGestures();
        HandleTouchGestures();
    }

    void HandleMouseGestures()
    {
        if (Input.GetMouseButtonDown(0))
        {
            gestureStartPos = Input.mousePosition;
            isGestureActive = true;
            ShowGestureIndicator(true);
        }

        if (Input.GetMouseButtonUp(0) && isGestureActive)
        {
            Vector3 gestureEndPos = Input.mousePosition;
            Vector3 gestureVector = gestureEndPos - gestureStartPos;

            RecognizeGesture(gestureVector);
            isGestureActive = false;
            ShowGestureIndicator(false);
        }

        if (isGestureActive)
        {
            UpdateGestureIndicator(Input.mousePosition);
        }
    }

    void RecognizeGesture(Vector3 gestureVector)
    {
        float angle = Mathf.Atan2(gestureVector.y, gestureVector.x) * Mathf.Rad2Deg;

        if (gestureVector.magnitude > 50f) // Minimum gesture distance
        {
            if (Mathf.Abs(angle) < 45f && gestureVector.x > 0)
            {
                SendRobotCommand("MOVE_RIGHT");
            }
            else if (Mathf.Abs(angle) > 135f)
            {
                SendRobotCommand("MOVE_LEFT");
            }
            else if (angle > 45f && angle < 135f)
            {
                SendRobotCommand("MOVE_FORWARD");
            }
            else if (angle < -45f && angle > -135f)
            {
                SendRobotCommand("MOVE_BACKWARD");
            }
        }
    }

    void SendRobotCommand(string command)
    {
        Debug.Log($"Recognized gesture: {command}");
        // Send command to robot via ROS
    }

    void ShowGestureIndicator(bool show)
    {
        gestureIndicator.SetActive(show);
    }

    void UpdateGestureIndicator(Vector3 screenPos)
    {
        Vector3 worldPos = mainCamera.ScreenToWorldPoint(new Vector3(screenPos.x, screenPos.y, 10f));
        gestureIndicator.transform.position = worldPos;
    }

    void HandleTouchGestures()
    {
        // Similar implementation for touch devices
        #if UNITY_ANDROID || UNITY_IOS
        if (Input.touchCount > 0)
        {
            Touch touch = Input.GetTouch(0);

            if (touch.phase == TouchPhase.Began)
            {
                gestureStartPos = touch.position;
                isGestureActive = true;
                ShowGestureIndicator(true);
            }
            else if (touch.phase == TouchPhase.Ended && isGestureActive)
            {
                RecognizeGesture(touch.position - gestureStartPos);
                isGestureActive = false;
                ShowGestureIndicator(false);
            }
        }
        #endif
    }
}
```

## Connecting Unity to ROS 2

### ROS-TCP-Connector Integration
Set up communication between Unity and ROS 2:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;

public class UnityRosBridge : MonoBehaviour
{
    ROSConnection ros;

    [Header("Topics")]
    public string cmdVelTopic = "/cmd_vel";
    public string laserScanTopic = "/scan";
    public string imuTopic = "/imu/data";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>(cmdVelTopic);
        ros.RegisterSubscriber<LaserScanMsg>(laserScanTopic, OnLaserScanReceived);
        ros.RegisterSubscriber<ImuMsg>(imuTopic, OnImuReceived);
    }

    public void SendVelocityCommand(float linearX, float angularZ)
    {
        var twist = new TwistMsg();
        twist.linear.Set(linearX, 0, 0);
        twist.angular.Set(0, 0, angularZ);

        ros.Publish(cmdVelTopic, twist);
    }

    void OnLaserScanReceived(LaserScanMsg msg)
    {
        Debug.Log($"Received laser scan with {msg.ranges.Length} points");
        // Process laser scan data
        ProcessLaserScan(msg);
    }

    void OnImuReceived(ImuMsg msg)
    {
        Debug.Log($"Received IMU data: {msg.orientation}");
        // Process IMU data
        ProcessImuData(msg);
    }

    void ProcessLaserScan(LaserScanMsg scan)
    {
        // Example: Visualize laser scan in Unity
        foreach (Transform child in transform)
        {
            Destroy(child.gameObject);
        }

        for (int i = 0; i < scan.ranges.Length; i += 10) // Sample every 10th point
        {
            float distance = scan.ranges[i];
            if (distance < scan.range_max && distance > scan.range_min)
            {
                float angle = scan.angle_min + (i * scan.angle_increment);

                Vector3 point = new Vector3(
                    distance * Mathf.Cos(angle),
                    0,
                    distance * Mathf.Sin(angle)
                );

                GameObject obstacle = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                obstacle.transform.position = point;
                obstacle.transform.localScale = Vector3.one * 0.1f;
                obstacle.GetComponent<Renderer>().material.color = Color.red;
            }
        }
    }

    void ProcessImuData(ImuMsg imu)
    {
        // Example: Update robot orientation based on IMU
        transform.rotation = new Quaternion(
            (float)imu.orientation.x,
            (float)imu.orientation.y,
            (float)imu.orientation.z,
            (float)imu.orientation.w
        );
    }
}
```

### Custom ROS Message Handling
Create handlers for custom robot messages:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

public class CustomMessageHandler : MonoBehaviour
{
    public class RobotStatusMsg : Message
    {
        public const string k_RosMessageName = "custom_msgs/RobotStatus";
        public override string RosMessageName => k_RosMessageName;

        public uint32 battery_level;
        public bool is_charging;
        public string current_task;
        public float[] joint_angles;
        public bool[] joint_limits_reached;

        public RobotStatusMsg()
        {
            this.battery_level = 0;
            this.is_charging = false;
            this.current_task = "";
            this.joint_angles = new float[0];
            this.joint_limits_reached = new bool[0];
        }

        public RobotStatusMsg(uint32 battery_level, bool is_charging, string current_task, float[] joint_angles, bool[] joint_limits_reached)
        {
            this.battery_level = battery_level;
            this.is_charging = is_charging;
            this.current_task = current_task;
            this.joint_angles = joint_angles;
            this.joint_limits_reached = joint_limits_reached;
        }
    }

    void OnCustomRobotStatus(RobotStatusMsg msg)
    {
        Debug.Log($"Robot Status: Battery={msg.battery_level}%, Charging={msg.is_charging}, Task={msg.current_task}");

        // Update UI elements based on robot status
        UpdateRobotUI(msg);
    }

    void UpdateRobotUI(RobotStatusMsg status)
    {
        // Update battery indicator
        // Update charging status
        // Update task display
        // Update joint angle indicators
    }
}
```

## Unity Asset Optimization for Robotics

### Level of Detail (LOD) Systems
Optimize performance with LOD groups:

```csharp
using UnityEngine;

public class RobotLODManager : MonoBehaviour
{
    [System.Serializable]
    public class LODLevel
    {
        public float screenRelativeTransitionHeight;
        public Renderer[] renderers;
    }

    public LODLevel[] lodLevels;
    private LODGroup lodGroup;

    void Start()
    {
        SetupLOD();
    }

    void SetupLOD()
    {
        lodGroup = gameObject.AddComponent<LODGroup>();

        LOD[] lods = new LOD[lodLevels.Length];

        for (int i = 0; i < lodLevels.Length; i++)
        {
            lods[i] = new LOD(lodLevels[i].screenRelativeTransitionHeight, lodLevels[i].renderers);
        }

        lodGroup.SetLODs(lods);
        lodGroup.RecalculateBounds();
    }
}
```

### Occlusion Culling
Improve performance by hiding objects not visible to cameras:

```csharp
using UnityEngine;

public class OcclusionManager : MonoBehaviour
{
    public Camera mainCamera;
    public GameObject[] cullableObjects;

    void Update()
    {
        FrustumPlanes frustum = new FrustumPlanes(mainCamera);

        foreach (GameObject obj in cullableObjects)
        {
            Renderer renderer = obj.GetComponent<Renderer>();
            if (renderer != null)
            {
                renderer.enabled = frustum.IsVisible(renderer.bounds);
            }
        }
    }

    struct FrustumPlanes
    {
        Plane[] planes;

        public FrustumPlanes(Camera cam)
        {
            planes = GeometryUtility.CalculateFrustumPlanes(cam);
        }

        public bool IsVisible(Bounds bounds)
        {
            return GeometryUtility.TestPlanesAABB(planes, bounds);
        }
    }
}
```

## Advanced Rendering Techniques

### Real-time Ray Tracing
For maximum visual fidelity:

```csharp
using UnityEngine;
using UnityEngine.Rendering;

public class RayTracingManager : MonoBehaviour
{
    public bool useRayTracing = false;
    private ReflectionProbe reflectionProbe;

    void Start()
    {
        SetupRayTracing();
    }

    void SetupRayTracing()
    {
        // Check if ray tracing is supported
        if (GraphicsSettings.renderPipelineAsset != null)
        {
            // Configure for HDRP if using ray tracing
            Debug.Log("Ray tracing setup complete");
        }
    }

    void Update()
    {
        if (useRayTracing)
        {
            EnableRayTracingEffects();
        }
        else
        {
            DisableRayTracingEffects();
        }
    }

    void EnableRayTracingEffects()
    {
        // Enable ray traced reflections, shadows, etc.
        RenderSettings.rayTracingMode = RayTracingMode.RayTraced;
    }

    void DisableRayTracingEffects()
    {
        RenderSettings.rayTracingMode = RayTracingMode.None;
    }
}
```

### Dynamic Weather System
Create realistic environmental conditions:

```csharp
using UnityEngine;

public class DynamicWeatherSystem : MonoBehaviour
{
    [Header("Weather Conditions")]
    public bool isRaining = false;
    public bool isSnowing = false;
    public bool isFoggy = false;

    [Header("Rain Effects")]
    public ParticleSystem rainEffect;
    public AudioSource rainAudio;
    public Light rainShadows;

    [Header("Wind Effects")]
    public float windForce = 0f;
    public float windVariation = 0.1f;

    void Update()
    {
        UpdateWeatherEffects();
        ApplyEnvironmentalForces();
    }

    void UpdateWeatherEffects()
    {
        if (isRaining)
        {
            if (!rainEffect.isPlaying) rainEffect.Play();
            if (!rainAudio.isPlaying) rainAudio.Play();
            rainShadows.enabled = true;
        }
        else
        {
            if (rainEffect.isPlaying) rainEffect.Stop();
            if (rainAudio.isPlaying) rainAudio.Pause();
            rainShadows.enabled = false;
        }
    }

    void ApplyEnvironmentalForces()
    {
        // Apply wind forces to objects
        float currentWind = windForce + Random.Range(-windVariation, windVariation);

        // Example: Apply to trees, cloth, etc.
        foreach (Rigidbody rb in FindObjectsOfType<Rigidbody>())
        {
            if (rb.CompareTag("WindAffected"))
            {
                rb.AddForce(Vector3.right * currentWind, ForceMode.Force);
            }
        }
    }

    public void ChangeWeather(string weatherType)
    {
        isRaining = weatherType == "rain";
        isSnowing = weatherType == "snow";
        isFoggy = weatherType == "fog";
    }
}
```

## Performance Optimization for Real-time Simulation

### Multi-threading for Physics
Offload physics calculations:

```csharp
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;

public class MultiThreadedPhysics : MonoBehaviour
{
    private List<Rigidbody> physicsObjects = new List<Rigidbody>();
    private bool shouldUpdate = true;

    void Start()
    {
        // Find all physics objects
        Rigidbody[] rbs = FindObjectsOfType<Rigidbody>();
        physicsObjects.AddRange(rbs);

        // Start physics simulation thread
        StartCoroutine(PhysicsSimulationLoop());
    }

    System.Collections.IEnumerator PhysicsSimulationLoop()
    {
        while (shouldUpdate)
        {
            yield return new WaitForFixedUpdate();

            // Process physics in parallel
            ProcessPhysicsInParallel();
        }
    }

    void ProcessPhysicsInParallel()
    {
        // Use parallel processing for physics calculations
        Parallel.ForEach(physicsObjects, (rb) =>
        {
            if (rb != null && rb.gameObject.activeInHierarchy)
            {
                // Apply custom physics calculations
                ApplyCustomPhysics(rb);
            }
        });
    }

    void ApplyCustomPhysics(Rigidbody rb)
    {
        // Custom physics logic here
        // This runs in parallel for better performance
    }

    void OnDestroy()
    {
        shouldUpdate = false;
    }
}
```

### Adaptive Quality Settings
Adjust quality based on performance:

```csharp
using UnityEngine;

public class AdaptiveQualityManager : MonoBehaviour
{
    [Header("Performance Settings")]
    public float targetFrameRate = 60f;
    public float frameRateThreshold = 5f;

    [Header("Quality Levels")]
    public int[] shadowResolutionLevels = { 512, 1024, 2048 };
    public int[] textureQualityLevels = { 0, 1, 2 }; // Low, Medium, High

    private int currentQualityLevel = 2; // Start at highest
    private float lastFrameRate;

    void Update()
    {
        float currentFrameRate = 1f / Time.unscaledDeltaTime;
        lastFrameRate = currentFrameRate;

        AdjustQualityBasedOnPerformance(currentFrameRate);
    }

    void AdjustQualityBasedOnPerformance(float frameRate)
    {
        if (frameRate < (targetFrameRate - frameRateThreshold))
        {
            // Performance is low, decrease quality
            if (currentQualityLevel > 0)
            {
                currentQualityLevel--;
                ApplyQualitySettings(currentQualityLevel);
            }
        }
        else if (frameRate > (targetFrameRate + frameRateThreshold))
        {
            // Performance is high, increase quality
            if (currentQualityLevel < 2)
            {
                currentQualityLevel++;
                ApplyQualitySettings(currentQualityLevel);
            }
        }
    }

    void ApplyQualitySettings(int level)
    {
        QualitySettings.shadowResolution = (ShadowResolution)shadowResolutionLevels[level];
        QualitySettings.masterTextureLimit = textureQualityLevels[level];

        // Apply other quality settings based on level
        switch (level)
        {
            case 0: // Low
                QualitySettings.shadows = ShadowQuality.Disable;
                QualitySettings.pixelLightCount = 1;
                break;
            case 1: // Medium
                QualitySettings.shadows = ShadowQuality.HardOnly;
                QualitySettings.pixelLightCount = 2;
                break;
            case 2: // High
                QualitySettings.shadows = ShadowQuality.All;
                QualitySettings.pixelLightCount = 4;
                break;
        }
    }
}
```

## Best Practices for Unity Robotics

### 1. Scene Organization
Keep your scenes well-organized:

```
Scene Hierarchy:
├── Environment/
│   ├── Terrain/
│   ├── Buildings/
│   ├── Vegetation/
│   └── Props/
├── Robots/
│   ├── HumanoidRobot/
│   │   ├── Body/
│   │   ├── Sensors/
│   │   └── Controllers/
├── Cameras/
├── Lights/
├── UI/
└── Managers/
    ├── ROSManager/
    ├── PhysicsManager/
    └── AudioManager/
```

### 2. Script Architecture
Follow clean architecture principles:

```csharp
// Separate concerns: Model, View, Controller
public class RobotModel
{
    public float batteryLevel;
    public Vector3 position;
    public Quaternion rotation;
    public Dictionary<string, float> jointAngles;
}

public class RobotView : MonoBehaviour
{
    public void UpdatePosition(Vector3 position) { /* Update visual position */ }
    public void UpdateRotation(Quaternion rotation) { /* Update visual rotation */ }
    public void UpdateBatteryDisplay(float level) { /* Update UI */ }
}

public class RobotController : MonoBehaviour
{
    private RobotModel model;
    private RobotView view;

    void UpdateRobot()
    {
        // Handle ROS communication
        // Update model
        // Update view
    }
}
```

### 3. Error Handling
Implement robust error handling:

```csharp
public class SafeRosConnection : MonoBehaviour
{
    private ROSConnection ros;
    private bool isConnected = false;

    void Start()
    {
        TryInitializeConnection();
    }

    void TryInitializeConnection()
    {
        try
        {
            ros = ROSConnection.GetOrCreateInstance();
            ros.Connect("127.0.0.1", 10000); // Default ROS TCP port
            isConnected = true;
            Debug.Log("Connected to ROS successfully");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Failed to connect to ROS: {e.Message}");
            // Implement fallback or retry mechanism
            Invoke("TryInitializeConnection", 5f); // Retry after 5 seconds
        }
    }

    void OnApplicationQuit()
    {
        if (ros != null)
        {
            ros.Disconnect();
        }
    }
}
```

## Troubleshooting Common Issues

### Connection Problems
If Unity can't connect to ROS:

1. Check if ROS master is running
2. Verify IP address and port in Unity
3. Check firewall settings
4. Ensure ROS-TCP-Connector is properly configured

### Performance Issues
For slow simulation:

1. Reduce polygon count of models
2. Use occlusion culling
3. Implement LOD systems
4. Optimize shader complexity
5. Reduce particle system complexity

### Rendering Artifacts
For visual glitches:

1. Check z-buffer settings
2. Verify proper lighting setup
3. Ensure correct material configurations
4. Check for overlapping geometry

## Practice Exercise

Create a Unity scene with:

1. A humanoid robot model in a high-fidelity environment
2. Realistic lighting and atmospheric effects
3. Interactive UI for robot control
4. ROS connection for sending and receiving commands
5. Performance optimizations (LOD, occlusion culling)
6. Environmental effects (weather, day/night cycle)

Connect this Unity environment to your ROS 2 system and test communication.

## Summary

Unity provides exceptional capabilities for creating high-fidelity environments for robot simulation. With its advanced rendering capabilities, intuitive HRI interfaces, and strong ROS integration, Unity serves as an excellent complement to physics-focused simulators like Gazebo. By combining Unity's visual realism with ROS 2's robotics capabilities, you can create comprehensive digital twin environments that enhance the development and testing of humanoid robots.