using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Threading.Tasks;

public class RobotController : MonoBehaviour
{
    // naming constraints do not change
    [SerializeField] private WheelCollider FLC;
    [SerializeField] private WheelCollider FRC;
    [SerializeField] private WheelCollider RLC;
    [SerializeField] private WheelCollider RRC;

    [SerializeField] private Transform FLT;
    [SerializeField] private Transform FRT;
    [SerializeField] private Transform RLT;
    [SerializeField] private Transform RRT;

    [SerializeField] private Transform FRS;
    [SerializeField] private Transform L1S;
    [SerializeField] private Transform L2S;
    [SerializeField] private Transform L3S;
    [SerializeField] private Transform R1S;
    [SerializeField] private Transform R2S;
    [SerializeField] private Transform R3S;
    [SerializeField] private Transform ORS;


    [Header("Movement Parameters")]
    public float motorTorque = 20f;
    public float maxSpeed = 50f;
    public float turnSpeed = 30f;

    [Header("Sensor Parameters")]
    public float sensorRange = 10f;
    public float obstacleDetectionDistance = 3f;
    public string roadMaterial = "MT_Road_01";

    [Header("Performance Tuning")]
    public float steeringSmoothing = 5f;
    public float accelerationSmoothing = 2f;

    private float currentSteerAngle = 0f;
    private float currentMotorTorque = 0f;

    private void Start()
    {
        SetSensorOrientations();
    }

    private void FixedUpdate()
    {
        var sensorReadings = GetSensorData();
        HandleNavigation(sensorReadings);
        UpdateWheelTransforms();
    }

    private void SetSensorOrientations()
    {
        FRS.localRotation = Quaternion.Euler(8, 0, 0);
        L1S.localRotation = Quaternion.Euler(8, -20, 0);
        L2S.localRotation = Quaternion.Euler(8, -40, 0);
        L3S.localRotation = Quaternion.Euler(10, -90, 0);
        R1S.localRotation = Quaternion.Euler(8, 20, 0);
        R2S.localRotation = Quaternion.Euler(8, 40, 0);
        R3S.localRotation = Quaternion.Euler(10, 90, 0);
    }

    private Dictionary<string, (float, string)> GetSensorData()
    {
        return new Dictionary<string, (float, string)>
        {
            { "Front", CheckSensor(FRS) },
            { "Left1", CheckSensor(L1S) },
            { "Left2", CheckSensor(L2S) },
            { "Left3", CheckSensor(L3S) },
            { "Right1", CheckSensor(R1S) },
            { "Right2", CheckSensor(R2S) },
            { "Right3", CheckSensor(R3S) }
        };
    }

    private void HandleNavigation(Dictionary<string, (float, string)> sensorReadings)
    {
        float moveSpeed = motorTorque; // Default speed
        float steerAngle = 0f;

        // Check distances to road edges on both sides
        var leftEdge = sensorReadings["Left3"].Item2.StartsWith("ED") || sensorReadings["Left3"].Item2.StartsWith("Plane") ? sensorReadings["Left3"].Item1 : sensorRange;
        var rightEdge = sensorReadings["Right3"].Item2.StartsWith("ED") || sensorReadings["Right3"].Item2.StartsWith("Plane") ? sensorReadings["Right3"].Item1 : sensorRange;
        float deviation = leftEdge - rightEdge;

        Debug.Log($"Deviation: {deviation}, LeftEdge: {leftEdge}, LeftEdgeItem: {sensorReadings["Left3"].Item2}, RightEdge: {rightEdge}, RightEdgeItem: {sensorReadings["Right3"].Item2},");
        Debug.Log($"Left1Item: {sensorReadings["Left1"].Item2}, Left2Item: {sensorReadings["Left2"].Item2}");
        Debug.Log($"Right1Item: {sensorReadings["Right1"].Item2}, Right2Item: {sensorReadings["Right2"].Item2},");

        // Adjust steering angle based on deviation
        if (Mathf.Abs(deviation) > 0.5f) // Adjust threshold as needed for sensitivity
            steerAngle = -deviation * turnSpeed / sensorRange;
        Debug.Log($"steerAngle from LS3 and RS3: {steerAngle}");

        // Check for turning point using all front sensors except Left3 and Right3
        bool isTurningPointDetected =
            sensorReadings["Front"].Item2.StartsWith("MT_Turn") ||
            sensorReadings["Left1"].Item2.StartsWith("MT_Turn") ||
            sensorReadings["Left2"].Item2.StartsWith("MT_Turn") ||
            sensorReadings["Right1"].Item2.StartsWith("MT_Turn") ||
            sensorReadings["Right2"].Item2.StartsWith("MT_Turn");

        if (isTurningPointDetected)
        {
            Debug.Log("Turning point detected. Hard braking applied.");

            // Hard brake by reducing speed aggressively
            moveSpeed = Mathf.Min(moveSpeed - (motorTorque / 5f), motorTorque / 6f); // Gradual but sharp reduction
            Debug.Log($"Slowing speed calculated: {moveSpeed}");
        }

        // Check for obstacles in front and react accordingly
        if (checkObstacle(sensorReadings, "Front") || checkObstacle(sensorReadings, "Left1") || checkObstacle(sensorReadings, "Left2") || checkObstacle(sensorReadings, "Right1") || checkObstacle(sensorReadings, "Right2"))
        {
            bool leftClear = sensorReadings["Left2"].Item1 > obstacleDetectionDistance ? sensorReadings["Left1"].Item1 > obstacleDetectionDistance : false;
            bool rightClear = sensorReadings["Right2"].Item1 > obstacleDetectionDistance ? sensorReadings["Right1"].Item1 > obstacleDetectionDistance : false;

            if (rightClear)
            {
                // steerAngle = isTurningPointDetected ? turnSpeed * 10 : turnSpeed; // Turn right
                steerAngle = turnSpeed;
                Debug.Log($"steerAngle when rightClear: {steerAngle}");
            }
            else if (leftClear)
            {
                // steerAngle = isTurningPointDetected ? -turnSpeed * 10 : -turnSpeed; // Turn left
                steerAngle = -turnSpeed;
                Debug.Log($"steerAngle when leftClear: {steerAngle}");
            }
            else
                moveSpeed = Mathf.Max(moveSpeed - (motorTorque / 8f), motorTorque / 8f); // Slow down further
        }

        Debug.Log($"Target Speed after braking: {moveSpeed}");

        ApplySteering(steerAngle);
        ApplyMotorTorque(moveSpeed);
    }

    private bool checkObstacle(Dictionary<string, (float, string)> sensorReadings, string sensor)
    {
        return sensorReadings[sensor].Item1 < obstacleDetectionDistance && !sensorReadings[sensor].Item2.StartsWith("CP") && !sensorReadings[sensor].Item2.StartsWith("MT_Road_01") && !sensorReadings[sensor].Item2.StartsWith("MT_Turn");
    }


    private (float, string) CheckSensor(Transform sensor)
    {
        RaycastHit hit;
        if (Physics.Raycast(sensor.position, sensor.forward, out hit, sensorRange))
        {
            Debug.DrawLine(sensor.position, hit.point, Color.red);
            return (hit.distance, hit.collider.gameObject.name);
        }
        Debug.DrawLine(sensor.position, sensor.position + sensor.forward * sensorRange, Color.green);
        return (sensorRange, "None");
    }

    private void ApplySteering(float targetAngle)
    {
        currentSteerAngle = Mathf.Lerp(currentSteerAngle, targetAngle, Time.deltaTime * steeringSmoothing);
        Debug.Log($"currentSteerAngle: {currentSteerAngle}");
        FLC.steerAngle = currentSteerAngle;
        FRC.steerAngle = currentSteerAngle;
    }

    private void ApplyMotorTorque(float targetTorque)
    {
        currentMotorTorque = Mathf.Lerp(currentMotorTorque, targetTorque, Time.deltaTime * accelerationSmoothing);
        Debug.Log($"currentMotorTorque: {currentMotorTorque}");
        FLC.motorTorque = currentMotorTorque;
        FRC.motorTorque = currentMotorTorque;
        RLC.motorTorque = currentMotorTorque;
        RRC.motorTorque = currentMotorTorque;
    }

    private void UpdateWheelTransforms()
    {
        UpdateWheelTransform(FLC, FLT);
        UpdateWheelTransform(FRC, FRT);
        UpdateWheelTransform(RLC, RLT);
        UpdateWheelTransform(RRC, RRT);
    }

    private void UpdateWheelTransform(WheelCollider collider, Transform wheelTransform)
    {
        Vector3 position;
        Quaternion rotation;
        collider.GetWorldPose(out position, out rotation);
        wheelTransform.position = position;
        wheelTransform.rotation = rotation;
    }
}

