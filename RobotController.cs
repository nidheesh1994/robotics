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
    public float motorTorque = 2000f;
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
        UpdateWheelTransforms();
    }

    private void SetSensorOrientations()
    {
        FRS.localRotation = Quaternion.Euler(10, 0, 0);
        L1S.localRotation = Quaternion.Euler(10, -20, 0);
        L2S.localRotation = Quaternion.Euler(10, -40, 0);
        L3S.localRotation = Quaternion.Euler(10, -90, 0);
        R1S.localRotation = Quaternion.Euler(10, 20, 0);
        R2S.localRotation = Quaternion.Euler(10, 40, 0);
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
        FLC.steerAngle = currentSteerAngle;
        FRC.steerAngle = currentSteerAngle;
    }

    private void ApplyMotorTorque(float targetTorque)
    {
        currentMotorTorque = Mathf.Lerp(currentMotorTorque, targetTorque, Time.deltaTime * accelerationSmoothing);
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

