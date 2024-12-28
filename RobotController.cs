using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Threading.Tasks;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class RobotController : Agent
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

    private float episodeTime = 0f; // Track the elapsed time since the episode started
    private const float maxEpisodeTime = 2500f; // Max episode time in seconds (120 seconds)

    public override void OnEpisodeBegin()
    {
        // Reset the episode timer at the beginning of each episode
        episodeTime = 0f;

        // Stop the robot's velocity and angular velocity to prevent movement at the start
        Rigidbody rb = GetComponent<Rigidbody>();
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        // Stop all wheels from rotating by setting motorTorque to zero
        FLC.motorTorque = 0f;
        FRC.motorTorque = 0f;
        RLC.motorTorque = 0f;
        RRC.motorTorque = 0f;

        // Reset wheel steer angles to prevent unintended turning
        FLC.steerAngle = 0f;
        FRC.steerAngle = 0f;

        // Reset robot position and rotation as you have it in your current code
        transform.localPosition = new Vector3(195.6539f, 0.6679955f, 192.1293f);
        transform.rotation = Quaternion.Euler(0f, 180f, 0f);

        // Set sensor orientations as defined
        SetSensorOrientations();

        // Debug.Log("Episode has started");
    }

    public override void Heuristic(in ActionBuffers actionOut)
    {
        ActionSegment<float> continuousActions = actionOut.ContinuousActions;
        continuousActions[0] = Input.GetAxisRaw("Vertical") * .5f;
        continuousActions[1] = Input.GetAxisRaw("Horizontal") * 15;
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // Update the elapsed time
        episodeTime += Time.deltaTime;

        // Debug.Log($"Episode time : {episodeTime}");

        // Check if the episode has exceeded the maximum time (120 seconds)
        if (episodeTime > maxEpisodeTime)
        {
            // End the episode after 120 seconds
            AddReward(-0.5f); // Optional: add some reward/penalty for time limit reached
            // Debug.Log("Max time passed ending");
            Debug.Log("EndEpisode: Timeout");
            EndEpisode();
        }

        float motorTorque = actions.ContinuousActions[0] * 200f;
        float steeringAngle = actions.ContinuousActions[1] * 30f;

        // Debug.Log($"motorTorque: {motorTorque}, steeringAngle: {steeringAngle}");
        ApplySteering(steeringAngle);
        ApplyMotorTorque(motorTorque);

        UpdateWheelTransforms();

        // Reward or penalty based on track conditions
        HandleTrackRewards(motorTorque, steeringAngle);
    }

    private void HandleTrackRewards(float motorTorque, float steeringAngle)
    {
        // Reward for moving forward on the road
        float speed = Vector3.Dot(transform.forward, GetComponent<Rigidbody>().linearVelocity);
        Debug.Log($"Speed: {speed}");
        if (speed > 0f)
        {
            if (speed > 0.5f)
                AddReward(0.1f); // Reward for moving forward
            else
                AddReward(-0.1f);
        }
        else
        {
            AddReward(-1f); // Penalty for moving backward
            Debug.Log("EndEpisode: Reversed");
            EndEpisode();
        }

        // Check for obstacles and avoid collisions
        var sensorReadings = GetSensorData();
        foreach (var sensorReading in sensorReadings)
        {
            string sensorName = sensorReading.Key;
            float distance = sensorReading.Value.Item1;
            string hitObject = sensorReading.Value.Item2;

            if (hitObject.Contains("Cube"))
            {
                if (distance < 1f)
                {
                    AddReward(-0.2f); // Penalty for hitting an obstacle
                    Debug.Log("EndEpisode: obstacle hit");
                    EndEpisode();
                }
                else if (sensorName == "Left3" || sensorName == "Right3")
                {
                    AddReward(0.025f * distance); // Bonus for avoiding obstacles
                }
            }

        }

        bool flat = false;

        bool isTurningPointDetected =
            sensorReadings["Front"].Item2.StartsWith("MT_Turn") ||
            sensorReadings["Left1"].Item2.StartsWith("MT_Turn") ||
            sensorReadings["Left2"].Item2.StartsWith("MT_Turn") ||
            sensorReadings["Right1"].Item2.StartsWith("MT_Turn") ||
            sensorReadings["Right2"].Item2.StartsWith("MT_Turn");

        Debug.Log($"Motor torque : {motorTorque}, steering angle: {steeringAngle}");
        if (sensorReadings["ORS"].Item1 < 2f && sensorReadings["ORS"].Item1 > -2f && !isTurningPointDetected)
        {
            if(speed <= 3f)
                AddReward(0.1f);
            else
                AddReward(-0.1f);
                
            if (motorTorque <= 12f && motorTorque >= 3f)
            {
                AddReward(0.8f);
                Debug.Log("Reward 0.8 added");
            }
            else
            {
                AddReward(-0.1f);
            }

            bool cpInfront = sensorReadings["Front"].Item2.StartsWith("CP") ||
                        sensorReadings["Right2"].Item2.StartsWith("CP") ||
                        sensorReadings["Left1"].Item2.StartsWith("CP");
            
            if(cpInfront && Mathf.Abs(steeringAngle) > 10f)
                AddReward(-1f);

            flat = true;
        }





        if (isTurningPointDetected)
        {
            if (motorTorque <= 3f)
                AddReward(1f);



            bool leftEdge = (sensorReadings["Left3"].Item2.StartsWith("ED") || sensorReadings["Left3"].Item2.StartsWith("Plane")) &&
                        (sensorReadings["Left2"].Item2.StartsWith("ED") || sensorReadings["Left2"].Item2.StartsWith("Plane")) &&
                        (sensorReadings["Left1"].Item2.StartsWith("ED") || sensorReadings["Left1"].Item2.StartsWith("Plane")) &&
                        (sensorReadings["Right3"].Item2.StartsWith("MT_Turn") && sensorReadings["Right2"].Item2.StartsWith("MT_Turn") && sensorReadings["Right1"].Item2.StartsWith("MT_Turn"));

            bool rightEdge = (sensorReadings["Right3"].Item2.StartsWith("ED") || sensorReadings["Right3"].Item2.StartsWith("Plane")) &&
                        (sensorReadings["Right2"].Item2.StartsWith("ED") || sensorReadings["Right2"].Item2.StartsWith("Plane")) &&
                        (sensorReadings["Right1"].Item2.StartsWith("ED") || sensorReadings["Right1"].Item2.StartsWith("Plane")) &&
                        (sensorReadings["Left1"].Item2.StartsWith("MT_Turn") && sensorReadings["Left2"].Item2.StartsWith("MT_Turn") && sensorReadings["Left3"].Item2.StartsWith("MT_Turn"));

            if (leftEdge && !rightEdge)
            {
                if (Mathf.Abs(steeringAngle) > 5f)
                    AddReward(0.1f * steeringAngle);
                else
                {
                    AddReward(-1f);
                    Debug.Log("Leftedge detected angle not correct");
                }

            }
            else if (rightEdge && !leftEdge)
            {
                if (Mathf.Abs(steeringAngle) < -5f)
                    AddReward(0.1f * steeringAngle);
                else
                {
                    AddReward(-1f);
                    Debug.Log("Rightedge detected angle not correct");
                }
            }

        }
        else
        {
            if (currentSteerAngle == -steeringAngle)
                AddReward(-0.5f);
        }

        // Debug.Log($"Left3 distance : {sensorReadings["Left3"].Item1}, object : {sensorReadings["Left3"].Item2}");
        if (sensorReadings["Left3"].Item2.StartsWith("ED") && (sensorReadings["Left3"].Item1 < 2.0f))
        {
            // Debug.Log($"Left3 distance inside : {sensorReadings["Left3"].Item1}");
            AddReward(-0.2f);
        }

        // Debug.Log($"Right3 distance : {sensorReadings["Right3"].Item1}, object : {sensorReadings["Right3"].Item2}");
        if (sensorReadings["Right3"].Item2.StartsWith("ED") && (sensorReadings["Right3"].Item1 < 2.0f))
        {
            // Debug.Log($"Right3 distance inside : {sensorReadings["Right3"].Item1}");
            AddReward(-0.2f);
        }

        if (sensorReadings["Left3"].Item2.StartsWith("ED") && sensorReadings["Right3"].Item2.StartsWith("ED") && !isTurningPointDetected)
        {
            AddReward(0.05f);
            float deviation = sensorReadings["Left3"].Item1 - sensorReadings["Right3"].Item1;
            if (Mathf.Abs(deviation) < 0.5f)
                AddReward(0.1f);
        }


        // Check if the robot passed a checkpoint
        for (int i = 1; i <= 22; i++)
        {
            string checkpointName = "CP" + i;
            if (CheckForCheckpointPassed(checkpointName))
            {
                AddReward(0.2f * i); // Reward for passing a checkpoint
                break;
            }
        }

        // Final reward for stopping after passing CP22 without falling off the track
        if (HasPassedFinalCheckpoint() && IsStopped())
        {
            AddReward(1f); // Final bonus for stopping after passing CP22

            Debug.Log("EndEpisode: successfully completed");
            EndEpisode();

        }

        // Check if the robot has fallen off the track
        if (IsOutOfTrack())
        {
            AddReward(-1f); // Penalty for going off track
            if (flat)
                AddReward(-0.5f);

            if (isTurningPointDetected)
                AddReward(-1f);

            Debug.Log("EndEpisode: out of track");
            EndEpisode(); // End the episode if out of the track
            // Debug.Log("Called end episode");
        }
    }

    private bool IsOutOfTrack()
    {
        // Check if the robot has moved outside the allowed track area
        // You can define track boundaries based on the road material or specific coordinates
        // Here, assuming the robot has gone out of bounds of the road material

        Collider[] hitColliders = Physics.OverlapSphere(transform.position, 2f); // Adjust radius as necessary
        foreach (var collider in hitColliders)
        {
            // Debug.Log($"collider object: {collider.gameObject.name}");


            if (collider.gameObject.name.StartsWith("ED"))
                return true;

            if ((collider.gameObject.name.StartsWith("MT_Road") || collider.gameObject.name.StartsWith("MT_Turn")))
            {
                // Debug.Log($"collider object: {collider.gameObject.name}");
                return false; // The robot is on the road, not out of track
            };
        }

        // If no road material is found, the robot is out of the track
        return true;
    }

    private bool CheckForCheckpointPassed(string checkpointName)
    {
        // Check if the robot has passed a checkpoint
        Collider[] hitColliders = Physics.OverlapSphere(transform.position, 1f); // Adjust radius as needed
        foreach (var collider in hitColliders)
        {
            if (collider.gameObject.name.StartsWith(checkpointName))
            {
                return true; // The robot has passed this checkpoint
            }
        }
        return false;
    }

    private bool HasPassedFinalCheckpoint()
    {
        // Check if the robot has passed CP22
        return CheckForCheckpointPassed("CP22");
    }

    private bool IsStopped()
    {
        // Check if the robot has stopped moving (velocity is low)
        return GetComponent<Rigidbody>().linearVelocity.magnitude < 0.1f; // Adjust as necessary for "stopped" state
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        var sensorReadings = GetSensorData();
        int frontItemId = GetHitItemId(sensorReadings["Front"].Item2);
        sensor.AddObservation(frontItemId);
        sensor.AddObservation(sensorReadings["Front"].Item1);
        sensor.AddObservation(sensorReadings["Left1"].Item1);
        sensor.AddObservation(sensorReadings["Left2"].Item1);
        sensor.AddObservation(sensorReadings["Left3"].Item1);
        sensor.AddObservation(sensorReadings["Right1"].Item1);
        sensor.AddObservation(sensorReadings["Right2"].Item1);
        sensor.AddObservation(sensorReadings["Right3"].Item1);
        sensor.AddObservation(sensorReadings["ORS"].Item1);
        sensor.AddObservation(sensorReadings["ORSZ"].Item1);
    }

    private int GetHitItemId(string hitName)
    {
        if (hitName.StartsWith("MT_Road"))
            return 1;
        else if (hitName.StartsWith("MT_Turn"))
            return 2;
        else if (hitName.StartsWith("CP"))
            return 3;
        else if (hitName.StartsWith("Plane"))
            return 4;
        else if (hitName.StartsWith("ED"))
            return 5;
        else if (hitName.StartsWith("Cube"))
            return 6;
        else
            return 0;
    }

    private void SetSensorOrientations()
    {
        FRS.localRotation = Quaternion.Euler(8, 0, 0);
        L1S.localRotation = Quaternion.Euler(8, -15, 0);
        L2S.localRotation = Quaternion.Euler(8, -35, 0);
        L3S.localRotation = Quaternion.Euler(10, -90, 0);
        R1S.localRotation = Quaternion.Euler(8, 15, 0);
        R2S.localRotation = Quaternion.Euler(8, 35, 0);
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
            { "Right3", CheckSensor(R3S) },
            { "ORS", CheckOrientationSensor() },
            { "ORSZ", CheckOrientationSensorZ() }
        };
    }

    private (float, string) CheckOrientationSensorZ()
    {
        // Get the robot's forward-facing angle relative to the world
        float zaw = transform.eulerAngles.z;
        // Debug.Log($"xaw pitch: {xaw}");
        // Normalize the pitch
        float normalizedPitch = (zaw > 180) ? zaw - 360 : zaw;

        // Return the xaw value along with a descriptor
        return (normalizedPitch, "OrientationZ");
    }

    private (float, string) CheckOrientationSensor()
    {
        // Get the robot's forward-facing angle relative to the world
        float xaw = transform.eulerAngles.x;
        // Debug.Log($"xaw pitch: {xaw}");
        // Normalize the pitch
        float normalizedPitch = (xaw > 180) ? xaw - 360 : xaw;

        // Return the xaw value along with a descriptor
        return (normalizedPitch, "OrientationX");
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

