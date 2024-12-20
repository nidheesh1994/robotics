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

    public float brakeTorque = 20f;
    public float turnSpeed = 30f;

    [Header("Sensor Parameters")]
    public float sensorRange = 10f;
    public float obstacleDetectionDistance = 7f;
    public string roadMaterial = "MT_Road_01";

    [Header("Performance Tuning")]
    public float finalBrakeForce = -140f;
    public float steeringSmoothing = 5f;
    public float accelerationSmoothing = 2f;

    public float decelerationSmoothing = 10f;

    private float currentSteerAngle = 0f;
    private float currentMotorTorque = 0f;

    private float currentBrakeTorque = 0f;

    private bool finishingPointDetected = false;

    private void Start()
    {
        SetSensorOrientations();
    }

    private void FixedUpdate()
    {
        var sensorReadings = GetSensorData();
        HandleNavigation(sensorReadings);
        UpdateWheelTransforms();
        // Debug.Log("Time");
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
            { "Right3", CheckSensor(R3S) },
            { "ORS", CheckOrientationSensor() }
        };
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

    private void HandleNavigation(Dictionary<string, (float, string)> sensorReadings)
    {
        float moveSpeed = motorTorque; // Default speed
        float steerAngle = 0f;




        // Check distances to road edges on both sides
        var leftEdge = sensorReadings["Left3"].Item2.StartsWith("ED") || sensorReadings["Left3"].Item2.StartsWith("Plane") ? sensorReadings["Left3"].Item1 : sensorRange;
        var rightEdge = sensorReadings["Right3"].Item2.StartsWith("ED") || sensorReadings["Right3"].Item2.StartsWith("Plane") ? sensorReadings["Right3"].Item1 : sensorRange;
        float deviation = leftEdge - rightEdge;

        if (sensorReadings["Left3"].Item2.StartsWith("None") && (sensorReadings["Right3"].Item2.StartsWith("MT_Road_01") || sensorReadings["Right3"].Item2.StartsWith("MT_Turn")))
        {
            deviation = sensorReadings["Right3"].Item1 - sensorRange;
            Debug.Log("Special check1");
        }

        if (sensorReadings["Right3"].Item2.StartsWith("None") && (sensorReadings["Left3"].Item2.StartsWith("MT_Road_01") || sensorReadings["Left3"].Item2.StartsWith("MT_Turn")))
        {
            deviation = sensorRange - sensorReadings["Left3"].Item1;
            Debug.Log("Special check2");
        }

        if (sensorReadings["Right3"].Item2.StartsWith("Cube") || sensorReadings["Left3"].Item2.StartsWith("Cube"))
        {
            deviation = 0;
        }

        Debug.Log($"Deviation: {deviation}, LeftEdge: {leftEdge}, LeftEdgeItem: {sensorReadings["Left3"].Item2}, RightEdge: {rightEdge}, RightEdgeItem: {sensorReadings["Right3"].Item2},");
        Debug.Log($"Left1Item: {sensorReadings["Left1"].Item2}, Left2Item: {sensorReadings["Left2"].Item2}");
        Debug.Log($"Right1Item: {sensorReadings["Right1"].Item2}, Right2Item: {sensorReadings["Right2"].Item2},");

        // Adjust steering angle based on deviation
        if (Mathf.Abs(deviation) > 0.5f && !(sensorReadings["Left3"].Item2.StartsWith("CP") && sensorReadings["Right3"].Item2.StartsWith("CP"))) // Adjust threshold as needed for sensitivity
        {
            steerAngle = -deviation * turnSpeed / sensorRange;
            // Debug.Log($"steerAngle from LS3 and RS3: {steerAngle}");
        }

        // Check for turning point using all front sensors except Left3 and Right3
        bool isTurningPointDetected =
            sensorReadings["Front"].Item2.StartsWith("MT_Turn") ||
            sensorReadings["Left1"].Item2.StartsWith("MT_Turn") ||
            sensorReadings["Left2"].Item2.StartsWith("MT_Turn") ||
            sensorReadings["Right1"].Item2.StartsWith("MT_Turn") ||
            sensorReadings["Right2"].Item2.StartsWith("MT_Turn");

        if (isTurningPointDetected)
        {
            // // Debug.Log("Turning point detected. Hard braking applied.");

            // Hard brake by reducing speed aggressively
            moveSpeed = Mathf.Min(moveSpeed - (motorTorque / 5f), motorTorque / 6f); // Gradual but sharp reduction
            // // Debug.Log($"Slowing speed calculated: {moveSpeed}");
        }

        // Log ORS orientation sensor reading
        if (sensorReadings.ContainsKey("ORS"))
        {
            var orsReading = sensorReadings["ORS"];
            float pitch = orsReading.Item1;

            // // Debug.Log($"Pitch: {pitch} ");


            // Adjust speed based on pitch
            if (pitch <= -2f) // Upward slope, mild to steep
            {
                if (!isTurningPointDetected || moveSpeed <= 3.5f)
                {

                    // // Debug.Log($"Uphill detected: Pitch = {pitch}. Increasing torque for acceleration.");
                    // // Debug.Log($"FrontItem: {sensorReadings["Front"].Item2}");
                    moveSpeed += motorTorque * 20f >= 270f ? 270f : motorTorque * 20f; // Boost acceleration
                }

            }
            else if (pitch > 2f) // Downward slope, mild to steep
            {
                if (!isTurningPointDetected)
                {
                    // // Debug.Log($"Downhill detected: Pitch = {pitch}. Applying brake.");
                    moveSpeed = (moveSpeed - (motorTorque * 2f)) <= 20 ? 20 : (moveSpeed - (motorTorque * 2f)); // Apply brake by reducing torque
                }
            }
            else
            {
                // // Debug.Log($"Flat terrain detected: Pitch = {pitch}. Maintaining default torque.");
            }
        }

        bool obstacleDetected = false;

        // Check for obstacles in front and react accordingly
        if (checkObstacle(sensorReadings, "Front") || checkObstacle(sensorReadings, "Left1") || checkObstacle(sensorReadings, "Left2") || checkObstacle(sensorReadings, "Right1") || checkObstacle(sensorReadings, "Right2"))
        {
            // Debug.Log("Obstacle detected");
            bool leftClear = sensorReadings["Left2"].Item1 > obstacleDetectionDistance ? sensorReadings["Left1"].Item1 > obstacleDetectionDistance : false;
            bool rightClear = sensorReadings["Right2"].Item1 > obstacleDetectionDistance ? sensorReadings["Right1"].Item1 > obstacleDetectionDistance : false;

            if (rightClear)
            {
                // steerAngle = isTurningPointDetected ? turnSpeed * 10 : turnSpeed; // Turn right
                if (sensorReadings["Left2"].Item2.StartsWith("Cube") && sensorReadings["Left2"].Item1 < obstacleDetectionDistance - 3 && sensorReadings["Left1"].Item1 > obstacleDetectionDistance)
                {

                    steerAngle = turnSpeed - 20;
                    Debug.Log($"LF2 detected object turnspeed: {steerAngle}");
                }
                else if (sensorReadings["Left1"].Item2.StartsWith("Cube") && sensorReadings["Left1"].Item1 < obstacleDetectionDistance - 2)
                {
                    steerAngle = turnSpeed - 10;
                    Debug.Log($"LF1 detected object turnspeed: {steerAngle}");
                }
                else if (!sensorReadings["Left1"].Item2.StartsWith("Cube") && !sensorReadings["Left2"].Item2.StartsWith("Cube"))
                {

                    steerAngle = turnSpeed - 5;
                    Debug.Log($"Other object detected. turnspeed: {steerAngle}");
                }
                obstacleDetected = true;

                // Debug.Log($"steerAngle when rightClear: {steerAngle}");
            }
            else if (leftClear)
            {
                // steerAngle = isTurningPointDetected ? -turnSpeed * 10 : -turnSpeed; // Turn left
                if (sensorReadings["Right2"].Item2.StartsWith("Cube") && sensorReadings["Right2"].Item1 < obstacleDetectionDistance - 3 && sensorReadings["Right1"].Item1 > obstacleDetectionDistance)
                {
                    steerAngle = -turnSpeed + 25;
                    Debug.Log($"RF2 detected object turnspeed: {steerAngle}");
                }
                else if (sensorReadings["Right1"].Item2.StartsWith("Cube") && sensorReadings["Right1"].Item1 < obstacleDetectionDistance - 2)
                {
                    steerAngle = -turnSpeed + 15;
                    Debug.Log($"RF1 detected object turnspeed: {steerAngle}");
                }

                else if (!sensorReadings["Right1"].Item2.StartsWith("Cube") && !sensorReadings["Right2"].Item2.StartsWith("Cube"))
                    steerAngle = -turnSpeed + 5;

                obstacleDetected = true;

                // Debug.Log($"steerAngle when leftClear: {steerAngle}");
            }
            else
                moveSpeed = Mathf.Max(moveSpeed - (motorTorque / 8f), motorTorque / 8f); // Slow down further
        }

        // Debug.Log($"Final Target Speed: {moveSpeed}");

        if (moveSpeed > 200f)
        {
            // Calculate the difference from 200
            float speedExcess = moveSpeed - 200f;

            // Scale the reduction (tweak the factor as needed)
            float reductionFactor = 0.1f; // Adjust this value for sensitivity
            if (steerAngle > 0f)
                steerAngle -= speedExcess * reductionFactor;
            else if (steerAngle < 0f)
                steerAngle += speedExcess * reductionFactor;

            // Optionally clamp the steerAngle to avoid excessive reduction
            steerAngle = Mathf.Clamp(steerAngle, -turnSpeed + 5, turnSpeed - 5); // Replace 'maxSteerAngle' with your max limit
            Debug.Log($"Adjusted steering angle: {steerAngle}");
        }

        if (sensorReadings["Left3"].Item2.StartsWith("ED") && sensorReadings["Right3"].Item2.StartsWith("ED"))
        {
            bool isFrontEmpty = sensorReadings["Left2"].Item2.StartsWith("None") &&
                sensorReadings["Left1"].Item2.StartsWith("None") &&
                sensorReadings["Right1"].Item2.StartsWith("None") &&
                sensorReadings["Front"].Item2.StartsWith("None") &&
                sensorReadings["Right2"].Item2.StartsWith("None");

            if (isFrontEmpty)
            {
                // Get the current speed of the car
                float currentSpeed = moveSpeed; // Assume moveSpeed holds the current speed in units/second

                // Calculate the required deceleration to stop in 2 seconds
                float deceleration = currentSpeed / 2f;

                // Apply braking force
                moveSpeed -= deceleration * Time.deltaTime;

                // Ensure speed doesn't go negative
                moveSpeed = -Mathf.Max(moveSpeed, 0f);

            }

        }

        // if (isTurningPointDetected)
        // {
        bool leftEdgeDetected = false;
        bool righEdgeDetected = false;
        if ((sensorReadings["Left1"].Item2.StartsWith("ED") || sensorReadings["Left2"].Item2.StartsWith("ED") || sensorReadings["Left3"].Item2.StartsWith("ED")) &&
            (!sensorReadings["Right1"].Item2.StartsWith("ED") && !sensorReadings["Right2"].Item2.StartsWith("ED") && !sensorReadings["Right3"].Item2.StartsWith("ED"))
        )
        {
            leftEdgeDetected = true;
            // Debug.Log($"Detection: leftEdge, steeringAngle: {steerAngle}");
        }
        else if (!sensorReadings["Left1"].Item2.StartsWith("None") && !sensorReadings["Left2"].Item2.StartsWith("None") && !sensorReadings["Left3"].Item2.StartsWith("None") &&
            sensorReadings["Right1"].Item2.StartsWith("None") && sensorReadings["Right2"].Item2.StartsWith("None") && sensorReadings["Right3"].Item2.StartsWith("None")
        )
        {
            leftEdgeDetected = true;
        }

        if ((sensorReadings["Right1"].Item2.StartsWith("ED") || sensorReadings["Right2"].Item2.StartsWith("ED") || sensorReadings["Right3"].Item2.StartsWith("ED")) &&
            (!sensorReadings["Left1"].Item2.StartsWith("ED") && !sensorReadings["Left2"].Item2.StartsWith("ED") && !sensorReadings["Left3"].Item2.StartsWith("ED"))
        )
        {
            righEdgeDetected = true;
            // Debug.Log($"Detection: rightEdge, steeringAngle: {steerAngle}");
        }
        else if (!sensorReadings["Right1"].Item2.StartsWith("None") && !sensorReadings["Right2"].Item2.StartsWith("None") && !sensorReadings["Right3"].Item2.StartsWith("None") &&
            sensorReadings["Left1"].Item2.StartsWith("None") && sensorReadings["Left2"].Item2.StartsWith("None") && sensorReadings["Left3"].Item2.StartsWith("None")
        )
        {
            righEdgeDetected = true;
        }

        Debug.Log($"Detection: steeringAngle: {steerAngle}");

        if ((leftEdgeDetected || righEdgeDetected) && !(leftEdgeDetected && righEdgeDetected) && !obstacleDetected)
        {
            if (leftEdgeDetected)
            {
                if (steerAngle < 5)
                    steerAngle = 5f;
                Debug.Log($"Detection: leftEdge, steeringAngle: {steerAngle}");

            }
            else
            {
                if (steerAngle > 5f)
                    steerAngle = -5f;
                Debug.Log($"Detection: rightEdge, steeringAngle: {steerAngle}");
            }
        }
        // }


        // if (!finishingPointDetected)
        // {
        //     finishingPointDetected = checkFinishingPoint(sensorReadings);
        // }
        // else
        // {
        //     // Get the rpm for each wheel
        //     float flcRpm = FLC.rpm; // Front Left Wheel RPM
        //     float frcRpm = FRC.rpm; // Front Right Wheel RPM
        //     float rlcRpm = RLC.rpm; // Rear Left Wheel RPM
        //     float rrcRpm = RRC.rpm; // Rear Right Wheel RPM

        //     // Check if the wheel is still moving (RPM > 0 means it's rotating)
        //     if (flcRpm > 0.1f || frcRpm > 0.1f || rlcRpm > 0.1f || rrcRpm > 0.1f)
        //     {
        //         //Applying hard breaking
        //         moveSpeed = finalBrakeForce;
        //         ApplyMotorTorque(finalBrakeForce);
        //     }
        //     else
        //     {
        //         moveSpeed = 0f;
        //         ApplyMotorTorque(moveSpeed);
        //     }

        // }

        // Access forward friction
        // WheelFrictionCurve forwardFriction = FLC.forwardFriction;
        // // Debug.Log($"Friction: {forwardFriction.stiffness}");


        ApplySteering(steerAngle);
        ApplyMotorTorque(moveSpeed);
    }

    private bool checkFinishingPoint(Dictionary<string, (float, string)> sensorReadings)
    {
        return sensorReadings["Left3"].Item2.StartsWith("CP22") || sensorReadings["Right3"].Item2.StartsWith("CP22");
        // sensorReadings["Left2"].Item2.StartsWith("CP1") ||
        // sensorReadings["Right1"].Item2.StartsWith("CP1") ||
        // sensorReadings["Right2"].Item2.StartsWith("CP1") ||
        // sensorReadings["Front"].Item2.StartsWith("CP1");
    }

    private bool checkObstacle(Dictionary<string, (float, string)> sensorReadings, string sensor)
    {
        return sensorReadings[sensor].Item1 < obstacleDetectionDistance && !sensorReadings[sensor].Item2.StartsWith("CP") && !sensorReadings[sensor].Item2.StartsWith("MT_Road") && !sensorReadings[sensor].Item2.StartsWith("MT_Turn");
    }


    private (float, string) CheckSensor(Transform sensor)
    {
        RaycastHit hit;
        if (Physics.Raycast(sensor.position, sensor.forward, out hit, sensorRange))
        {
            // Debug.DrawLine(sensor.position, hit.point, Color.red);
            return (hit.distance, hit.collider.gameObject.name);
        }
        // Debug.DrawLine(sensor.position, sensor.position + sensor.forward * sensorRange, Color.green);
        return (sensorRange, "None");
    }

    private void ApplySteering(float targetAngle)
    {
        currentSteerAngle = Mathf.Lerp(currentSteerAngle, targetAngle, Time.deltaTime * steeringSmoothing);
        // Debug.Log($"currentSteerAngle: {currentSteerAngle}");
        FLC.steerAngle = currentSteerAngle;
        FRC.steerAngle = currentSteerAngle;
    }

    private void ApplyMotorTorque(float targetTorque)
    {
        currentMotorTorque = Mathf.Lerp(currentMotorTorque, targetTorque, Time.deltaTime * accelerationSmoothing);
        if (finishingPointDetected && targetTorque == 0f)
            currentMotorTorque = 0;
        // Debug.Log($"currentMotorTorque: {currentMotorTorque}");
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

