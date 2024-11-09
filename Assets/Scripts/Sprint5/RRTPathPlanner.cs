using System.Collections.Generic;
using UnityEngine;

public class CCDIKWithArticulation : MonoBehaviour
{
    public ArticulationBody[] joints;      // Array of joint articulation bodies, from base to end effector
    public Transform endEffector;          // End-effector of the robot
    public Transform target;               // Target position for the end-effector
    public float threshold = 0.01f;        // How close the end effector should be to the target
    public int maxIterations = 10;         // Maximum iterations per update
    public float rotationStep = 1.0f;      // Step size for joint rotation adjustments
    public float stepSize = 5f; // Degree change per iteration for each joint


    void Update()
    {
        if (Vector3.Distance(endEffector.position, target.position) > threshold)
            PerformCCD();
    }

    private void PerformCCD()
{
    if (Vector3.Distance(endEffector.position, target.position) < threshold)
        return; // End effector is close enough to the target

    for (int iteration = 0; iteration < maxIterations; iteration++)
    {
        for (int i = joints.Length - 1; i >= 0; i--)
        {
            ArticulationBody joint = joints[i];

            // Get direction vectors
            Vector3 toEndEffector = endEffector.position - joint.transform.position;
            Vector3 toTarget = target.position - joint.transform.position;

            // Check for obstacles in the direction of movement
            bool isObstacleDetected = Physics.Raycast(joint.transform.position, toTarget.normalized, toTarget.magnitude);
            if (isObstacleDetected)
            {
                // If there's an obstacle, slightly adjust the target angle
                toTarget = Quaternion.Euler(0, stepSize, 0) * toTarget; // Rotate slightly to avoid the obstacle
            }

            // Calculate rotation angle to bring end effector closer to target
            float angle = Vector3.SignedAngle(toEndEffector, toTarget, joint.transform.up);

            // Limit the angle to stepSize to ensure gradual movement
            angle = Mathf.Clamp(angle, -stepSize, stepSize);

            // Apply rotation to the joint
            Quaternion rotation = Quaternion.AngleAxis(angle, joint.transform.up);
            joint.transform.rotation = rotation * joint.transform.rotation;

            // Update articulation body joint angle
            var drive = joint.xDrive;
            drive.target = angle;
            joint.xDrive = drive;

            // Check if the end effector is close enough to the target
            if (Vector3.Distance(endEffector.position, target.position) < threshold)
                return;
        }
    }
}

}
