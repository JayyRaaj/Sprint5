using UnityEngine;

public class SimpleCCDInverseKinematics : MonoBehaviour
{
    public Transform[] joints;        // Array of joint transforms
    public Transform target;          // Target position for the end effector
    public float tolerance = 0.01f;   // Allowable distance error
    public float maxRotationSpeed = 5f;  // Maximum rotation speed for convergence
    public float damping = 0.9f;      // Damping factor for smooth stopping

    private void Update()
    {
        SimpleCyclicCoordinateDescent();
    }

    void SimpleCyclicCoordinateDescent()
    {
        float distanceToTarget = Vector3.Distance(joints[joints.Length - 1].position, target.position);

        // Loop until the end effector is close enough to the target
        while (distanceToTarget > tolerance)
        {
            // Iterate from the end effector to the base joint
            for (int i = joints.Length - 2; i >= 0; i--)  // Skip the end effector joint
            {
                // Calculate vectors from the current joint to the end effector and to the target
                Vector3 toEndEffector = joints[joints.Length - 1].position - joints[i].position;
                Vector3 toTarget = target.position - joints[i].position;

                // Calculate rotation to bring the end effector closer to the target
                Quaternion rotationToTarget = Quaternion.FromToRotation(toEndEffector, toTarget);

                // Smoothly interpolate rotation with damping
                Quaternion smoothRotation = Quaternion.Slerp(joints[i].rotation, rotationToTarget * joints[i].rotation, damping * Time.deltaTime);

                // Rotate the joint towards the target with a maximum speed limit
                joints[i].rotation = Quaternion.RotateTowards(joints[i].rotation, smoothRotation, maxRotationSpeed * Time.deltaTime);
            }

            // Update distance for stopping condition
            distanceToTarget = Vector3.Distance(joints[joints.Length - 1].position, target.position);
        }
    }
}
