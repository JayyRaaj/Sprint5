using UnityEngine;

public class GradientDescentIK : MonoBehaviour
{
    public Transform[] joints;              // Array of joints in the robot arm
    public Transform endEffector;           // The end effector of the robot
    public Transform target;                // The target cube to reach
    public float learningRate = 0.3f;       // Adjusted learning rate
    public float distanceThreshold = 0.05f; // Distance threshold to stop the algorithm
    public int maxIterations = 1000;        // Maximum number of iterations

    // Rotation limits for each joint (example values, adjust for your robot)
    public float[] minZRotation;
    public float[] maxZRotation;

    private void Update()
    {
        PerformIK();
    }

    private void PerformIK()
    {
        int iteration = 0;
        float currentDistance = Vector3.Distance(endEffector.position, target.position);

        while (currentDistance > distanceThreshold && iteration < maxIterations)
        {
            bool hasMoved = false;

            for (int i = 0; i < joints.Length; i++)
            {
                // Calculate gradient for the current joint
                float gradient = CalculateGradient(joints[i], endEffector, target.position);

                // Apply rotation around z-axis only with learning rate, then clamp
                float newRotationZ = joints[i].localEulerAngles.z - gradient * learningRate;
                newRotationZ = Mathf.Clamp(newRotationZ, minZRotation[i], maxZRotation[i]);
                
                // Set the new rotation while keeping x and y the same
                joints[i].localEulerAngles = new Vector3(joints[i].localEulerAngles.x, joints[i].localEulerAngles.y, newRotationZ);

                hasMoved = true;
            }

            // Recalculate distance and increase iteration count
            currentDistance = Vector3.Distance(endEffector.position, target.position);
            iteration++;

            if (!hasMoved)
            {
                Debug.LogWarning("No movement detected. Consider adjusting learning rate or constraints.");
                break;
            }
        }

        if (currentDistance <= distanceThreshold)
        {
            Debug.Log("Target reached within distance threshold.");
        }
        else
        {
            Debug.Log("Max iterations reached or movement stopped without reaching the target.");
        }
    }

    private float CalculateGradient(Transform joint, Transform endEffector, Vector3 targetPosition)
    {
        // Save the current rotation
        Quaternion originalRotation = joint.rotation;

        // Small delta for gradient calculation
        float deltaTheta = 1.0f;
        joint.Rotate(0, 0, deltaTheta);  // Rotate around z-axis only

        // Calculate the distance after the small rotation
        float distanceAfterRotation = Vector3.Distance(endEffector.position, targetPosition);

        // Restore the original rotation
        joint.rotation = originalRotation;

        // Calculate the gradient
        return (distanceAfterRotation - Vector3.Distance(endEffector.position, targetPosition)) / deltaTheta;
    }
}
