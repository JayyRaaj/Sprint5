using UnityEngine;

public class GradientDescentIK : MonoBehaviour
{
    public Transform[] joints;  // Array of joints in the robot arm
    public Transform endEffector;  // The end effector of the robot
    public Transform target;  // The target object to reach
    public float learningRate = 0.5f;  // Adjusted learning rate
    public float distanceThreshold = 0.05f;  // Slightly larger threshold
    public int maxIterations = 1000;  // Maximum number of iterations

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

                // Log the gradient to observe its effect
                Debug.Log($"Iteration {iteration}, Joint {i} gradient: {gradient}");

                // Apply rotation if gradient is sufficient
                if (Mathf.Abs(gradient) > 1e-3)  // Filter out tiny gradients
                {
                    joints[i].Rotate(0, 0, -gradient * learningRate, Space.Self);  // Only z-axis rotation
                    hasMoved = true;
                }
            }

            // Update the current distance and increment iteration count
            currentDistance = Vector3.Distance(endEffector.position, target.position);
            iteration++;

            // Check if any joint moved; if not, break
            if (!hasMoved)
            {
                Debug.LogWarning("No movement detected - consider further adjusting learning rate or axis constraints.");
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

        // Increase deltaTheta for clearer gradient calculation
        float deltaTheta = 1.0f;
        joint.Rotate(0, 0, deltaTheta);  // Rotate around z-axis only

        // Calculate the distance after the rotation
        float distanceAfterRotation = Vector3.Distance(endEffector.position, targetPosition);

        // Restore the original rotation
        joint.rotation = originalRotation;

        // Calculate the gradient
        return (distanceAfterRotation - Vector3.Distance(endEffector.position, targetPosition)) / deltaTheta;
    }
}
