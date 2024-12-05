using UnityEngine;

public class JacobianIKWithDebug : MonoBehaviour
{
    public Transform[] joints; // Array of joint transforms (6 for Niryo One)
    public Transform endEffector; // End effector transform
    public Transform target; // Target transform
    public float learningRate = 1f; // Balanced learning rate
    public int iterations = 50; // Moderate number of iterations
    public float smoothSpeed = 5f; // New parameter for smoothing
    public Vector2[] jointLimits = new Vector2[] {
        new Vector2(-175f, 175f),     // joint_1 (base) Y-axis
        new Vector2(-110f, 36.68f),   // joint_2 (arm) Z-axis
        new Vector2(-80.07f, 90f),    // joint_3 (elbow) Z-axis
        new Vector2(-175f, 175f),     // joint_4 (forearm) Y-axis
        new Vector2(-100f, 110f),     // joint_5 (wrist) Z-axis
        new Vector2(-147.5f, 147.5f)  // joint_6 (hand) Y-axis
    };

    // Define rotation axes for each joint with correct directions
    private Vector3[] jointAxes = new Vector3[] {
        Vector3.up,       // joint_1 (base) rotates around Y
        Vector3.forward,  // joint_2 (arm) rotates around Z
        Vector3.forward,  // joint_3 (elbow) rotates around Z
        Vector3.up,       // joint_4 (forearm) rotates around Y
        Vector3.forward,  // joint_5 (wrist) rotates around Z
        Vector3.up        // joint_6 (hand) rotates around Y
    };

    private Vector3 gripperOffset = new Vector3(0f, -0.07f, 0f); // Increased Y offset

    void Start()
    {
        // Initialize joints to a safe starting position
        ResetJointsToSafePosition();
    }

    void ResetJointsToSafePosition()
    {
        if (joints == null || joints.Length == 0) return;

        // Set each joint to the middle of its range
        for (int i = 0; i < joints.Length; i++)
        {
            float midPoint = (jointLimits[i].x + jointLimits[i].y) / 2f;
            Vector3 rotation = Vector3.zero;
            
            // Apply rotation around the correct axis
            if (jointAxes[i] == Vector3.up)
                rotation.y = midPoint;
            else if (jointAxes[i] == Vector3.forward)
                rotation.z = midPoint;
            
            joints[i].localRotation = Quaternion.Euler(rotation);
        }
    }

    void Update()
    {
        if (joints == null || joints.Length == 0) return;

        // Debug distance to target
        float distanceToTarget = Vector3.Distance(endEffector.position, target.position);
        Debug.Log($"Distance to target: {distanceToTarget}");
        
        // Only continue iterations if we're not close enough
        if (distanceToTarget > 0.001f)  // Increased precision threshold
        {
            float deltaTime = Time.deltaTime * smoothSpeed;
            for (int i = 0; i < iterations; i++)
            {
                SolveIK(deltaTime);
            }
        }
    }

    void SolveIK(float deltaTime)
    {
        Vector3 targetPos = target.position;
        Vector3 currentPos = endEffector.position + endEffector.TransformDirection(gripperOffset);
        Vector3 error = targetPos - currentPos;

        // Reduced precision threshold to allow closer positioning
        if (error.magnitude < 0.0005f) return;

        Vector3[] jacobian = ComputeJacobian();
        float scaleFactor = 5f; // Increased from 2f to 5f for more decisive movement

        for (int i = 0; i < joints.Length; i++)
        {
            float deltaTheta = Vector3.Dot(jacobian[i], error) * learningRate * scaleFactor;
            float currentAngle = GetCurrentAngle(joints[i], jointAxes[i]);
            if (currentAngle > 180f) currentAngle -= 360f;
            
            // Special handling for joint 2 (arm)
            if (i == 1) 
            {
                // Reduce movement speed but allow more range
                deltaTheta *= 0.3f;
                deltaTheta = Mathf.Clamp(deltaTheta, -5f, 5f);
                
                // Add some margin to prevent getting stuck at limits
                float margin = 2f;
                if ((currentAngle > jointLimits[i].y - margin && deltaTheta > 0) ||
                    (currentAngle < jointLimits[i].x + margin && deltaTheta < 0))
                {
                    continue;
                }
            }
            // Special handling for joint 3 (elbow)
            else if (i == 2) 
            {
                deltaTheta *= 0.5f;
                deltaTheta = Mathf.Clamp(deltaTheta, -3f, 3f);
            }

            float newAngle = currentAngle + deltaTheta;
            if (newAngle < jointLimits[i].x || newAngle > jointLimits[i].y)
            {
                Debug.Log($"Joint {i} limit reached: current={currentAngle}, attempted={newAngle}, limits=({jointLimits[i].x}, {jointLimits[i].y})");
                continue;
            }

            // Apply rotation in local space
            joints[i].Rotate(jointAxes[i] * deltaTheta, Space.Self);
        }
    }

    // Helper function to get current angle based on rotation axis
    private float GetCurrentAngle(Transform joint, Vector3 axis)
    {
        if (axis == Vector3.up)
            return joint.localEulerAngles.y;
        else if (axis == Vector3.forward)
            return joint.localEulerAngles.z;
        else
            return joint.localEulerAngles.x;
    }

    Vector3[] ComputeJacobian()
    {
        Vector3[] jacobian = new Vector3[joints.Length];
        Vector3 currentPos = endEffector.position + endEffector.TransformDirection(gripperOffset);

        for (int i = 0; i < joints.Length; i++)
        {
            Vector3 jointAxis = joints[i].TransformDirection(jointAxes[i]);
            Vector3 r = currentPos - joints[i].position;
            
            float influence = 1.0f - (i / (float)joints.Length) * 0.1f;
            jacobian[i] = Vector3.Cross(jointAxis, r) * influence * 15f;
        }

        return jacobian;
    }

    // Visual Debugging of Joint Axes
    void OnDrawGizmos()
    {
        if (joints == null) return;

        for (int i = 0; i < joints.Length; i++)
        {
            // Draw Local Axes for Each Joint
            Gizmos.color = Color.red; // X-axis
            Gizmos.DrawLine(joints[i].position, joints[i].position + joints[i].right * 0.1f);

            Gizmos.color = Color.green; // Y-axis
            Gizmos.DrawLine(joints[i].position, joints[i].position + joints[i].up * 0.1f);

            Gizmos.color = Color.blue; // Z-axis
            Gizmos.DrawLine(joints[i].position, joints[i].position + joints[i].forward * 0.1f);
        }

        // Draw a Line from the End Effector to the Target
        if (endEffector != null && target != null)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawLine(endEffector.position, target.position);
        }

        if (endEffector != null)
        {
            // Draw the actual end effector point we're using for calculations
            Gizmos.color = Color.magenta;
            Vector3 actualEndEffectorPos = endEffector.position + endEffector.TransformDirection(gripperOffset);
            Gizmos.DrawWireSphere(actualEndEffectorPos, 0.01f);
        }
    }
}


