using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FKManager : MonoBehaviour
{
    public Joint m_root;
    public Transform m_endEffector;
    public LayerMask collisionLayer;
    public float executionSpeed = 1.0f;
    public bool showDebugVisualization = true;

    private List<Joint> m_joints = new List<Joint>();
    private bool m_isExecuting = false;

    void Start()
    {
            Debug.Log("Starting FKManager setup.");

        // Build chain of joints
        Joint current = m_root;
        while (current != null)
        {
            m_joints.Add(current);
                    Debug.Log("Added joint: " + current.name);

            current = current.GetChild();
        }
    }



void Update()
{
    if (m_root == null || m_joints.Count == 0) return;

    // Automatically change poses at intervals
    autoPoseChangeInterval -= Time.deltaTime;
    if (autoPoseChangeInterval <= 0 && !m_isExecuting)
    {
        GoToNextPose();
        autoPoseChangeInterval = 5.0f; // Change pose every 5 seconds
    }

    // Compute and visualize end effector pose (keep this part)
    // ComputeEndEffectorPose(GetCurrentJointAngles());
}

// Member variable to control the timing of automatic pose changes
private float autoPoseChangeInterval = 5.0f;

public void GoToNextPose()
{
    currentPoseIndex = (currentPoseIndex + 1) % predefinedPoses.Length;
    Debug.Log("Changing to next pose: " + currentPoseIndex);
    ExecuteForwardKinematics(predefinedPoses[currentPoseIndex]);
}


    // Optional: Add a method to cycle through preset poses
    private int currentPoseIndex = 0;
    private float[][] predefinedPoses = new float[][]
    {
        new float[] { 45f, -45f, 45f },   // Pose 1
        new float[] { -45f, 45f, -45f },  // Pose 2
        new float[] { 90f, 0f, -90f },    // Pose 3
        new float[] { 0f, 0f, 0f }        // Reset pose
    };

    // public void GoToNextPose()
    // {
    //     if (!m_isExecuting)
    //     {
    //         currentPoseIndex = (currentPoseIndex + 1) % predefinedPoses.Length;
    //         ExecuteForwardKinematics(predefinedPoses[currentPoseIndex]);
    //     }
    // }

    // Add this to show current angles in the Unity Inspector
    void OnGUI()
    {
        if (showDebugVisualization)
        {
            GUILayout.BeginArea(new Rect(10, 10, 300, 200));
            GUILayout.Label("Joint Angles:");
            float[] angles = GetCurrentJointAngles();
            for (int i = 0; i < angles.Length; i++)
            {
                GUILayout.Label($"Joint {i + 1}: {angles[i]:F1} degrees");
            }
            GUILayout.Label("\nControls:");
            GUILayout.Label("Q/A: First Joint");
            GUILayout.Label("W/S: Second Joint");
            GUILayout.Label("E/D: Third Joint");
            GUILayout.Label("R: Reset");
            GUILayout.Label("Space: Next Pose");
            GUILayout.EndArea();
        }
    }
    

    // Main method to execute Forward Kinematics
    public void ExecuteForwardKinematics(float[] jointAngles)
    {
            Debug.Log("Executing FK with angles: " + string.Join(", ", jointAngles));

        if (m_isExecuting || jointAngles.Length != m_joints.Count)
        {
            Debug.LogError("Cannot execute FK: Already executing or invalid number of joint angles");
            
            return;
        }

        // First reset all joints to their initial position
        ResetAllJoints();

        // Validate the target pose
        if (ValidateTargetPose(jointAngles))
        {
            StartCoroutine(ExecuteTrajectory(jointAngles));
        }
        else
        {
            Debug.LogWarning("Target pose is invalid due to collisions or joint limits!");
            ResetAllJoints(); // Reset back to initial position
        }
    }

    private void ResetAllJoints()
    {
        foreach (Joint joint in m_joints)
        {
            joint.ResetRotation();
        }
    }

    private bool ValidateTargetPose(float[] targetAngles)
    {
        bool isValid = true;

        // Try applying rotations and check validity
        for (int i = 0; i < m_joints.Count; i++)
        {
            // Check if rotation is possible within joint limits
            if (!m_joints[i].Rotate(targetAngles[i]))
            {
                isValid = false;
                break;
            }
        }

        // If joint rotations are valid, check for collisions
        if (isValid)
        {
            isValid = !CheckCollisions();
        }

        // Reset joints back to original position
        ResetAllJoints();

        return isValid;
    }

    private bool CheckCollisions()
    {
        // Check each joint for collisions
        foreach (Joint joint in m_joints)
        {
            // Sphere check around joint
            Collider[] colliders = Physics.OverlapSphere(joint.transform.position, 0.1f, collisionLayer);
            if (colliders.Length > 0)
            {
                return true; // Collision detected
            }

            // Check link to child joint
            Joint childJoint = joint.GetChild();
            if (childJoint != null)
            {
                Vector3 direction = childJoint.transform.position - joint.transform.position;
                float distance = direction.magnitude;
                if (Physics.Raycast(joint.transform.position, direction.normalized, distance, collisionLayer))
                {
                    return true; // Collision detected
                }
            }
        }
        return false; // No collisions
    }

    private IEnumerator ExecuteTrajectory(float[] targetAngles)
    {
        m_isExecuting = true;

        float elapsedTime = 0f;
        float duration = 1f / executionSpeed;

        // Store initial angles
        float[] startAngles = new float[m_joints.Count];
        for (int i = 0; i < m_joints.Count; i++)
        {
            startAngles[i] = m_joints[i].currentAngle;
        }

        while (elapsedTime < duration)
        {
            float t = elapsedTime / duration;
            
            // Reset to initial position and apply interpolated rotations
            ResetAllJoints();
            
            // Apply interpolated rotations
            for (int i = 0; i < m_joints.Count; i++)
            {
                float interpolatedAngle = Mathf.Lerp(startAngles[i], targetAngles[i], t);
                m_joints[i].Rotate(interpolatedAngle - m_joints[i].currentAngle); // Rotate by the difference
            }

            elapsedTime += Time.deltaTime;
            yield return null;
        }

        // Ensure final position is exact
        ResetAllJoints();
        for (int i = 0; i < m_joints.Count; i++)
        {
            m_joints[i].Rotate(targetAngles[i]);
        }

        m_isExecuting = false;
    }

    void OnDrawGizmos()
    {
        if (!showDebugVisualization || !Application.isPlaying) return;

        // Draw joint chain
        for (int i = 0; i < m_joints.Count; i++)
        {
            if (m_joints[i] == null) continue;

            // Draw joint position
            Gizmos.color = Color.blue;
            Gizmos.DrawSphere(m_joints[i].transform.position, 0.05f);

            // Draw link to child
            Joint childJoint = m_joints[i].GetChild();
            if (childJoint != null)
            {
                Gizmos.color = Color.green;
                Gizmos.DrawLine(m_joints[i].transform.position, childJoint.transform.position);
            }

            // Draw rotation axis
            Gizmos.color = Color.red;
            Gizmos.DrawRay(m_joints[i].transform.position, m_joints[i].rotationAxis * 0.1f);
        }
    }

    // Helper method to get current joint angles
    public float[] GetCurrentJointAngles()
    {
        float[] angles = new float[m_joints.Count];
        for (int i = 0; i < m_joints.Count; i++)
        {
            angles[i] = m_joints[i].currentAngle;
        }
        return angles;
    }
}
