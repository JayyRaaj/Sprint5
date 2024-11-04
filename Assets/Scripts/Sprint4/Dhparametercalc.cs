using UnityEngine;
using System.Collections.Generic;
#if UNITY_EDITOR
using UnityEditor;
#endif

public class DHParameterCalculator : MonoBehaviour
{
    [Header("Robot Configuration")]
    [SerializeField] private List<ArticulationBody> joints = new List<ArticulationBody>();
    [SerializeField] private Transform endEffector;

    [Header("Calculated Parameters")]
    [SerializeField] private float[] linkLengths = new float[6];    // a parameters
    [SerializeField] private float[] linkOffsets = new float[6];    // d parameters
    [SerializeField] private float[] twistAngles = new float[6];    // alpha parameters
    [SerializeField] private Vector2[] jointLimits = new Vector2[6]; // min/max angles

    [Header("Debug")]
    [SerializeField] private bool showGizmos = true;
    [SerializeField] private float gizmoSize = 0.05f;

    private void OnValidate()
    {
        if (joints.Count != 6)
        {
            Debug.LogWarning("Please assign exactly 6 joints in order from base to end effector.");
        }
    }

    [ContextMenu("Calculate DH Parameters")]
    public void CalculateDHParameters()
    {
        if (joints.Count != 6 || endEffector == null)
        {
            Debug.LogError("Please assign all joints and end effector before calculating!");
            return;
        }

        // Store original joint positions
        float[] originalAngles = new float[6];
        for (int i = 0; i < 6; i++)
        {
            ArticulationBody joint = joints[i];
            originalAngles[i] = joint.xDrive.target;
        }

        // Set all joints to zero position
        SetAllJointsToZero();

        // Calculate parameters for each joint
        for (int i = 0; i < 6; i++)
        {
            CalculateParametersForJoint(i);
        }

        // Restore original positions
        for (int i = 0; i < 6; i++)
        {
            ArticulationDrive drive = joints[i].xDrive;
            drive.target = originalAngles[i];
            joints[i].xDrive = drive;
        }

        // Find joint limits
        for (int i = 0; i < 6; i++)
        {
            ArticulationBody joint = joints[i];
            jointLimits[i] = new Vector2(joint.xDrive.lowerLimit, joint.xDrive.upperLimit);
        }

        Debug.Log("DH Parameters calculated! Check the inspector for results.");
        PrintDHParameters();
    }

    private void SetAllJointsToZero()
    {
        foreach (var joint in joints)
        {
            ArticulationDrive drive = joint.xDrive;
            drive.target = 0;
            joint.xDrive = drive;
        }
    }

    private void CalculateParametersForJoint(int index)
    {
        ArticulationBody currentJoint = joints[index];
        ArticulationBody nextJoint = (index < 5) ? joints[index + 1] : null;
        Transform nextTransform = (nextJoint != null) ? nextJoint.transform : endEffector;

        // Get joint axes in world space
        Vector3 currentAxis = currentJoint.transform.right; // Assuming X-axis drive
        Vector3 nextAxis = (nextJoint != null) ? nextJoint.transform.right : Vector3.right;

        // Calculate link length (a) - perpendicular distance between joint axes
        Vector3 connection = nextTransform.position - currentJoint.transform.position;
        linkLengths[index] = Vector3.ProjectOnPlane(connection, currentAxis).magnitude;

        // Calculate link offset (d) - distance along joint axis
        linkOffsets[index] = Vector3.Dot(connection, currentAxis);

        // Calculate twist angle (alpha) - angle between joint axes
        twistAngles[index] = Vector3.SignedAngle(currentAxis, nextAxis, Vector3.up) * Mathf.Deg2Rad;
    }

    private void PrintDHParameters()
    {
        Debug.Log("=== DH Parameters ===");
        Debug.Log("Link Lengths (a):");
        for (int i = 0; i < 6; i++)
            Debug.Log($"a[{i}] = {linkLengths[i]:F4}");

        Debug.Log("\nLink Offsets (d):");
        for (int i = 0; i < 6; i++)
            Debug.Log($"d[{i}] = {linkOffsets[i]:F4}");

        Debug.Log("\nTwist Angles (alpha) in radians:");
        for (int i = 0; i < 6; i++)
            Debug.Log($"alpha[{i}] = {twistAngles[i]:F4}");

        Debug.Log("\nJoint Limits (degrees):");
        for (int i = 0; i < 6; i++)
            Debug.Log($"Joint {i}: Min = {jointLimits[i].x:F1}, Max = {jointLimits[i].y:F1}");
    }

    private void OnDrawGizmos()
    {
        if (!showGizmos || joints.Count != 6) return;

        // Draw joint axes and frames
        for (int i = 0; i < joints.Count; i++)
        {
            if (joints[i] == null) continue;

            Transform jointTransform = joints[i].transform;
            Vector3 position = jointTransform.position;

            // Draw joint position
            Gizmos.color = Color.white;
            Gizmos.DrawWireSphere(position, gizmoSize);

            // Draw joint axis
            Gizmos.color = Color.red;
            Gizmos.DrawRay(position, jointTransform.right * gizmoSize * 2);

            // Draw connection to next joint
            if (i < joints.Count - 1 && joints[i + 1] != null)
            {
                Gizmos.color = Color.yellow;
                Gizmos.DrawLine(position, joints[i + 1].transform.position);
            }
            else if (i == joints.Count - 1 && endEffector != null)
            {
                Gizmos.color = Color.green;
                Gizmos.DrawLine(position, endEffector.position);
            }
        }
    }

    #if UNITY_EDITOR
    [CustomEditor(typeof(DHParameterCalculator))]
    public class DHParameterCalculatorEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();

            DHParameterCalculator calculator = (DHParameterCalculator)target;

            EditorGUILayout.Space();
            if (GUILayout.Button("Calculate DH Parameters"))
            {
                calculator.CalculateDHParameters();
            }
        }
    }
    #endif
}