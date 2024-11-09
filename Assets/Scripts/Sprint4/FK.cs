using UnityEngine;
using System.Collections.Generic;

public class FK : MonoBehaviour
{
    // DH parameters
    private readonly float[] d = { 0.029f, 0f, 0.4915275f, -0.3534305f, 0.4395067f, 0f };
    private readonly float[] a = { 0.3995009f, 0f, 1.612003f, 2.767049f, 2.649052f, 0f };
    private readonly float[] alpha = { -0.1075475f, 0f, -0.02583352f, 0f, 0f, 0.1105957f };

    [Header("Joint Angles")]
    [Range(-180f, 180f)] public float jointAngle0; // Base rotation
    [Range(-180f, 180f)] public float jointAngle1; // Shoulder Y-axis
    [Range(-180f, 180f)] public float jointAngle2; // Elbow X-axis
    [Range(-180f, 180f)] public float jointAngle3; // Wrist Z-axis
    [Range(-180f, 180f)] public float jointAngle4; // Wrist rotation
    [Range(-180f, 180f)] public float jointAngle5; // End effector

    private float[] jointAnglesRadians = new float[6];

    [Header("Robot Configuration")]
    public List<ArticulationBody> joints = new List<ArticulationBody>();
    
    [Header("End Effector")]
    public Vector3 expectedPosition;
    public Quaternion expectedRotation;

    [Header("Joint Settings")]
    [Range(0f, 50000f)] public float jointStiffness = 10000f;
    [Range(0f, 1000f)] public float jointDamping = 100f;
    [Range(0f, 1000f)] public float forceLimit = 100f;
    [Range(0f, 1f)] public float jointFriction = 0f;
    [Range(0f, 1f)] public float angularDamping = 0.05f;

    private void Start()
    {
        ValidateSetup();
        InitializeJoints();
    }

    private void ValidateSetup()
    {
        if (joints.Count != 6)
        {
            Debug.LogError("Robot requires exactly 6 joints");
            enabled = false;
            return;
        }

        foreach (var joint in joints)
        {
            if (joint == null)
            {
                Debug.LogError("Missing joint reference");
                enabled = false;
                return;
            }
        }
    }

    private void InitializeJoints()
    {
        // Configure each joint with its proper axis of rotation
        for (int i = 0; i < joints.Count; i++)
        {
            var joint = joints[i];
            joint.jointType = ArticulationJointType.RevoluteJoint;

            // Set the rotation axis based on the joint
            var drive = joint.xDrive;
            drive.stiffness = jointStiffness;
            drive.damping = jointDamping;
            drive.forceLimit = forceLimit;
            
            // Configure joint rotation axes and limits
            switch (i)
            {
                case 0: // Base - Rotate around Y axis
                    ConfigureJoint(joint, Quaternion.Euler(0, 90, 0), -180f, 180f);
                    break;
                    
                case 1: // Shoulder - Rotate around Y axis
                    ConfigureJoint(joint, Quaternion.Euler(0, 90, 0), -180f, 180f);
                    break;
                    
                case 2: // Elbow - Rotate around X axis
                    ConfigureJoint(joint, Quaternion.Euler(90, 0, 0), -180f, 180f);
                    break;
                    
                case 3: // Wrist - Rotate around Z axis
                    ConfigureJoint(joint, Quaternion.Euler(0, 0, 90), -180f, 180f);
                    break;
                    
                case 4: // Wrist rotation
                    ConfigureJoint(joint, Quaternion.Euler(0, 90, 0), -180f, 180f);
                    break;
                    
                case 5: // End effector
                    ConfigureJoint(joint, Quaternion.Euler(0, 90, 0), -180f, 180f);
                    break;
            }

            joint.xDrive = drive;
            joint.jointFriction = jointFriction;
            joint.angularDamping = angularDamping;
        }
    }

    private void ConfigureJoint(ArticulationBody joint, Quaternion rotation, float minLimit, float maxLimit)
    {
        joint.anchorRotation = rotation;
        joint.twistLock = ArticulationDofLock.LimitedMotion;
        
        var drive = joint.xDrive;
        drive.lowerLimit = minLimit;
        drive.upperLimit = maxLimit;
        joint.xDrive = drive;
    }

    void Update()
    {
        if (!enabled) return;

        // Convert joint angles to radians
        jointAnglesRadians[0] = jointAngle0 * Mathf.Deg2Rad;
        jointAnglesRadians[1] = jointAngle1 * Mathf.Deg2Rad;
        jointAnglesRadians[2] = jointAngle2 * Mathf.Deg2Rad;
        jointAnglesRadians[3] = jointAngle3 * Mathf.Deg2Rad;
        jointAnglesRadians[4] = jointAngle4 * Mathf.Deg2Rad;
        jointAnglesRadians[5] = jointAngle5 * Mathf.Deg2Rad;

        // Update joint positions
        for (int i = 0; i < joints.Count; i++)
        {
            SetJointTarget(joints[i], jointAnglesRadians[i]);
        }

        // Calculate forward kinematics
        Matrix4x4 transform = CalculateForwardKinematics();
        expectedPosition = ExtractPosition(transform);
        expectedRotation = ExtractRotation(transform);
    }

    private void SetJointTarget(ArticulationBody joint, float angleRadians)
    {
        var drive = joint.xDrive;
        drive.target = angleRadians * Mathf.Rad2Deg;
        drive.stiffness = jointStiffness;
        drive.damping = jointDamping;
        drive.forceLimit = forceLimit;
        joint.xDrive = drive;
    }

    private Matrix4x4 CalculateForwardKinematics()
    {
        Matrix4x4 transform = Matrix4x4.identity;

        for (int i = 0; i < jointAnglesRadians.Length; i++)
        {
            transform *= CalculateDHMatrix(i);
        }

        return transform;
    }

    private Matrix4x4 CalculateDHMatrix(int jointIndex)
    {
        float theta = jointAnglesRadians[jointIndex];
        float d_i = d[jointIndex];
        float a_i = a[jointIndex];
        float alpha_i = alpha[jointIndex];

        float cosTheta = Mathf.Cos(theta);
        float sinTheta = Mathf.Sin(theta);
        float cosAlpha = Mathf.Cos(alpha_i);
        float sinAlpha = Mathf.Sin(alpha_i);

        Matrix4x4 matrix = Matrix4x4.identity;

        // First row
        matrix.m00 = cosTheta;
        matrix.m01 = -sinTheta * cosAlpha;
        matrix.m02 = sinTheta * sinAlpha;
        matrix.m03 = a_i * cosTheta;

        // Second row
        matrix.m10 = sinTheta;
        matrix.m11 = cosTheta * cosAlpha;
        matrix.m12 = -cosTheta * sinAlpha;
        matrix.m13 = a_i * sinTheta;

        // Third row
        matrix.m20 = 0;
        matrix.m21 = sinAlpha;
        matrix.m22 = cosAlpha;
        matrix.m23 = d_i;

        // Fourth row
        matrix.m30 = 0;
        matrix.m31 = 0;
        matrix.m32 = 0;
        matrix.m33 = 1;

        return matrix;
    }

    private Vector3 ExtractPosition(Matrix4x4 transform)
    {
        Vector4 pos = transform.GetColumn(3);
        // Convert from DH convention to Unity's coordinate system
        return new Vector3(pos.y, pos.z, -pos.x);
    }

    private Quaternion ExtractRotation(Matrix4x4 transform)
    {
        // Extract the rotation matrix (3x3) from the transformation matrix
        Vector3 forward = new Vector3(transform.m01, transform.m11, transform.m21).normalized;
        Vector3 upward = new Vector3(transform.m02, transform.m12, transform.m22).normalized;
        Vector3 right = Vector3.Cross(upward, forward);
        
        // Create rotation matrix
        Matrix4x4 rotMatrix = Matrix4x4.identity;
        rotMatrix.SetColumn(0, new Vector4(right.x, right.y, right.z, 0));
        rotMatrix.SetColumn(1, new Vector4(upward.x, upward.y, upward.z, 0));
        rotMatrix.SetColumn(2, new Vector4(forward.x, forward.y, forward.z, 0));
        
        // Convert to quaternion
        return QuaternionFromMatrix(rotMatrix);
    }

    private Quaternion QuaternionFromMatrix(Matrix4x4 m)
    {
        // Convert rotation matrix to quaternion
        Quaternion q = Quaternion.identity;
        
        float trace = m.m00 + m.m11 + m.m22;
        float s;

        if (trace > 0)
        {
            s = Mathf.Sqrt(trace + 1.0f) * 2;
            q.w = 0.25f * s;
            q.x = (m.m21 - m.m12) / s;
            q.y = (m.m02 - m.m20) / s;
            q.z = (m.m10 - m.m01) / s;
        }
        else if ((m.m00 > m.m11) && (m.m00 > m.m22))
        {
            s = Mathf.Sqrt(1.0f + m.m00 - m.m11 - m.m22) * 2;
            q.w = (m.m21 - m.m12) / s;
            q.x = 0.25f * s;
            q.y = (m.m01 + m.m10) / s;
            q.z = (m.m02 + m.m20) / s;
        }
        else if (m.m11 > m.m22)
        {
            s = Mathf.Sqrt(1.0f + m.m11 - m.m00 - m.m22) * 2;
            q.w = (m.m02 - m.m20) / s;
            q.x = (m.m01 + m.m10) / s;
            q.y = 0.25f * s;
            q.z = (m.m12 + m.m21) / s;
        }
        else
        {
            s = Mathf.Sqrt(1.0f + m.m22 - m.m00 - m.m11) * 2;
            q.w = (m.m10 - m.m01) / s;
            q.x = (m.m02 + m.m20) / s;
            q.y = (m.m12 + m.m21) / s;
            q.z = 0.25f * s;
        }

        return q;
    }

    private void OnDrawGizmos()
    {
        if (!Application.isPlaying || !enabled) return;

        // Draw end effector position
        Gizmos.color = Color.green;
        Gizmos.DrawWireSphere(expectedPosition, 0.05f);

        // Draw coordinate frame
        DrawCoordinateFrame(expectedPosition, expectedRotation, 0.1f);

        // Draw joint axes
        if (joints != null && joints.Count > 0)
        {
            foreach (var joint in joints)
            {
                if (joint != null)
                {
                    Gizmos.color = Color.yellow;
                    Gizmos.DrawWireSphere(joint.transform.position, 0.02f);
                    Gizmos.color = Color.red;
                    Gizmos.DrawRay(joint.transform.position, joint.transform.right * 0.1f);
                }
            }
        }
    }

    private void DrawCoordinateFrame(Vector3 position, Quaternion rotation, float size)
    {
        Gizmos.color = Color.red;
        Gizmos.DrawRay(position, rotation * Vector3.right * size);
        Gizmos.color = Color.green;
        Gizmos.DrawRay(position, rotation * Vector3.up * size);
        Gizmos.color = Color.blue;
        Gizmos.DrawRay(position, rotation * Vector3.forward * size);
    }
}