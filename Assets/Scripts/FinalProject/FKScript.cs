using UnityEngine;

public class RobotJointController : MonoBehaviour
{
    [Header("Robot Joints")]
    public ArticulationBody[] joints;

    [Header("Joint Angles")]
    [Range(-175f, 175f)] public float joint1 = 0f;
    [Range(-110f, 36.68f)] public float joint2 = 0f;
    [Range(-80.07f, 90f)] public float joint3 = 0f;
    [Range(-175f, 175f)] public float joint4 = 0f;
    [Range(-100f, 110f)] public float joint5 = 0f;
    [Range(-147.5f, 147.5f)] public float joint6 = 0f;

    private readonly float d1 = 0.103f;
    private readonly float a1 = 0.21f;
    private readonly float d4 = 0.18f;
    private readonly float d6 = 0.0319f;

    private float[] previousJoints = new float[6];

    void Update()
    {
        if (joints == null || joints.Length < 6)
        {
            Debug.LogError("Assign all six joints in the Inspector.");
            return;
        }

        float[] currentJoints = { joint1, joint2, joint3, joint4, joint5, joint6 };
        bool jointsChanged = false;

        // Check for changes
        for (int i = 0; i < 6; i++)
        {
            if (!Mathf.Approximately(currentJoints[i], previousJoints[i]))
            {
                jointsChanged = true;
                break;
            }
        }

        if (!jointsChanged) return;

        // Update joints and FK
        for (int i = 0; i < joints.Length; i++)
        {
            SetJointRotation(joints[i], currentJoints[i]);
        }

        Matrix4x4 fkMatrix = CalculateFK();
        Debug.Log($"End Effector Position: X={fkMatrix.m03:F4}, Y={fkMatrix.m13:F4}, Z={fkMatrix.m23:F4}");
        Debug.Log($"FK Matrix: {fkMatrix}");

        currentJoints.CopyTo(previousJoints, 0);
    }

    private void SetJointRotation(ArticulationBody joint, float angle)
    {
        var drive = joint.xDrive;
        drive.target = angle;
        joint.xDrive = drive;
    }

    public Matrix4x4 CalculateFK()
    {
        float q1 = joint1 * Mathf.Deg2Rad;
        float q2 = joint2 * Mathf.Deg2Rad;
        float q3 = joint3 * Mathf.Deg2Rad;
        float q4 = joint4 * Mathf.Deg2Rad;
        float q5 = joint5 * Mathf.Deg2Rad;
        float q6 = joint6 * Mathf.Deg2Rad;

        // DH parameters [theta, d, a, alpha]
        Matrix4x4 T01 = GetDHTransform(q1, d1, 0, -Mathf.PI/2);
        Matrix4x4 T12 = GetDHTransform(q2, 0, a1, 0);
        Matrix4x4 T23 = GetDHTransform(q3, 0, 0, Mathf.PI/2);
        Matrix4x4 T34 = GetDHTransform(q4, d4, 0, -Mathf.PI/2);
        Matrix4x4 T45 = GetDHTransform(q5, 0, 0, Mathf.PI/2);
        Matrix4x4 T56 = GetDHTransform(q6, d6, 0, 0);

        return T01 * T12 * T23 * T34 * T45 * T56;
    }

    private Matrix4x4 GetDHTransform(float theta, float d, float a, float alpha)
    {
        Matrix4x4 T = Matrix4x4.identity;
        float ct = Mathf.Cos(theta);
        float st = Mathf.Sin(theta);
        float ca = Mathf.Cos(alpha);
        float sa = Mathf.Sin(alpha);

        T.SetRow(0, new Vector4(ct, -st*ca, st*sa, a*ct));
        T.SetRow(1, new Vector4(st, ct*ca, -ct*sa, a*st));
        T.SetRow(2, new Vector4(0, sa, ca, d));
        T.SetRow(3, new Vector4(0, 0, 0, 1));

        return T;
    }
}
