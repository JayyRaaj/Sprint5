using UnityEngine;

public class RobotJointControllerrr : MonoBehaviour
{
    [Header("Robot Joints")]
    public ArticulationBody[] joints; // Array of joints

    [Header("Joint Angles")]
    [Range(-175f, 175f)] public float joint1 = 0f;
    [Range(-110f, 36.68f)] public float joint2 = 0f;
    [Range(-80.07f, 90f)] public float joint3 = 0f;
    [Range(-175f, 175f)] public float joint4 = 0f;
    [Range(-100f, 110f)] public float joint5 = 0f;
    [Range(-147.5f, 147.5f)] public float joint6 = 0f;

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

        // Check if joint angles have changed
        for (int i = 0; i < 6; i++)
        {
            if (!Mathf.Approximately(currentJoints[i], previousJoints[i]))
            {
                jointsChanged = true;
                break;
            }
        }

        if (!jointsChanged) return;

        // Update joint rotations
        for (int i = 0; i < joints.Length; i++)
        {
            SetJointRotation(joints[i], currentJoints[i]);
        }

        currentJoints.CopyTo(previousJoints, 0);
    }

    private void SetJointRotation(ArticulationBody joint, float angle)
    {
        var drive = joint.xDrive;
        drive.target = angle;
        joint.xDrive = drive;
    }
}
