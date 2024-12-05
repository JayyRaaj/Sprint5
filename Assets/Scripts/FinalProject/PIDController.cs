    using UnityEngine;

public class RobotPIDJointController : MonoBehaviour
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

    [Header("PID Gains")]
    [SerializeField] private float[] Kp = new float[6] { 100f, 100f, 100f, 100f, 100f, 100f }; // Proportional gains
    [SerializeField] private float[] Ki = new float[6] { 0f, 0f, 0f, 0f, 0f, 0f };     // Integral gains
    [SerializeField] private float[] Kd = new float[6] { 20f, 20f, 20f, 20f, 20f, 20f }; // Derivative gains

    private float[] targetAngles = new float[6];
    private float[] previousAngles = new float[6];
    private float[] errorSum = new float[6];
    private float[] lastError = new float[6];

    private void Start()
    {
        // Initialize target angles to current joint positions
        for (int i = 0; i < joints.Length; i++)
        {
            targetAngles[i] = joints[i].xDrive.target;
        }
    }

    private void Update()
    {
        if (joints == null || joints.Length < 6)
        {
            Debug.LogError("Assign all six joints in the Inspector.");
            return;
        }

        // Update target angles based on input sliders
        float[] desiredAngles = { joint1, joint2, joint3, joint4, joint5, joint6 };

        for (int i = 0; i < joints.Length; i++)
        {
            targetAngles[i] = Mathf.Clamp(desiredAngles[i], -180f, 180f);
            float pidOutput = CalculatePID(i, desiredAngles[i], Time.deltaTime);
            SetJointRotation(joints[i], pidOutput);
        }
    }

    private float CalculatePID(int jointIndex, float desiredAngle, float deltaTime)
    {
        float currentAngle = joints[jointIndex].xDrive.target;
        float error = desiredAngle - currentAngle;

        // Proportional term
        float pTerm = Kp[jointIndex] * error;

        // Integral term
        errorSum[jointIndex] += error * deltaTime;
        float iTerm = Ki[jointIndex] * errorSum[jointIndex];

        // Derivative term
        float derivative = (error - lastError[jointIndex]) / deltaTime;
        float dTerm = Kd[jointIndex] * derivative;

        // Save last error for derivative computation
        lastError[jointIndex] = error;

        // PID output
        return pTerm + iTerm + dTerm;
    }

    private void SetJointRotation(ArticulationBody joint, float targetAngle)
    {
        var drive = joint.xDrive;
        drive.target = Mathf.Clamp(targetAngle, drive.lowerLimit, drive.upperLimit);
        joint.xDrive = drive;
    }
}
