using UnityEngine;

public class InverseKinematics : MonoBehaviour
{
    [Header("Joint Configuration")]
    public ArticulationBody[] joints;
    private float[] jointAngles;
    
    [Header("Joint Limits (in degrees)")]
    public float[] jointLimitsLower = { -175f, -90f, -80f, -175f, -100f, -147.5f };
    public float[] jointLimitsUpper = { 175f, 36.7f, 90f, 175f, 110f, 147.5f };

    [Header("End Effector Configuration")]
    public Transform endEffector;  // Reference to the end effector transform
    private Vector3 endEffectorOffset;  // Offset from the last joint to the end effector

    [Header("Inverse Kinematics Parameters")]
    public GameObject targetDot;
    [Range(0.1f, 5.0f)]
    public float learningRate = 2.0f;
    [Range(10, 1000)]
    public int maxIterations = 1000;
    [Range(0.001f, 0.1f)]
    public float positionThreshold = 0.01f;

    private float[] gradients;

    private void Start()
    {
        int jointCount = joints.Length;
        jointAngles = new float[jointCount];
        gradients = new float[jointCount];

        for (int i = 0; i < jointCount; i++)
        {
            jointAngles[i] = joints[i].xDrive.target;
        }

        // Calculate the offset from the last joint to the end effector in local space
        if (endEffector != null && joints.Length > 0)
        {
            endEffectorOffset = joints[joints.Length - 1].transform.InverseTransformPoint(endEffector.position);
        }
    }

    private void Update()
    {
        if (targetDot != null)
        {
            PerformGDIK(targetDot.transform.position);
        }
    }

    private void PerformGDIK(Vector3 targetPosition)
    {
        for (int iteration = 0; iteration < maxIterations; iteration++)
        {
            Matrix4x4 currentTransform = CalculateForwardKinematics(jointAngles);
            Vector3 currentEndEffectorPosition = GetEndEffectorPosition(currentTransform);
            float positionError = Vector3.Distance(targetPosition, currentEndEffectorPosition);

            if (positionError < positionThreshold)
            {
                break;
            }

            ComputeGradients(positionError, targetPosition);
            
            for (int j = 0; j < jointAngles.Length; j++)
            {
                jointAngles[j] -= learningRate * gradients[j];
                jointAngles[j] = Mathf.Clamp(jointAngles[j], jointLimitsLower[j], jointLimitsUpper[j]);
                
                ArticulationDrive drive = joints[j].xDrive;
                drive.target = jointAngles[j];
                joints[j].xDrive = drive;
            }
        }
    }

    private void ComputeGradients(float positionError, Vector3 targetPosition)
    {
        const float perturbation = 2.5f;
        
        for (int i = 0; i < jointAngles.Length; i++)
        {
            float originalAngle = jointAngles[i];
            jointAngles[i] += perturbation;

            Matrix4x4 perturbedTransform = CalculateForwardKinematics(jointAngles);
            Vector3 perturbedPosition = GetEndEffectorPosition(perturbedTransform);
            float positionDifference = Vector3.Distance(perturbedPosition, targetPosition);
            
            gradients[i] = (positionDifference - positionError) / 0.5f;
            jointAngles[i] = originalAngle;
        }
    }

    private Vector3 GetEndEffectorPosition(Matrix4x4 transform)
    {
        // Transform the end effector offset by the final transformation matrix
        Vector3 position = transform.MultiplyPoint3x4(endEffectorOffset);
        return position;
    }

    private Matrix4x4 CalculateForwardKinematics(float[] joints)
    {
        Matrix4x4 T_shoulder = CreateTransformationMatrix(-Vector3.up, joints[0], new Vector3(0, 0.103f, 0));
        Matrix4x4 T_arm = CreateTransformationMatrix(-Vector3.right, joints[1], new Vector3(0, 0.08f, 0));
        Matrix4x4 T_elbow = CreateTransformationMatrix(-Vector3.right, joints[2], new Vector3(0, 0.21f, 0));
        Matrix4x4 T_forearm = CreateTransformationMatrix(Vector3.forward, joints[3], new Vector3(0, 0.03f, 0.0415f));
        Matrix4x4 T_wrist = CreateTransformationMatrix(-Vector3.up, joints[4], new Vector3(0, 0, 0.18f));
        Matrix4x4 T_hand = CreateTransformationMatrix(Vector3.up, joints[5], new Vector3(0.0164f, -0.0055f, 0));

        return T_shoulder * T_arm * T_elbow * T_forearm * T_wrist * T_hand;
    }

    private Matrix4x4 CreateTransformationMatrix(Vector3 axis, float angle, Vector3 translation)
    {
        return Matrix4x4.Translate(translation) * Matrix4x4.Rotate(Quaternion.AngleAxis(angle, axis));
    }

    
}