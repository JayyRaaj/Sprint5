// using UnityEngine;

// public class InverseKinematics : MonoBehaviour
// {
//     [Header("Joint Configuration")]
//     public ArticulationBody[] joints;
//     public float[] jointAngles;
    
//     [Header("Joint Limits")]
//     public float[] jointLimitsLower = new float[] { -3.05433f, -1.5708f, -1.397485f, -3.05433f, -1.74533f, -2.57436f };
//     public float[] jointLimitsUpper = new float[] { 3.05433f, 0.640187f, 1.5708f, 3.05433f, 1.91986f, 2.57436f };

//     [Header("IK Parameters")]
//     [Range(0.1f, 5.0f)]
//     public float learningRate = 2.0f;
//     [Range(10, 2000)]
//     public int maxIterations = 1000;
//     [Range(0.001f, 0.1f)]
//     public float positionThreshold = 0.01f;
//     public float dampingFactor = 0.1f;  // Added damping for stability
    
//     [Header("Target")]
//     public Transform target;
    
//     private float[] previousGradients;
//     private float[] velocities;  // For momentum
//     private const float momentum = 0.9f;
//     private const float epsilon = 1e-6f;  // Small value to prevent division by zero

//     void Start()
//     {
//         InitializeArrays();
//         ConvertJointLimitsToRadians();
//     }

//     private void InitializeArrays()
//     {
//         int jointCount = joints.Length;
//         jointAngles = new float[jointCount];
//         previousGradients = new float[jointCount];
//         velocities = new float[jointCount];

//         for (int i = 0; i < jointCount; i++)
//         {
//             jointAngles[i] = joints[i].xDrive.target;
//         }
//     }

//     private void ConvertJointLimitsToRadians()
//     {
//         for (int i = 0; i < joints.Length; i++)
//         {
//             jointLimitsLower[i] *= Mathf.Deg2Rad;
//             jointLimitsUpper[i] *= Mathf.Deg2Rad;
//         }
//     }

//     void Update()
//     {
//         if (target != null)
//         {
//             PerformGDIK(target.position);
//         }
//     }

//     void PerformGDIK(Vector3 targetPosition)
//     {
//         float previousError = float.MaxValue;
//         int stagnationCount = 0;
//         const int maxStagnation = 5;
//         const float stagnationThreshold = 0.001f;

//         for (int iteration = 0; iteration < maxIterations; iteration++)
//         {
//             // Calculate current end effector position
//             Matrix4x4 endEffectorTransform = CalculateForwardKinematics(jointAngles);
//             Vector3 currentPosition = endEffectorTransform.GetColumn(3);
            
//             // Calculate error
//             float positionError = Vector3.Distance(targetPosition, currentPosition);
            
//             // Check for convergence
//             if (positionError < positionThreshold)
//             {
//                 break;
//             }

//             // Check for stagnation
//             if (Mathf.Abs(previousError - positionError) < stagnationThreshold)
//             {
//                 stagnationCount++;
//                 if (stagnationCount > maxStagnation)
//                 {
//                     // Randomly perturb joints to escape local minimum
//                     PerturbJoints();
//                     stagnationCount = 0;
//                 }
//             }
//             else
//             {
//                 stagnationCount = 0;
//             }

//             // Compute and apply gradients with momentum and adaptive learning
//             UpdateJointAngles(targetPosition, positionError);
            
//             previousError = positionError;
//         }

//         // Apply final joint angles
//         ApplyJointMovements();
//     }

//     private void UpdateJointAngles(Vector3 targetPosition, float currentError)
//     {
//         float[] gradients = ComputeGradients(currentError, targetPosition);
        
//         for (int i = 0; i < jointAngles.Length; i++)
//         {
//             // Apply momentum
//             velocities[i] = momentum * velocities[i] - learningRate * gradients[i];
            
//             // Update joint angle with damping
//             float update = velocities[i] + dampingFactor * (previousGradients[i] - gradients[i]);
//             jointAngles[i] += update;
            
//             // Clamp to joint limits
//             jointAngles[i] = Mathf.Clamp(jointAngles[i], jointLimitsLower[i], jointLimitsUpper[i]);
            
//             previousGradients[i] = gradients[i];
//         }
//     }

//     private void PerturbJoints()
//     {
//         for (int i = 0; i < jointAngles.Length; i++)
//         {
//             float randomPerturbation = Random.Range(-0.1f, 0.1f);
//             jointAngles[i] += randomPerturbation;
//             jointAngles[i] = Mathf.Clamp(jointAngles[i], jointLimitsLower[i], jointLimitsUpper[i]);
//         }
//     }

//     float[] ComputeGradients(float positionError, Vector3 targetPosition)
//     {
//         float[] gradients = new float[jointAngles.Length];
//         const float delta = 0.01f; // Smaller perturbation for more accurate gradients

//         for (int i = 0; i < jointAngles.Length; i++)
//         {
//             float originalAngle = jointAngles[i];

//             // Forward difference for gradient computation
//             jointAngles[i] += delta;
//             Matrix4x4 forwardTransform = CalculateForwardKinematics(jointAngles);
//             Vector3 forwardPosition = forwardTransform.GetColumn(3);
//             float forwardError = Vector3.Distance(forwardPosition, targetPosition);

//             // Backward difference for more stable gradients
//             jointAngles[i] = originalAngle - delta;
//             Matrix4x4 backwardTransform = CalculateForwardKinematics(jointAngles);
//             Vector3 backwardPosition = backwardTransform.GetColumn(3);
//             float backwardError = Vector3.Distance(backwardPosition, targetPosition);

//             // Central difference formula
//             gradients[i] = (forwardError - backwardError) / (2 * delta);

//             // Restore original angle
//             jointAngles[i] = originalAngle;
//         }

//         return gradients;
//     }

//     private void ApplyJointMovements()
//     {
//         for (int i = 0; i < joints.Length; i++)
//         {
//             ArticulationDrive drive = joints[i].xDrive;
//             drive.target = jointAngles[i];
//             joints[i].xDrive = drive;
//         }
//     }

// }