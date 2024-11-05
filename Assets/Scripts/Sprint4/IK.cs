using UnityEngine;


public class NiryoOneIK : MonoBehaviour
{
    public Transform target; // Reference to the Target cube
    public Transform endEffector; // Reference to the end effector of the robot
    public ArticulationBody[] joints; // Array for all six joint articulations
    public float threshold = 0.1f; // Distance threshold to consider end effector at the target
    public int maxIterations = 10; // Maximum number of IK iterations per frame
    public float stepSize = 5f; // Degree change per iteration for each joint

    private void Update()
    {
        PerformInverseKinematics();
    }

    private void PerformInverseKinematics()
    {
        if (Vector3.Distance(endEffector.position, target.position) < threshold)
            return; // End effector is close enough to the target

        for (int iteration = 0; iteration < maxIterations; iteration++)
        {
            for (int i = joints.Length - 1; i >= 0; i--)
            {
                ArticulationBody joint = joints[i];

                // Get direction vectors
                Vector3 toEndEffector = endEffector.position - joint.transform.position;
                Vector3 toTarget = target.position - joint.transform.position;

                // Calculate rotation angle to bring end effector closer to target
                float angle = Vector3.SignedAngle(toEndEffector, toTarget, joint.transform.up);

                // Limit the angle to stepSize to ensure gradual movement
                angle = Mathf.Clamp(angle, -stepSize, stepSize);

                // Apply rotation to the joint
                Quaternion rotation = Quaternion.AngleAxis(angle, joint.transform.up);
                joint.transform.rotation = rotation * joint.transform.rotation;

                // Update articulation body joint angle
                var drive = joint.xDrive;
                drive.target = angle;
                joint.xDrive = drive;

                // Check if the end effector is close enough to the target
                if (Vector3.Distance(endEffector.position, target.position) < threshold)
                    return;
            }
        }
    }
}
