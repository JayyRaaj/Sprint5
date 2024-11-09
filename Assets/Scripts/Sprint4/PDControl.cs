using UnityEngine;

public class PDControl : MonoBehaviour
{
    public ArticulationBody[] joints; // Array for all joint articulations
    public float[] stiffness; // Stiffness (P term) for each joint
    public float[] damping; // Damping (D term) for each joint
    public Transform target; // Target position for the end effector

    private void FixedUpdate()
    {
        ApplyPDControl();
    }

    private void ApplyPDControl()
    {
        // Iterate over each joint and apply PD control
        for (int i = 0; i < joints.Length; i++)
        {
            ArticulationBody joint = joints[i];

            // Get the current joint angle and target angle
            float currentAngle = joint.jointPosition[0]; // Get the current joint position
            float targetAngle = CalculateTargetAngle(i); // Implement this method to get the desired target angle

            // Calculate the error
            float error = targetAngle - currentAngle;

            // Calculate the derivative of the error
            float derivative = -joint.jointVelocity[0]; // Get the current joint velocity

            // PD control output
            float controlOutput = (stiffness[i] * error) + (damping[i] * derivative);

            // Apply the control output to the joint drive
            var drive = joint.xDrive;
            drive.target = controlOutput; // Set the target based on PD output
            joint.xDrive = drive;
        }
    }

    private float CalculateTargetAngle(int jointIndex)
    {
       
        return 0f; // Placeholder: Replace this with your calculation logic
    }
}
