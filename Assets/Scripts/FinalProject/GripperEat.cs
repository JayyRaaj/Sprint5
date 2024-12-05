using UnityEngine;

public class GripperControl : MonoBehaviour
{
    public ArticulationBody gripperJointRight;
    public ArticulationBody gripperJointLeft;

    private float timer;
    private float nextActionTime;

    void Start()
    {
        SetRandomInterval();
    }

    void Update()
    {
        timer += Time.deltaTime;

        if (timer >= nextActionTime)
        {
            ToggleGripper();
            timer = 0f;
        }
    }

    private bool isGripperClosed = false;

    void ToggleGripper()
    {
        if (isGripperClosed)
        {
            OpenGripper();
        }
        else
        {
            CloseGripper();
        }
        isGripperClosed = !isGripperClosed;
    }

    void OpenGripper()
    {
        SetJointTarget(gripperJointRight, 0f); // Neutral position
        SetJointTarget(gripperJointLeft, 0f);  // Neutral position
    }

    void CloseGripper()
    {
        SetJointTarget(gripperJointRight, 1.47f); // Upper limit
        SetJointTarget(gripperJointLeft, -1.47f); // Lower limit
    }

    void SetJointTarget(ArticulationBody joint, float target)
    {
        var drive = joint.xDrive;
        drive.target = target;
        joint.xDrive = drive;
    }

    void SetRandomInterval()
    {
        nextActionTime = 2f; // Fixed 5-second interval instead of random
    }
}
