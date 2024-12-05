using UnityEngine;

public class PickandPlaceAuto : MonoBehaviour
{
    public Transform target;
    public Transform endEffector;
    public Transform placeLocation; // New: Where to place the object
    public ArticulationBody[] joints;
    public float threshold = 0.1f;
    public int maxIterations = 10;
    public float stepSize = 5f;

    private bool isHolding = false;
    private Vector3 offsetToTarget;

    private void Update()
    {
        if (!isHolding)
        {
            // Move to pick up position
            PerformInverseKinematics(target.position);
            
            // Check if we're close enough to pick up
            if (Vector3.Distance(endEffector.position, target.position) < threshold)
            {
                PickupObject();
            }
        }
        else
        {
            // Move to place position
            PerformInverseKinematics(placeLocation.position);
            
            // Check if we're close enough to place
            if (Vector3.Distance(endEffector.position, placeLocation.position) < threshold)
            {
                ReleaseObject();
            }
        }
    }

    private void PickupObject()
    {
        isHolding = true;
        offsetToTarget = target.position - endEffector.position;
        target.SetParent(endEffector);
    }

    private void ReleaseObject()
    {
        isHolding = false;
        target.SetParent(null);
    }

    private void PerformInverseKinematics(Vector3 targetPosition)
    {
        if (Vector3.Distance(endEffector.position, targetPosition) < threshold)
            return;

        for (int iteration = 0; iteration < maxIterations; iteration++)
        {
            for (int i = joints.Length - 1; i >= 0; i--)
            {
                ArticulationBody joint = joints[i];

                Vector3 toEndEffector = endEffector.position - joint.transform.position;
                Vector3 toTarget = targetPosition - joint.transform.position;

                float angle = Vector3.SignedAngle(toEndEffector, toTarget, joint.transform.up);
                angle = Mathf.Clamp(angle, -stepSize, stepSize);

                Quaternion rotation = Quaternion.AngleAxis(angle, joint.transform.up);
                joint.transform.rotation = rotation * joint.transform.rotation;

                var drive = joint.xDrive;
                drive.target = angle;
                joint.xDrive = drive;

                if (Vector3.Distance(endEffector.position, targetPosition) < threshold)
                    return;
            }
        }
    }
}
