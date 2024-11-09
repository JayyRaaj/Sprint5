using System.Collections.Generic;
using UnityEngine;

public class FABRIK : MonoBehaviour
{
    public Transform[] joints;
    public float tolerance = 0.05f;
    public int maxIterations = 10;

    private float[] segmentLengths;
    private float totalLength;

    void Start()
    {
        InitializeSegments();
    }

    void InitializeSegments()
    {
        segmentLengths = new float[joints.Length - 1];
        totalLength = 0;

        for (int i = 0; i < joints.Length - 1; i++)
        {
            segmentLengths[i] = Vector3.Distance(joints[i].position, joints[i + 1].position);
            totalLength += segmentLengths[i];
        }
    }

    public void MoveTowardsTarget(Vector3 targetPosition)
    {
        if (Vector3.Distance(joints[joints.Length - 1].position, targetPosition) > tolerance)
        {
            PerformFABRIK(targetPosition);
        }
    }

    void PerformFABRIK(Vector3 targetPosition)
    {
        joints[joints.Length - 1].position = targetPosition;

        for (int i = joints.Length - 2; i >= 0; i--)
        {
            float r = Vector3.Distance(joints[i + 1].position, joints[i].position);
            float lambda = segmentLengths[i] / r;
            joints[i].position = (1 - lambda) * joints[i + 1].position + lambda * joints[i].position;
        }

        joints[0].position = transform.position;

        for (int i = 0; i < joints.Length - 1; i++)
        {
            float r = Vector3.Distance(joints[i + 1].position, joints[i].position);
            float lambda = segmentLengths[i] / r;
            joints[i + 1].position = (1 - lambda) * joints[i].position + lambda * joints[i + 1].position;
        }
    }
}
