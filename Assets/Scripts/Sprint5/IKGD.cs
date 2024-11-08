using UnityEngine;

public class InverseKinematicsGD : MonoBehaviour
{
    public Transform[] joints; // Array of joints
    public Transform target; // Target position for end effector
    public float learningRate = 0.01f;
    public float tolerance = 0.01f;

    private void Update()
    {
        GradientDescentIKGD();
    }

    void GradientDescentIKGD()
    {
        float error = Vector3.Distance(joints[joints.Length - 1].position, target.position);

        while (error > tolerance)
        {
            for (int i = joints.Length - 1; i >= 0; i--)
            {
                Vector3 currentPos = joints[joints.Length - 1].position;
                Vector3 partialDerivative = CalculatePartialDerivative(i);

                joints[i].rotation *= Quaternion.Euler(partialDerivative * -learningRate);

                error = Vector3.Distance(joints[joints.Length - 1].position, target.position);
                if (error <= tolerance) break;
            }
        }
    }

    Vector3 CalculatePartialDerivative(int jointIndex)
    {
        float deltaTheta = 0.01f;
        Vector3 originalPosition = joints[joints.Length - 1].position;

        joints[jointIndex].rotation *= Quaternion.Euler(Vector3.up * deltaTheta);
        Vector3 newPosition = joints[joints.Length - 1].position;
        joints[jointIndex].rotation *= Quaternion.Euler(Vector3.up * -deltaTheta);

        return (newPosition - originalPosition) / deltaTheta;
    }
}
