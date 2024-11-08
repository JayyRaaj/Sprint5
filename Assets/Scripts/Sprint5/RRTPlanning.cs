using UnityEngine;
using System.Collections.Generic;

public class NiryoOneController : MonoBehaviour
{
    [Header("Robot Configuration")]
    public Transform baseFrame;
    public Transform[] joints;
    public float[] jointMinLimits;
    public float[] jointMaxLimits;
    public Transform endEffector;

    [Header("Path Planning Parameters")]
    public Transform targetPosition;
    public float stepSize = 0.1f;
    public int maxIterations = 1000;
    public float goalThreshold = 0.05f;

    [Header("Environment")]
    public List<Transform> obstacles;
    public float obstacleRadius = 0.1f;
    public float jointRadius = 0.05f;

    private List<JointState> pathStates;
    private bool isExecutingPath = false;
    private int currentPathIndex = 0;

    public struct JointState
    {
        public float[] jointAngles;
        public Vector3 endEffectorPosition;

        public JointState(float[] angles, Vector3 position)
        {
            jointAngles = angles;
            endEffectorPosition = position;
        }
    }

    void Start()
    {
        if (baseFrame == null || joints.Length == 0 || jointMinLimits.Length != joints.Length || jointMaxLimits.Length != joints.Length)
        {
            Debug.LogError("Configuration is incomplete.");
            return;
        }

        PlanPath();
    }

    void Update()
    {
        if (isExecutingPath && pathStates != null && pathStates.Count > 0)
        {
            ExecutePathStep();
        }
    }

    void PlanPath()
    {
        Vector3 targetInBase = baseFrame.InverseTransformPoint(targetPosition.position);
        float[] startAngles = GetCurrentJointAngles();
        Vector3 startPosInBase = baseFrame.InverseTransformPoint(GetEndEffectorPosition());
        JointState startState = new JointState(startAngles, startPosInBase);

        List<JointState> path = RRTPlanning(startState, targetInBase);

        if (path != null)
        {
            pathStates = path;
            isExecutingPath = true;
            Debug.Log("Path found! Number of waypoints: " + path.Count);
        }
        else
        {
            Debug.LogWarning("No valid path found!");
        }
    }

    List<JointState> RRTPlanning(JointState start, Vector3 goalPosition)
    {
        List<JointState> tree = new List<JointState> { start };
        List<int> parents = new List<int> { -1 };

        for (int i = 0; i < maxIterations; i++)
        {
            Vector3 randomPosition = Random.value < 0.1f ? goalPosition : GetRandomValidPosition();

            int nearestIndex = FindNearestState(tree, randomPosition);
            JointState nearestState = tree[nearestIndex];

            JointState newState = GenerateNewState(nearestState, randomPosition);

            if (IsStateValid(newState))
            {
                tree.Add(newState);
                parents.Add(nearestIndex);

                if (Vector3.Distance(newState.endEffectorPosition, goalPosition) < goalThreshold)
                {
                    return ExtractPath(tree, parents, tree.Count - 1);
                }
            }
        }

        return null;
    }

    JointState GenerateNewState(JointState fromState, Vector3 targetPosition)
    {
        float[] newAngles = new float[joints.Length];
        System.Array.Copy(fromState.jointAngles, newAngles, joints.Length);

        for (int i = 0; i < joints.Length; i++)
        {
            float testAngle = newAngles[i] + Random.Range(-stepSize, stepSize);
            testAngle = Mathf.Clamp(testAngle, jointMinLimits[i], jointMaxLimits[i]);
            newAngles[i] = testAngle;
        }

        Vector3 newPosition = ComputeForwardKinematics(newAngles);
        return new JointState(newAngles, newPosition);
    }

    Vector3 ComputeForwardKinematics(float[] jointAngles)
    {
        Quaternion[] originalRotations = new Quaternion[joints.Length];
        for (int i = 0; i < joints.Length; i++)
        {
            originalRotations[i] = joints[i].localRotation;
            joints[i].localRotation = Quaternion.Euler(0, jointAngles[i], 0);
        }

        Vector3 worldPos = endEffector.position;
        Vector3 baseFramePos = baseFrame.InverseTransformPoint(worldPos);

        for (int i = 0; i < joints.Length; i++)
        {
            joints[i].localRotation = originalRotations[i];
        }

        return baseFramePos;
    }

    bool IsStateValid(JointState state)
    {
        for (int i = 0; i < joints.Length; i++)
        {
            if (state.jointAngles[i] < jointMinLimits[i] || state.jointAngles[i] > jointMaxLimits[i])
            {
                return false;
            }
        }

        Quaternion[] originalRotations = new Quaternion[joints.Length];
        for (int i = 0; i < joints.Length; i++)
        {
            originalRotations[i] = joints[i].localRotation;
            joints[i].localRotation = Quaternion.Euler(0, state.jointAngles[i], 0);
        }

        bool isValid = !CheckCollisions();

        for (int i = 0; i < joints.Length; i++)
        {
            joints[i].localRotation = originalRotations[i];
        }

        return isValid;
    }

    bool CheckCollisions()
    {
        foreach (Transform obstacle in obstacles)
        {
            Vector3 obstacleInBase = baseFrame.InverseTransformPoint(obstacle.position);

            foreach (Transform joint in joints)
            {
                Vector3 jointInBase = baseFrame.InverseTransformPoint(joint.position);
                if (Vector3.Distance(obstacleInBase, jointInBase) < (obstacleRadius + jointRadius))
                {
                    return true;
                }
            }
        }

        for (int i = 0; i < joints.Length; i++)
        {
            for (int j = i + 2; j < joints.Length; j++)
            {
                Vector3 joint1InBase = baseFrame.InverseTransformPoint(joints[i].position);
                Vector3 joint2InBase = baseFrame.InverseTransformPoint(joints[j].position);

                if (Vector3.Distance(joint1InBase, joint2InBase) < jointRadius * 2)
                {
                    return true;
                }
            }
        }

        return false;
    }

    void ExecutePathStep()
    {
        if (currentPathIndex >= pathStates.Count)
        {
            isExecutingPath = false;
            Debug.Log("Path execution completed!");
            return;
        }

        JointState targetState = pathStates[currentPathIndex];
        bool reached = true;

        for (int i = 0; i < joints.Length; i++)
        {
            float currentAngle = joints[i].localEulerAngles.y;
            float targetAngle = targetState.jointAngles[i];
            float newAngle = Mathf.MoveTowards(currentAngle, targetAngle, stepSize * Time.deltaTime * 100);
            joints[i].localRotation = Quaternion.Euler(0, newAngle, 0);

            if (Mathf.Abs(newAngle - targetAngle) > 0.1f)
            {
                reached = false;
            }
        }

        if (reached)
        {
            currentPathIndex++;
        }
    }

    private float[] GetCurrentJointAngles()
    {
        float[] angles = new float[joints.Length];
        for (int i = 0; i < joints.Length; i++)
        {
            angles[i] = joints[i].localEulerAngles.y;
        }
        return angles;
    }

    private Vector3 GetEndEffectorPosition()
    {
        return endEffector.position;
    }

    private Vector3 GetRandomValidPosition()
    {
        float radius = 1.0f;
        Vector3 randomWorldPos = Random.insideUnitSphere * radius + baseFrame.position;
        return baseFrame.InverseTransformPoint(randomWorldPos);
    }

    private int FindNearestState(List<JointState> states, Vector3 targetPosition)
    {
        int nearest = 0;
        float minDist = float.MaxValue;

        for (int i = 0; i < states.Count; i++)
        {
            float dist = Vector3.Distance(states[i].endEffectorPosition, targetPosition);
            if (dist < minDist)
            {
                minDist = dist;
                nearest = i;
            }
        }

        return nearest;
    }

    private List<JointState> ExtractPath(List<JointState> tree, List<int> parents, int goalIndex)
    {
        List<JointState> path = new List<JointState>();
        int current = goalIndex;

        while (current != -1)
        {
            path.Add(tree[current]);
            current = parents[current];
        }

        path.Reverse();
        return path;
    }

    void OnDrawGizmos()
    {
        if (baseFrame == null) return;

        if (pathStates != null)
        {
            Gizmos.color = Color.green;
            for (int i = 0; i < pathStates.Count - 1; i++)
            {
                Vector3 p1 = baseFrame.TransformPoint(pathStates[i].endEffectorPosition);
                Vector3 p2 = baseFrame.TransformPoint(pathStates[i + 1].endEffectorPosition);
                Gizmos.DrawLine(p1, p2);
            }
        }

        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(baseFrame.position, 1.0f);

        Gizmos.color = Color.red;
        Gizmos.DrawRay(baseFrame.position, baseFrame.right * 0.2f);
        Gizmos.color = Color.green;
        Gizmos.DrawRay(baseFrame.position, baseFrame.up * 0.2f);
        Gizmos.color = Color.blue;
        Gizmos.DrawRay(baseFrame.position, baseFrame.forward * 0.2f);
    }
}
