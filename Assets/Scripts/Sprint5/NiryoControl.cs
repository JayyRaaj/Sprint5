// using System.Collections;
// using System.Collections.Generic;
// using UnityEngine;

// public class RRTPathPlanner : MonoBehaviour
// {
//     public Transform target; // Target to reach
//     public List<Transform> joints; // List of joints in the robot arm
//     public float stepSize = 0.05f; // Step size of the RRT
//     public int maxIterations = 1000; // Maximum number of iterations to find the path

//     void Start()
//     {
//         if (joints == null || joints.Count == 0)
//         {
//             Debug.LogError("Joints are not assigned in the inspector.");
//             return;
//         }
//         StartCoroutine(PlanPathToTarget());
//     }

//     IEnumerator PlanPathToTarget()
//     {
//         List<Vector3> path = RRT(joints[0].position, target.position);
//         foreach (Vector3 position in path)
//         {
//             MoveArm(position); // Simulate arm movement
//             yield return new WaitForSeconds(0.1f); // Wait for the movement to be visible
//         }
//     }

//     void MoveArm(Vector3 targetPosition)
//     {
//         for (int i = 0; i < joints.Count; i++)
//         {
//             Vector3 toTarget = targetPosition - joints[i].position;
//             Quaternion targetRotation = Quaternion.LookRotation(toTarget);
            
//             // Interpolate rotation in smaller steps for smoother control
//             joints[i].rotation = Quaternion.Lerp(joints[i].rotation, targetRotation, stepSize * 0.5f);
//             Debug.Log($"Rotating {joints[i].name} towards {targetRotation.eulerAngles} by step {stepSize * 0.5f}");
//         }
//     }


//     List<Vector3> RRT(Vector3 start, Vector3 goal)
//     {
//         List<Vector3> path = new List<Vector3>();
//         path.Add(start);
//         Vector3 currentNode = start;

//         for (int i = 0; i < maxIterations; i++)
//         {
//             Vector3 randomNode = Random.insideUnitSphere * 5 + goal; // Generate a random node around the goal
//             Vector3 direction = (randomNode - currentNode).normalized;
//             Vector3 newNode = currentNode + direction * stepSize;

//             if (!Physics.Linecast(currentNode, newNode)) // Check for collisions
//             {
//                 path.Add(newNode);
//                 currentNode = newNode;
//                 Debug.Log("New node added at: " + newNode);
//                 if (Vector3.Distance(newNode, goal) < stepSize)
//                 {
//                     Debug.Log("Goal reached.");
//                     break;
//                 }
//             }
//         }

//         return path;
//     }
// }


using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RRTPathPlanner : MonoBehaviour
{
    public Transform target; // Target to reach
    public List<Transform> joints; // List of joints in the robot arm
    public Transform endEffector; // End effector transform
    public float stepSize = 0.05f; // Step size of the RRT
    public int maxIterations = 1000; // Maximum number of iterations for path planning
    public float ikTolerance = 0.01f; // How close end effector needs to be to target
    public int maxIKIterations = 10; // Maximum FABRIK iterations per movement

    private Vector3[] jointPositions;
    private float[] jointLengths;
    private Vector3 rootPos;

    void Start()
    {
        if (joints == null || joints.Count == 0 || endEffector == null)
        {
            Debug.LogError("Joints or end effector not assigned!");
            return;
        }

        InitializeArrays();
        rootPos = joints[0].position;
        StartCoroutine(PlanPathToTarget());
    }

    void InitializeArrays()
    {
        int totalPoints = joints.Count + 1; // +1 for end effector
        jointPositions = new Vector3[totalPoints];
        jointLengths = new float[totalPoints - 1];

        // Calculate initial positions and lengths
        for (int i = 0; i < joints.Count; i++)
        {
            jointPositions[i] = joints[i].position;
            if (i < joints.Count - 1)
            {
                jointLengths[i] = Vector3.Distance(joints[i].position, joints[i + 1].position);
            }
        }
        
        // Add end effector
        jointPositions[totalPoints - 1] = endEffector.position;
        jointLengths[totalPoints - 2] = Vector3.Distance(joints[joints.Count - 1].position, endEffector.position);
    }

    IEnumerator PlanPathToTarget()
    {
        List<Vector3> path = RRT(endEffector.position, target.position);
        foreach (Vector3 targetPos in path)
        {
            yield return StartCoroutine(MoveToTarget(targetPos));
            yield return new WaitForSeconds(0.1f);
        }
    }

    IEnumerator MoveToTarget(Vector3 targetPosition)
    {
        int iterations = 0;
        while (iterations < maxIKIterations && 
               Vector3.Distance(endEffector.position, targetPosition) > ikTolerance)
        {
            // Update current positions
            for (int i = 0; i < joints.Count; i++)
            {
                jointPositions[i] = joints[i].position;
            }
            jointPositions[jointPositions.Length - 1] = endEffector.position;

            // FABRIK forward pass
            jointPositions[jointPositions.Length - 1] = targetPosition;
            for (int i = jointPositions.Length - 2; i >= 0; i--)
            {
                Vector3 direction = (jointPositions[i] - jointPositions[i + 1]).normalized;
                jointPositions[i] = jointPositions[i + 1] + direction * jointLengths[i];
            }

            // FABRIK backward pass
            jointPositions[0] = rootPos; // Lock base position
            for (int i = 1; i < jointPositions.Length; i++)
            {
                Vector3 direction = (jointPositions[i] - jointPositions[i - 1]).normalized;
                jointPositions[i] = jointPositions[i - 1] + direction * jointLengths[i - 1];
            }

            // Apply rotations to joints
            for (int i = 0; i < joints.Count; i++)
            {
                Vector3 direction;
                if (i < joints.Count - 1)
                {
                    direction = jointPositions[i + 1] - jointPositions[i];
                }
                else
                {
                    direction = jointPositions[jointPositions.Length - 1] - jointPositions[i];
                }

                // Only update rotation if we have a valid direction
                if (direction.magnitude > 0.001f)
                {
                    Quaternion targetRotation = Quaternion.LookRotation(direction);
                    joints[i].rotation = Quaternion.Lerp(joints[i].rotation, targetRotation, 0.5f);
                }
            }

            iterations++;
            yield return new WaitForSeconds(0.01f);
        }
    }

    List<Vector3> RRT(Vector3 start, Vector3 goal)
    {
        List<Vector3> path = new List<Vector3>();
        List<Vector3> nodes = new List<Vector3>();
        List<int> parents = new List<int>();

        nodes.Add(start);
        parents.Add(0);

        for (int i = 0; i < maxIterations; i++)
        {
            // Bias towards goal occasionally
            Vector3 randomPoint = (Random.value < 0.1f) ? 
                goal : 
                start + Random.insideUnitSphere * Vector3.Distance(start, goal);

            // Find nearest node
            int nearestIndex = 0;
            float nearestDistance = float.MaxValue;
            for (int j = 0; j < nodes.Count; j++)
            {
                float distance = Vector3.Distance(nodes[j], randomPoint);
                if (distance < nearestDistance)
                {
                    nearestDistance = distance;
                    nearestIndex = j;
                }
            }

            // Create new node in direction of random point
            Vector3 direction = (randomPoint - nodes[nearestIndex]).normalized;
            Vector3 newNode = nodes[nearestIndex] + direction * stepSize;

            // Check if new node is valid
            if (!Physics.Linecast(nodes[nearestIndex], newNode))
            {
                nodes.Add(newNode);
                parents.Add(nearestIndex);

                // Check if we can reach goal
                if (Vector3.Distance(newNode, goal) < stepSize)
                {
                    // Construct path from goal to start
                    int currentIndex = nodes.Count - 1;
                    while (currentIndex != 0)
                    {
                        path.Insert(0, nodes[currentIndex]);
                        currentIndex = parents[currentIndex];
                    }
                    path.Insert(0, start);
                    Debug.Log("Path found with " + path.Count + " nodes");
                    return path;
                }
            }
        }

        Debug.LogWarning("Failed to find path in " + maxIterations + " iterations");
        return path;
    }

    void OnDrawGizmos()
    {
        // Visualize the joints and connections
        if (joints != null && joints.Count > 0)
        {
            Gizmos.color = Color.blue;
            for (int i = 0; i < joints.Count - 1; i++)
            {
                if (joints[i] != null && joints[i + 1] != null)
                {
                    Gizmos.DrawLine(joints[i].position, joints[i + 1].position);
                    Gizmos.DrawWireSphere(joints[i].position, 0.03f);
                }
            }
            
            // Draw end effector
            if (endEffector != null && joints[joints.Count - 1] != null)
            {
                Gizmos.color = Color.red;
                Gizmos.DrawLine(joints[joints.Count - 1].position, endEffector.position);
                Gizmos.DrawWireSphere(endEffector.position, 0.03f);
            }
        }
    }
}
