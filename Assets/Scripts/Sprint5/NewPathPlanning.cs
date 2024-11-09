using UnityEngine;
using System.Collections.Generic;

public class NewPathPlanning : MonoBehaviour
{
    public Transform endEffector;
    public Transform robotBase;
    public GameObject targetObject;
    public List<GameObject> obstacles;
    public float stepSize = 0.05f;
    public float reachThreshold = 0.1f;

    private Vector3 start;
    private Vector3 targetPosition;
    private bool targetReached = false;

    void Start()
    {
        start = endEffector.position;
        FindTargetPosition();
        FindObstaclePositions();
    }

    void Update()
    {
        if (!targetReached)
        {
            FindTargetPosition();
            List<Vector3> path = PlanPath(start, targetPosition);

            if (path != null)
            {
                FollowPath(path);
            }
            else
            {
                Debug.LogError("Path not found!");
            }
        }
    }

    void FindTargetPosition()
    {
        if (targetObject != null)
        {
            targetPosition = targetObject.transform.position;
            Debug.Log($"Target position detected at: {targetPosition}");
            print($"Target position detected at: "+ targetPosition);
        }
        else
        {
            Debug.LogError("No target object found in the scene.");
        }
    }

    void FindObstaclePositions()
    {
        foreach (var obstacle in obstacles)
        {
            Debug.Log($"Obstacle detected at: {obstacle.transform.position}");
            print($"Obstacle detected at: " + obstacle.transform.position);

        }
    }

    List<Vector3> PlanPath(Vector3 start, Vector3 target)
    {
        List<Vector3> path = new List<Vector3> { start };
        Vector3 currentPosition = start;

        while (Vector3.Distance(currentPosition, target) > reachThreshold)
        {
            Vector3 nextStep = Vector3.MoveTowards(currentPosition, target, stepSize);

            if (!IsPathBlocked(currentPosition, nextStep))
            {
                path.Add(nextStep);
                currentPosition = nextStep;
                Debug.Log($"Moving to: {nextStep}");
            }
            else
            {
                Debug.LogWarning("Path blocked, trying to find alternative route.");
                return null; // Pathfinding failed; use another method if desired
            }
        }

        path.Add(target);
        Debug.Log("Path planning complete.");
        return path;
    }

    bool IsPathBlocked(Vector3 start, Vector3 end)
    {
        foreach (var obstacle in obstacles)
        {
            Collider obstacleCollider = obstacle.GetComponent<Collider>();
            if (obstacleCollider != null && obstacleCollider.bounds.IntersectRay(new Ray(start, end - start)))
            {
                Debug.Log($"Path blocked by obstacle at: {obstacle.transform.position}");
                print($"Path blocked by obstacle at: "+ obstacle.transform.position);
                return true;
            }
        }
        return false;
    }

    void FollowPath(List<Vector3> path)
    {
        foreach (Vector3 point in path)
        {
            endEffector.position = point;
            Debug.Log($"End effector moving to: {point}");
            print($"End effector moving to: " +endEffector.position);

            if (Vector3.Distance(endEffector.position, targetPosition) <= reachThreshold)
            {
                Debug.Log("Target reached.");
                targetReached = true;
                break;
            }
        }
    }
}
