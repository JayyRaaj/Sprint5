using System.Collections.Generic;
using UnityEngine;

public class RRTPathfinder
{
    private Vector3 start;
    private Vector3 target;
    private float stepSize;
    private int maxIterations;
    private LayerMask obstacleLayer;
    private float obstacleRadius;

    public RRTPathfinder(Vector3 start, Vector3 target, float stepSize, int maxIterations, LayerMask obstacleLayer, float obstacleRadius)
    {
        this.start = start;
        this.target = target;
        this.stepSize = stepSize;
        this.maxIterations = maxIterations;
        this.obstacleLayer = obstacleLayer;
        this.obstacleRadius = obstacleRadius;
    }

    public List<Vector3> FindPath()
    {
        List<Vector3> path = new List<Vector3> { start };
        Vector3 currentPos = start;

        for (int i = 0; i < maxIterations; i++)
        {
            Vector3 randomPoint = GetRandomPoint();
            Vector3 closestPoint = GetClosestPoint(path, randomPoint);
            Vector3 newPoint = Vector3.MoveTowards(closestPoint, randomPoint, stepSize);

            if (!Physics.CheckSphere(newPoint, obstacleRadius, obstacleLayer))
            {
                path.Add(newPoint);
                currentPos = newPoint;

                if (Vector3.Distance(newPoint, target) < stepSize)
                {
                    path.Add(target);
                    break;
                }
            }
        }

        if (path.Count == 1)
        {
            Debug.LogError("Path could not be generated. Check obstacle layer and parameters.");
        }

        return path;
    }

    private Vector3 GetRandomPoint()
    {
        return new Vector3(
            Random.Range(start.x - 10, target.x + 10),
            Random.Range(start.y - 10, target.y + 10),
            Random.Range(start.z - 10, target.z + 10)
        );
    }

    private Vector3 GetClosestPoint(List<Vector3> path, Vector3 point)
    {
        Vector3 closest = path[0];
        float minDist = Vector3.Distance(point, closest);

        foreach (Vector3 p in path)
        {
            float dist = Vector3.Distance(point, p);
            if (dist < minDist)
            {
                minDist = dist;
                closest = p;
            }
        }

        return closest;
    }
}
