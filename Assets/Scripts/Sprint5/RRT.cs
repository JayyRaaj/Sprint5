using System.Collections.Generic;
using UnityEngine;

public class RRTWithFABRIKController : MonoBehaviour
{
    public Transform[] joints;
    public Transform target;
    public LayerMask obstacleLayer;
    public float stepSize = 0.5f;
    public int maxRRTIterations = 1000;
    public float obstacleRadius = 0.2f;
    public float tolerance = 0.05f;

    private RRTPathfinder pathfinder;
    private FABRIK fabrik;
    private List<Vector3> waypoints;
    private int currentWaypointIndex = 0;

void Start()
{
    Debug.Log("Starting RRTWithFABRIKController initialization...");

    if (joints == null || joints.Length == 0)
    {
        Debug.LogError("Joints array is not assigned or empty.");
        return;
    }

    if (target == null)
    {
        Debug.LogError("Target is not assigned.");
        return;
    }

    Debug.Log("Initializing RRT pathfinder...");
    pathfinder = new RRTPathfinder(
        joints[0].position,
        target.position,
        stepSize,
        maxRRTIterations,
        obstacleLayer,
        obstacleRadius
    );

    waypoints = pathfinder.FindPath();
    Debug.Log("Pathfinder completed. Waypoints count: " + waypoints?.Count);

    if (waypoints == null || waypoints.Count == 0)
    {
        Debug.LogError("No path found. Check obstacle layer and pathfinding parameters.");
        return;
    }

    fabrik = GetComponent<FABRIK>();
    if (fabrik == null)
    {
        Debug.LogError("FABRIK component not found on GameObject.");
        return;
    }

    fabrik.joints = joints;
    Debug.Log("FABRIK component initialized successfully.");
}


void Update()
{
    // Ensure waypoints and fabrik are initialized before proceeding
    if (waypoints == null || waypoints.Count == 0)
    {
        Debug.LogWarning("Waypoints not initialized. Skipping Update.");
        return;
    }

    if (fabrik == null)
    {
        Debug.LogWarning("FABRIK component not initialized. Skipping Update.");
        return;
    }

    // Move towards each waypoint using FABRIK
    if (currentWaypointIndex < waypoints.Count)
    {
        Vector3 waypoint = waypoints[currentWaypointIndex];
        fabrik.MoveTowardsTarget(waypoint);

        if (Vector3.Distance(joints[joints.Length - 1].position, waypoint) < tolerance)
        {
            currentWaypointIndex++;
        }
    }
    else
    {
        fabrik.MoveTowardsTarget(target.position);
    }
}

}
