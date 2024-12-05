using UnityEngine;

public class TargetWorldPosition : MonoBehaviour
{
    public Transform target; // Assign your target object in the Inspector

    void Update()
    {
        if (target != null)
        {
            // Get the world position of the target
            Vector3 worldPosition = target.position;

            // Log the world coordinates to the console
            Debug.Log($"[TargetWorldPosition] Target World Coordinates: {worldPosition}");
        }
        else
        {
            Debug.LogWarning("[TargetWorldPosition] Target is not assigned!");
        }
    }
}
