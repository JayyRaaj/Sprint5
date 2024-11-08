using UnityEngine;

public class BaseRotation : MonoBehaviour
{
    public Transform target; // The target to rotate towards
    public float rotationSpeed = 5.0f; // Speed of rotation

    void Update()
    {
        if (target != null)
        {
            // Calculate the direction from the base to the target
            Vector3 direction = target.position - transform.position;

            // Calculate the angle to the target
            float angleToTarget = Mathf.Atan2(direction.z, direction.x) * Mathf.Rad2Deg;

            // Log the angle to the console
            Debug.Log("Angle to Target: " + angleToTarget);

            // Create a rotation that looks in the direction of the target
            Quaternion lookRotation = Quaternion.LookRotation(direction);

            // Smoothly rotate the base towards the target
            transform.rotation = Quaternion.Slerp(transform.rotation, lookRotation, rotationSpeed * Time.deltaTime);
        }
    }
}
