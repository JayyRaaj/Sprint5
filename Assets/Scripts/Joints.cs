// using System.Collections;
// using System.Collections.Generic;
// using UnityEngine;

// public class Joint : MonoBehaviour
// {
//     public Joint m_child;

//     public Joint GetChild()
//     {
//         return m_child;
//     }
    
//     public void Rotate(float _angle)
//     {
//         transform.Rotate(Vector3.up * _angle);
//     }
// }

using UnityEngine;

public class Joint : MonoBehaviour
{
    public Joint m_child;
    public Vector3 rotationAxis = Vector3.up;
    public float minAngle = -180f;
    public float maxAngle = 180f;
    public float currentAngle = 0f;

    public Joint GetChild()
    {
        return m_child;
    }
    
    public bool Rotate(float _angle)
    {
        float newAngle = currentAngle + _angle;
        
        // Check if the new angle would exceed limits
        if (newAngle < minAngle || newAngle > maxAngle)
        {
            return false;
        }
        
        transform.Rotate(rotationAxis * _angle);
        currentAngle = newAngle;
        return true;
    }
    
    public void ResetRotation()
    {
        transform.localRotation = Quaternion.identity;
        currentAngle = 0f;
    }
    
    // Helper method to set rotation constraints
    public void SetConstraints(float min, float max)
    {
        minAngle = min;
        maxAngle = max;
    }
    
    public void SetRotationAxis(Vector3 axis)
    {
        rotationAxis = axis.normalized;
    }
}