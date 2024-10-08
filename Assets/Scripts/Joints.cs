using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Joints : MonoBehaviour
{
    public Joints m_child;

    public Joints GetChild()
    {
        return m_child;
    }
    
    public void Rotate(float _angle)
    {
        transform.Rotate(Vector3.up * _angle);
    }
}
