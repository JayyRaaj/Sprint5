using System.Collections;
using System.Collections.Generic;
using UnityEngine;

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

    // Start is called before the first frame update
    void Start()
    {
        start = endEffector.position;
        FindTargetPosition();
        print("Start:" + start);
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    void FindTargetPosition()
    {
        if (targetObject != null)
        {
            targetPosition = targetObject.transform.position;
            Debug.Log($"Target position detected at: {targetPosition}");
        }
        else
        {
            Debug.LogError("No target object found in the scene.");
        }
    }
}