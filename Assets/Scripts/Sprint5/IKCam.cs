using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System.Text;

public class NiryoOneIKCAM : MonoBehaviour
{
    public Transform endEffector; // Reference to the end effector of the robot
    public ArticulationBody[] joints; // Array for all six joint articulations
    public float threshold = 0.01f; // Distance threshold to consider end effector at the target
    public int maxIterations = 10; // Maximum number of IK iterations per frame
    public float stepSize = 15f; // Degree change per iteration for each joint

    private Vector3 targetPosition; // The current target position from Python
    private UdpClient udpClient;
    private IPEndPoint endPoint;

    void Start()
    {
        // Initialize the UDP client to receive position data from Python
        udpClient = new UdpClient(65402); // Adjust the port number as needed
        endPoint = new IPEndPoint(IPAddress.Any, 65402);
        Debug.Log("UDP Server started... Waiting for data.");
    }

    void Update()
    {
        try
        {
            if (udpClient.Available > 0)
            {
                // Receive data from Python
                byte[] data = udpClient.Receive(ref endPoint);
                string receivedData = Encoding.ASCII.GetString(data).TrimEnd('\0');
                Debug.Log("Received Data: " + receivedData);

                // Parse the position data (expecting the format [x, y, z])
                string[] positionData = receivedData.Trim(new char[] { '[', ']' }).Split(',');

                if (positionData.Length == 3 &&
                    float.TryParse(positionData[0], out float x) &&
                    float.TryParse(positionData[1], out float y) &&
                    float.TryParse(positionData[2], out float z))
                {
                    targetPosition = new Vector3(x, y, z); // Update the target position
                    Debug.Log("Parsed Target Position: " + targetPosition);

                    // Move the arm to the new target position
                    PerformInverseKinematics();
                }
            }
        }
        catch (System.Exception ex)
        {
            Debug.Log("Error receiving data: " + ex.Message);
        }
    }

    private void PerformInverseKinematics()
    {
        float initialDistance = Vector3.Distance(endEffector.position, targetPosition);
        if (initialDistance < threshold)
        {
            Debug.Log("Target already within threshold, no movement needed.");
            return;
        }

        for (int iteration = 0; iteration < maxIterations; iteration++)
        {
            bool closeEnough = false;

            for (int i = joints.Length - 1; i >= 0; i--)
            {
                ArticulationBody joint = joints[i];
                Vector3 toEndEffector = endEffector.position - joint.transform.position;
                Vector3 toTarget = targetPosition - joint.transform.position;

                float angle = Vector3.SignedAngle(toEndEffector, toTarget, joint.transform.up);
                angle = Mathf.Clamp(angle, -stepSize, stepSize);

                Quaternion rotation = Quaternion.AngleAxis(angle, joint.transform.up);
                joint.transform.rotation = rotation * joint.transform.rotation;

                var drive = joint.xDrive;
                drive.target += angle;
                joint.xDrive = drive;

                if (Vector3.Distance(endEffector.position, targetPosition) < threshold)
                {
                    closeEnough = true;
                    Debug.Log("End effector reached target within threshold.");
                    break;
                }
            }

            if (closeEnough)
                break;
        }

        Debug.Log("Final Distance to Target: " + Vector3.Distance(endEffector.position, targetPosition));
    }

    private void OnApplicationQuit()
    {
        udpClient.Close();
        Debug.Log("UDP Server stopped.");
    }
}
