using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System.Text;

public class NiryoOneIKCAM : MonoBehaviour
{
    public Transform endEffector; // Reference to the end effector of the robot
    public ArticulationBody[] joints; // Array for all six joint articulations
    public float threshold = 0.1f; // Distance threshold to consider end effector at the target
    public int maxIterations = 10; // Maximum number of IK iterations per frame
    public float stepSize = 5f; // Degree change per iteration for each joint

    private Vector3 targetPosition; // The current target position from Python
    private TcpListener listener;
    private TcpClient client;
    private NetworkStream stream;
    private byte[] data;

    void Start()
    {
        // Initialize the socket listener to receive position data from Python
        listener = new TcpListener(IPAddress.Parse("127.0.0.1"), 12345); // Adjust the port number as needed
        listener.Start();
        Debug.Log("Server started... Waiting for data.");
    }

    void Update()
    {
        if (listener.Pending())
        {
            // Accept incoming connection
            client = listener.AcceptTcpClient();
            stream = client.GetStream();
            data = new byte[client.ReceiveBufferSize];
            stream.Read(data, 0, data.Length);
            string receivedData = Encoding.ASCII.GetString(data);
            Debug.Log("Received Data: " + receivedData);

            // Parse the position data (expecting the format [x, y, z])
            string[] positionData = receivedData.Trim(new char[] { '[', ']' }).Split(',');
            float x = float.Parse(positionData[0]);
            float y = float.Parse(positionData[1]);
            float z = float.Parse(positionData[2]);

            targetPosition = new Vector3(x, y, z); // Update the target position

            // Move the arm to the new target position
            PerformInverseKinematics();
        }
    }

    private void PerformInverseKinematics()
    {
        // If the end effector is already close enough to the target, stop the calculation
        if (Vector3.Distance(endEffector.position, targetPosition) < threshold)
            return;

        // Perform the IK iterations to move the arm towards the target
        for (int iteration = 0; iteration < maxIterations; iteration++)
        {
            for (int i = joints.Length - 1; i >= 0; i--)
            {
                ArticulationBody joint = joints[i];

                // Get direction vectors
                Vector3 toEndEffector = endEffector.position - joint.transform.position;
                Vector3 toTarget = targetPosition - joint.transform.position;

                // Calculate the rotation angle to bring the end effector closer to the target
                float angle = Vector3.SignedAngle(toEndEffector, toTarget, joint.transform.up);

                // Limit the angle to stepSize to ensure gradual movement
                angle = Mathf.Clamp(angle, -stepSize, stepSize);

                // Apply rotation to the joint
                Quaternion rotation = Quaternion.AngleAxis(angle, joint.transform.up);
                joint.transform.rotation = rotation * joint.transform.rotation;

                // Update the articulation body joint angle
                var drive = joint.xDrive;
                drive.target = angle;
                joint.xDrive = drive;

                // Check if the end effector is close enough to the target
                if (Vector3.Distance(endEffector.position, targetPosition) < threshold)
                    return;
            }
        }
    }

    private void OnApplicationQuit()
    {
        // Clean up and close the connection when the application is quit
        if (client != null)
        {
            stream.Close();
            client.Close();
        }
        listener.Stop();
    }
}
