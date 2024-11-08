using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System;

public class NiryoOneControl : MonoBehaviour
{
    public Transform[] joints; // Array of robot joints
    public Transform endEffector; // End-effector transform
    public float learningRate = 0.05f; // Learning rate for movement adjustment
    public float threshold = 0.01f; // Threshold to stop movement
    private Vector3 targetPosition; // Target position in 3D space
    private bool positionReceived = false;

    private UdpClient udpClient;
    private Thread receiveThread;
    private int port = 65402; // Port to match Python sender

    void Start()
    {
        udpClient = new UdpClient(port);
        receiveThread = new Thread(new ThreadStart(ReceiveData));
        receiveThread.IsBackground = true;
        receiveThread.Start();
    }

    void Update()
    {
        // Move the end-effector only if a position is received
        if (positionReceived && Vector3.Distance(endEffector.position, targetPosition) > threshold)
        {
            MoveToTarget();
        }
    }

    private void ReceiveData()
    {
        IPEndPoint endPoint = new IPEndPoint(IPAddress.Any, port);
        while (true)
        {
            try
            {
                byte[] data = udpClient.Receive(ref endPoint);

                if (data.Length == 12) // Expecting 3 floats (4 bytes each)
                {
                    float x = BitConverter.ToSingle(data, 0);
                    float y = BitConverter.ToSingle(data, 4);
                    float z = BitConverter.ToSingle(data, 8);

                    targetPosition = new Vector3(x, y, z);
                    positionReceived = true;
                    Debug.Log($"Received target position: X={x}, Y={y}, Z={z}");
                }
            }
            catch (Exception ex)
            {
                Debug.Log("Error receiving data: " + ex.Message);
            }
        }
    }

    private void MoveToTarget()
    {
        for (int i = joints.Length - 1; i >= 0; i--)
        {
            Vector3 jointPos = joints[i].position;
            Vector3 endEffectorPos = endEffector.position;
            Vector3 toTarget = targetPosition - endEffectorPos;
            Vector3 toEndEffector = endEffectorPos - jointPos;

            // Calculate gradient (cross-product magnitude as an approximation for gradient descent step)
            float gradient = Vector3.Cross(toEndEffector, toTarget).magnitude;
            joints[i].Rotate(Vector3.up, -learningRate * gradient);
        }
    }

    private void OnApplicationQuit()
    {
        receiveThread.Abort();
        udpClient.Close();
    }
}
