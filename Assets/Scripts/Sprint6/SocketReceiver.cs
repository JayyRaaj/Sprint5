using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Collections.Concurrent;
using UnityEngine;

public class RobotArmController : MonoBehaviour
{
    public ArticulationBody joint1, joint2, joint3, joint4, joint5, joint6;
    private Thread receivingThread;
    private TcpListener tcpListener;
    private bool isRunning = true;
    private ConcurrentQueue<string> commandQueue = new ConcurrentQueue<string>();
    private bool isIncrementing = true;
    private string lastCommand = "";
    public ArticulationBody gripperJointRight, gripperJointLeft;


    [Range(-175f, 175f)] public float joint1Limit = 0f;
    [Range(-110f, 36.68f)] public float joint2Limit = 0f;
    [Range(-80.07f, 90f)] public float joint3Limit = 0f;
    [Range(-175f, 175f)] public float joint4Limit = 0f;
    [Range(-100f, 110f)] public float joint5Limit = 0f;
    [Range(-147.5f, 147.5f)] public float joint6Limit = 0f;

    void Start()
    {
        tcpListener = new TcpListener(IPAddress.Parse("127.0.0.1"), 65432);
        tcpListener.Start();
        receivingThread = new Thread(ListenForCommands) { IsBackground = true };
        receivingThread.Start();
    }

    private void ListenForCommands()
    {
        try
        {
            using (TcpClient client = tcpListener.AcceptTcpClient())
            using (NetworkStream stream = client.GetStream())
            {
                byte[] buffer = new byte[1024];
                while (isRunning)
                {
                    int bytesRead = stream.Read(buffer, 0, buffer.Length);
                    if (bytesRead > 0)
                    {
                        string command = Encoding.ASCII.GetString(buffer, 0, bytesRead).Trim();
                        commandQueue.Enqueue(command);
                    }
                }
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"Socket error: {e.Message}");
        }
    }

    void Update()
    {
        while (commandQueue.TryDequeue(out string command))
        {
            ProcessCommand(command);
        }
    }

    private void CloseGripper()
    {
        SetJointTarget(gripperJointRight, 1.47f); // Upper limit
        SetJointTarget(gripperJointLeft, -1.47f); // Lower limit
    }

    private void SetJointTarget(ArticulationBody joint, float target)
    {
        var drive = joint.xDrive;
        drive.target = target;
        joint.xDrive = drive;
    }

    private void ProcessCommand(string command)
    {
        // Split the command in case it contains two hand gestures
        string[] gestures = command.Split('+');
        
        if (gestures.Length == 2)
        {
            // Handle two-handed gestures
            ProcessTwoHandedGesture(gestures[0], gestures[1]);
            return;
        }

        // Original single-hand gesture processing
        if (command == "Open_Palm" && lastCommand != "Open_Palm")
        {
            isIncrementing = !isIncrementing;
        }
        else if (command != "Open_Palm")
        {
            float increment = isIncrementing ? 1f : -1f;

            switch (command)
            {
                case "Thumb_Up":
                    IncrementJoint(joint1, increment, -175f, 175f);
                    break;
                case "Thumb_Down":
                    IncrementJoint(joint2, increment, -110f, 36.68f);
                    break;
                case "Pointing_Up":
                    IncrementJoint(joint3, increment, -80.07f, 90f);
                    break;
                case "Victory":
                    IncrementJoint(joint4, increment, -175f, 175f);
                    break;
                case "ILoveYou":
                    IncrementJoint(joint5, increment, -110f, 110f);
                    break;
                case "Closed_Fist":
                    break;
                default:
                    Debug.LogWarning($"Unknown command: {command}");
                    break;
            }
        }

        lastCommand = command;
    }

    private void ProcessTwoHandedGesture(string leftHand, string rightHand)
    {
        Debug.Log($"Two-handed gesture: Left={leftHand}, Right={rightHand}");
        
        // Example of handling specific two-handed combinations
        if (leftHand == "Closed_Fist" && rightHand == "Closed_Fist")
        {
            // Handle double fist gesture
            CloseGripper();
        }
        // Add more two-handed gesture combinations as needed
    }

    private void IncrementJoint(ArticulationBody joint, float increment, float lowerLimit, float upperLimit)
    {
        var drive = joint.xDrive;
        float newTarget = drive.target + increment;

        if (newTarget > upperLimit)
        {
            increment = -Math.Abs(increment);
            newTarget = drive.target + increment;
        }
        else if (newTarget < lowerLimit)
        {
            increment = Math.Abs(increment);
            newTarget = drive.target + increment;
        }

        drive.target = newTarget;
        joint.xDrive = drive;
    }

    private void ResetAllJoints()
    {
        foreach (var joint in new[] { joint1, joint2, joint3, joint4, joint5 , joint6})
        {
            var drive = joint.xDrive;
            drive.target = 0;
            joint.xDrive = drive;
        }
    }

    private void OnApplicationQuit()
    {
        isRunning = false;
        tcpListener?.Stop();
        receivingThread?.Abort();
    }
}
