using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using UnityEngine.Rendering;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class RosPointCloudMesh : MonoBehaviour
{
    [Header("ROS")]
    public string topicName = "/oak/points";

    [Header("Point limits")]
    public int maxAccumulatedPoints = 100000;   // hard cap (VERY important)
    public int stride = 4;                      // keep 1 out of N points

    [Header("Update throttling")]
    public float meshUpdateInterval = 0.1f;     // seconds (10 Hz)

    [Header("Depth filter (meters)")]
    public float minDepth = 0.3f;
    public float maxDepth = 5.0f;

    Mesh mesh;

    // Accumulated buffers
    List<Vector3> accumulatedVertices = new List<Vector3>();
    List<Color> accumulatedColors = new List<Color>();

    // Reused index buffer
    int[] indices = Array.Empty<int>();

    ROSConnection ros;

    bool newDataReceived = false;
    float lastMeshUpdateTime = 0f;

    void Start()
    {
        mesh = new Mesh();
        mesh.indexFormat = IndexFormat.UInt32;
        GetComponent<MeshFilter>().mesh = mesh;

        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<PointCloud2Msg>(topicName, PointCloudCallback);
    }

    void Update()
    {
        // Throttle mesh updates (VERY important for stability)
        if (!newDataReceived)
            return;

        if (Time.time - lastMeshUpdateTime < meshUpdateInterval)
            return;

        lastMeshUpdateTime = Time.time;
        newDataReceived = false;

        int count = accumulatedVertices.Count;
        if (count == 0)
            return;

        // Ensure index buffer is large enough
        if (indices.Length < count)
        {
            indices = new int[count];
            for (int i = 0; i < count; i++)
                indices[i] = i;
        }

        mesh.Clear();
        mesh.SetVertices(accumulatedVertices);
        mesh.SetColors(accumulatedColors);
        mesh.SetIndices(indices, 0, count, MeshTopology.Points, 0);
    }

    void PointCloudCallback(PointCloud2Msg msg)
    {
        int step = (int)msg.point_step;
        int totalPoints = msg.data.Length / step;

        for (int i = 0; i < totalPoints; i += stride)
        {
            int offset = i * step;

            float x = BitConverter.ToSingle(msg.data, offset + 0);
            float y = BitConverter.ToSingle(msg.data, offset + 4);
            float z = BitConverter.ToSingle(msg.data, offset + 8);

            // Filter invalid points
            if (float.IsNaN(x) || float.IsNaN(y) || float.IsNaN(z))
                continue;
            if (z < minDepth || z > maxDepth)
                continue;

            // ROS optical → Unity (correct mapping)
            Vector3 point = new Vector3(x, -y, z);

            // RGB (offset = 16 for on-device cloud)
            float rgbFloat = BitConverter.ToSingle(msg.data, offset + 16);
            int rgbInt = BitConverter.ToInt32(BitConverter.GetBytes(rgbFloat), 0);

            byte r = (byte)((rgbInt >> 16) & 0xFF);
            byte g = (byte)((rgbInt >> 8) & 0xFF);
            byte b = (byte)(rgbInt & 0xFF);

            Color color = new Color(r / 255f, g / 255f, b / 255f, 1f);

            accumulatedVertices.Add(point);
            accumulatedColors.Add(color);
        }

        // HARD CAP — remove oldest points
        int overflow = accumulatedVertices.Count - maxAccumulatedPoints;
        if (overflow > 0)
        {
            accumulatedVertices.RemoveRange(0, overflow);
            accumulatedColors.RemoveRange(0, overflow);
        }

        newDataReceived = true;
    }

    // Optional helper you can call from UI / keybind
    public void ClearPointCloud()
    {
        accumulatedVertices.Clear();
        accumulatedColors.Clear();
        mesh.Clear();
    }
}
