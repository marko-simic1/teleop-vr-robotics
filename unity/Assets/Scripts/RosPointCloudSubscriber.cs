using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using UnityEngine.Rendering;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class RtabmapCloudMapRenderer : MonoBehaviour
{
    [Header("ROS")]
    public string topicName = "/cloud_map";

    [Header("Limits")]
    public int maxPoints = 200000;
    public int stride = 2;

    [Header("Rendering")]
    public float pointSize = 3.0f;
    public float updateInterval = 0.5f;

    Mesh mesh;
    Mesh cachedMesh;                     // CACHE
    Material material;
    ROSConnection ros;

    List<Vector3> vertices = new List<Vector3>();
    List<Color> colors = new List<Color>();
    int[] indices = Array.Empty<int>();

    bool newData = false;
    float lastUpdate = 0f;
    bool initialized = false;

    //  Called every time object becomes active
    void OnEnable()
    {
        // Debug.Log("RtabmapCloudMapRenderer OnEnable CALLED");

        if (!initialized)
        {
            mesh = new Mesh();
            mesh.indexFormat = IndexFormat.UInt32;
            GetComponent<MeshFilter>().mesh = mesh;

            material = GetComponent<MeshRenderer>().material;
            material.SetFloat("_PointSize", pointSize);

            initialized = true;
        }

        //  Restore cached cloud instantly
        if (cachedMesh != null)
        {
            mesh.Clear();
            mesh.vertices = cachedMesh.vertices;
            mesh.colors = cachedMesh.colors;
            mesh.SetIndices(
                cachedMesh.GetIndices(0),
                MeshTopology.Points,
                0
            );
        }

        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<PointCloud2Msg>(topicName, CloudCallback);

        Debug.Log("Subscribed to /cloud_map");
    }

    //  Called every time object becomes inactive
    void OnDisable()
    {
        if (ros != null)
        {
            ros.Unsubscribe(topicName);
            Debug.Log("Unsubscribed from /cloud_map");
        }

        newData = false;
    }

    void Update()
    {
        if (material != null)
            material.SetFloat("_PointSize", pointSize);

        if (!newData)
            return;

        if (Time.time - lastUpdate < updateInterval)
            return;

        lastUpdate = Time.time;
        newData = false;

        int count = vertices.Count;
        if (count == 0)
            return;

        if (indices.Length < count)
        {
            indices = new int[count];
            for (int i = 0; i < count; i++)
                indices[i] = i;
        }

        mesh.Clear();
        mesh.SetVertices(vertices);
        mesh.SetColors(colors);
        mesh.SetIndices(indices, 0, count, MeshTopology.Points, 0);

        //  Cache the mesh after building
        CacheCurrentMesh();
    }

    void CloudCallback(PointCloud2Msg msg)
    {
        vertices.Clear();
        colors.Clear();

        int step = (int)msg.point_step;
        int total = msg.data.Length / step;

        for (int i = 0; i < total && vertices.Count < maxPoints; i += stride)
        {
            int offset = i * step;

            float x = BitConverter.ToSingle(msg.data, offset + 0);
            float y = BitConverter.ToSingle(msg.data, offset + 4);
            float z = BitConverter.ToSingle(msg.data, offset + 8);

            if (float.IsNaN(x) || float.IsNaN(y) || float.IsNaN(z))
                continue;

            // ROS → Unity
            vertices.Add(new Vector3(x, z, -y));

            Color color = Color.white;

            if (step >= 16)
            {
                float rgbFloat = BitConverter.ToSingle(msg.data, offset + 16);
                int rgb = BitConverter.ToInt32(BitConverter.GetBytes(rgbFloat), 0);

                byte r = (byte)((rgb >> 16) & 0xFF);
                byte g = (byte)((rgb >> 8) & 0xFF);
                byte b = (byte)(rgb & 0xFF);

                color = new Color(r / 255f, g / 255f, b / 255f, 1f);
            }

            colors.Add(color);
        }

        newData = true;
    }

    //  Save a snapshot of the current mesh
    void CacheCurrentMesh()
    {
        if (cachedMesh == null)
            cachedMesh = new Mesh();

        cachedMesh.Clear();
        cachedMesh.vertices = mesh.vertices;
        cachedMesh.colors = mesh.colors;
        cachedMesh.SetIndices(
            mesh.GetIndices(0),
            MeshTopology.Points,
            0
        );
    }

    public void ClearMap()
    {
        vertices.Clear();
        colors.Clear();

        if (mesh != null)
            mesh.Clear();

        if (cachedMesh != null)
            cachedMesh.Clear();
    }
}
