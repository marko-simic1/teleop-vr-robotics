using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

public class VirtualLaserScanPublisher : MonoBehaviour
{
    [Header("ROS")]
    public string scanTopic = "/scan";
    public string frameId = "base_link";

    [Header("Laser settings")]
    public int rayCount = 181;            
    public float fovDeg = 180f;            
    public float rangeMin = 0.05f;
    public float rangeMax = 10.0f;
    public float publishHz = 10f;

    [Header("Raycast")]
    public Transform laserOrigin;          
    public LayerMask obstacleLayers = ~0;   

    ROSConnection ros;
    float[] ranges;

    float angleMin;
    float angleMax;
    float angleInc;
    float nextPubTime;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<LaserScanMsg>(scanTopic);

        if (laserOrigin == null)
            laserOrigin = transform;

        ranges = new float[rayCount];

        angleMin = -Mathf.Deg2Rad * (fovDeg * 0.5f);
        angleMax =  Mathf.Deg2Rad * (fovDeg * 0.5f);
        angleInc = (rayCount > 1) ? (angleMax - angleMin) / (rayCount - 1) : 0f;

        Debug.Log($"[VirtualLaserScanPublisher] Publishing {scanTopic} frame_id={frameId} rays={rayCount} fov={fovDeg}deg");
    }

    void Update()
    {
        if (publishHz <= 0f) return;
        if (Time.time < nextPubTime) return;
        nextPubTime = Time.time + (1f / publishHz);

        PublishScan();
    }

    void PublishScan()
    {
        Vector3 origin = laserOrigin.position;

        for (int i = 0; i < rayCount; i++)
        {
            float a = angleMin + i * angleInc; // rad
            Vector3 dir = Quaternion.AngleAxis(a * Mathf.Rad2Deg, Vector3.up) * laserOrigin.forward;

            if (Physics.Raycast(origin, dir, out RaycastHit hit, rangeMax, obstacleLayers))
            {
                float d = hit.distance;
                ranges[i] = Mathf.Max(rangeMin, d);

                Debug.DrawRay(origin, dir * d, Color.red, 0.11f);
            }
            else
            {
                ranges[i] = float.PositiveInfinity; // standardno za "nema prepreke"
            }
        }

        var header = new HeaderMsg();
        header.frame_id = frameId;
    
        var msg = new LaserScanMsg
        {
            header = header,
            angle_min = angleMin,
            angle_max = angleMax,
            angle_increment = angleInc,
            time_increment = 0f,
            scan_time = (publishHz > 0f) ? (1f / publishHz) : 0f,
            range_min = rangeMin,
            range_max = rangeMax,
            ranges = ranges,
            intensities = Array.Empty<float>()
        };

        ros.Publish(scanTopic, msg);
    }
}
