using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class OakImageSubscriber : MonoBehaviour
{
    public string topicName = "/oak/rgb/image_raw/compressed";

    private ROSConnection ros;
    private Texture2D texture;
    private Renderer quadRenderer;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<ImageMsg>(topicName, ImageCallback);

        quadRenderer = GetComponent<Renderer>();
    }

    void ImageCallback(ImageMsg msg)
    {
        if (texture == null)
        {
            texture = new Texture2D(
                (int)msg.width,
                (int)msg.height,
                TextureFormat.RGB24,
                false
            );

            quadRenderer.material = new Material(Shader.Find("Unlit/Texture"));
            quadRenderer.material.mainTexture = texture;
        }

        texture.LoadRawTextureData(msg.data);
        texture.Apply();
    }
}
