using UnityEngine;

public class VisualizationManager : MonoBehaviour
{
    public GameObject cameraFeedObject;
    public GameObject pointCloudObject;

    void Start()
    {
        // Default mode on Play
        SetCameraMode();
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.C))
            SetCameraMode();

        if (Input.GetKeyDown(KeyCode.M))
            SetMapMode();
    }

    void SetCameraMode()
    {
        cameraFeedObject.SetActive(true);
        pointCloudObject.SetActive(false);
    }

    void SetMapMode()
    {
        cameraFeedObject.SetActive(false);
        pointCloudObject.SetActive(true);
    }
}
