using UnityEngine;
using UnityEngine.XR;

public class VisualizationManager : MonoBehaviour
{
    public GameObject cameraFeedObject;
    public GameObject pointCloudObject;
    bool lastAPressed = false;
    bool lastBPressed = false;


    InputDevice rightController;

    void Start()
    {
        SetCameraMode();
        InitializeController();
    }

    void InitializeController()
    {
        var devices = new System.Collections.Generic.List<InputDevice>();
        InputDevices.GetDevicesAtXRNode(XRNode.RightHand, devices);

        if (devices.Count > 0)
            rightController = devices[0];
    }

    void Update()
    {
        if (!rightController.isValid)
            InitializeController();

        bool aPressed = false;
        bool bPressed = false;

        // Read Meta Quest controller buttons
        rightController.TryGetFeatureValue(CommonUsages.primaryButton, out aPressed);
        rightController.TryGetFeatureValue(CommonUsages.secondaryButton, out bPressed);

        // Rising-edge detection
        if (aPressed && !lastAPressed)
            SetCameraMode();

        if (bPressed && !lastBPressed)
            SetMapMode();

        lastAPressed = aPressed;
        lastBPressed = bPressed;

        // --- OPTIONAL keyboard fallback ---
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