using UnityEngine;
using UnityEngine.UI;
using Meta.XR;
using OpenCvSharp;
using OpenCvSharp.Aruco;

public class CameraToMat : MonoBehaviour
{
    [SerializeField] private PassthroughCameraAccess passthroughCameraAccess;
    [SerializeField] private RawImage rawImage;


    private bool textureAssigned;

    void Update()
    {
        if (passthroughCameraAccess == null || rawImage == null) return;

        if (!passthroughCameraAccess.IsPlaying) return;

        if (!textureAssigned)
        {
            rawImage.texture = passthroughCameraAccess.GetTexture();
            textureAssigned = true;

            Debug.Log("camera start");
        }

    }
}
