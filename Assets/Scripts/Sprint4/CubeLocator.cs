using UnityEngine;
using Unity.Mathematics;

public class CubeLocator : MonoBehaviour
{
    public CameraIntrinsics cameraIntrinsics;
    public Camera rgbdCamera;

    private RenderTexture rgbRenderTexture;
    private Texture2D rgbImage;
    private float alpha_u, alpha_v, u_0, v_0;
    private Vector3 lastKnownPosition = Vector3.zero; // Store last known position

    void Start()
    {
        rgbRenderTexture = new RenderTexture(rgbdCamera.pixelWidth, rgbdCamera.pixelHeight, 24);
        rgbdCamera.targetTexture = rgbRenderTexture;
        rgbImage = new Texture2D(rgbRenderTexture.width, rgbRenderTexture.height, TextureFormat.RGB24, false);

        float3x3 intrinsicMatrix = cameraIntrinsics.GetIntrinsic(rgbdCamera);
        alpha_u = intrinsicMatrix.c0.x;
        alpha_v = intrinsicMatrix.c1.y;
        u_0 = intrinsicMatrix.c0.z;
        v_0 = intrinsicMatrix.c1.z;
    }

    void Update()
    {
        LocateObjectIn3D();
    }

    void LocateObjectIn3D()
    {
        CaptureRGBImage();
        Vector2 pixelCoordinates = FindRedCubePixel(rgbImage);

        if (pixelCoordinates != Vector2.zero)
        {
            float depth = GetDepthAtPixel(pixelCoordinates);
            Vector3 objectWorldPosition = PixelToWorld(pixelCoordinates, depth);

            // Only log position if it has changed
            if (objectWorldPosition != lastKnownPosition)
            {
                // Debug.Log("Cube world position: " + objectWorldPosition.ToString("F6"));
                lastKnownPosition = objectWorldPosition;
            }
        }
        else
        {
            // Debug.Log("Cube not detected in the image.");
        }
    }

    void CaptureRGBImage()
    {
        rgbdCamera.targetTexture = rgbRenderTexture;
        rgbdCamera.Render();
        RenderTexture.active = rgbRenderTexture;
        rgbImage.ReadPixels(new Rect(0, 0, rgbRenderTexture.width, rgbRenderTexture.height), 0, 0);
        rgbImage.Apply();
        RenderTexture.active = null;
    }

    Vector2 FindRedCubePixel(Texture2D rgbImage)
    {
        for (int y = 0; y < rgbImage.height; y++)
        {
            for (int x = 0; x < rgbImage.width; x++)
            {
                Color color = rgbImage.GetPixel(x, y);
                if (color.r > 0.8f && color.g < 0.2f && color.b < 0.2f)
                {
                    return new Vector2(x, y);
                }
            }
        }
        return Vector2.zero;
    }

    float GetDepthAtPixel(Vector2 pixelCoordinates)
    {
        rgbdCamera.targetTexture = null;
        rgbdCamera.depthTextureMode = DepthTextureMode.Depth;
        rgbdCamera.Render();
        RenderTexture.active = rgbRenderTexture;

        Texture2D depthTex = new Texture2D(rgbRenderTexture.width, rgbRenderTexture.height, TextureFormat.RFloat, false);
        depthTex.ReadPixels(new Rect((int)pixelCoordinates.x, (int)pixelCoordinates.y, 1, 1), 0, 0);
        depthTex.Apply();

        Color depthColor = depthTex.GetPixel(0, 0);
        RenderTexture.active = null;

        return depthColor.r * 10.0f;
    }

    Vector3 PixelToWorld(Vector2 pixelCoordinates, float depth)
    {
        float X = (pixelCoordinates.x - u_0) * depth / alpha_u;
        float Y = (pixelCoordinates.y - v_0) * depth / alpha_v;
        float Z = depth;

        Vector3 localPosition = new Vector3(X, Y, Z);
        return rgbdCamera.transform.TransformPoint(localPosition);
    }
}
