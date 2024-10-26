using Unity.Mathematics;
using UnityEngine;

public class CameraIntrinsics : MonoBehaviour 
{
    public Camera Target;

    void Start()
    {
        Debug.Log(GetIntrinsic(Target));
    }   

    void Update()
    {
    } 

    public float3x3 GetIntrinsic(Camera cam)
    {
        float pixel_aspect_ratio = (float)cam.pixelWidth / (float)cam.pixelHeight;

        float alpha_u = cam.focalLength * ((float)cam.pixelWidth / cam.sensorSize.x);
        float alpha_v = cam.focalLength * pixel_aspect_ratio * ((float)cam.pixelHeight / cam.sensorSize.y);

        float u_0 = (float)cam.pixelWidth / 2;
        float v_0 = (float)cam.pixelHeight / 2;

        //IntrinsicMatrix in row major
        float3x3 camIntriMatrix = new float3x3(alpha_u,      0f, u_0,
                                                    0f, alpha_v, v_0,
                                                    0f,      0f,  1f);
        return camIntriMatrix;
    }
}