using UnityEngine;
using UnityEngine.UI;
using Meta.XR;
using OpenCvSharp;
using OpenCvSharp.Aruco;
using Unity.VisualScripting;
using Unity.Mathematics;

public class PassthroughCameraViewer : MonoBehaviour
{
    [SerializeField] private PassthroughCameraAccess passthroughCameraAccess;
    [SerializeField] private TMPro.TextMeshProUGUI idsText;
    [SerializeField] private GameObject cube;

    // 마커의 크기인 18mm의 절반 사이즈 사용
    private float halfSize = 0.018f/2f;
    // 마커 크기 나타내는 변수
    private Point3f[] objectPoints;
    // 카메라 파라미터 나타내는 변수
    private double[,] cameraMatrix;
    // 렌즈 왜곡정보 변수
    private double[] distCoeffs;
    
    private Dictionary dictionary;
    private float timer;

    // 마커 탐지 기준(1f/목표 주사율f)
    private float detectionInterval = 1f/10f;

    // 카드 위치 보정 오프셋
    private Vector3 localOffset = new Vector3(0f, -0.027f, 0.02f);

    Vector3 worldPosition = Vector3.zero;
    Quaternion worldRotation = Quaternion.identity;

    // private bool textureAssigned;
    // [SerializeField] private RawImage rawImage;

    void Start()
    {
        dictionary = CvAruco.GetPredefinedDictionary(PredefinedDictionaryName.Dict4X4_50);
        // 마커 네 꼭지점을 나타내는 배열
        // 평면이기에 z는 0으로 통일
        objectPoints = new[]
        {
            new Point3f(-halfSize, halfSize, 0), 
            new Point3f(halfSize, halfSize, 0), 
            new Point3f(halfSize, -halfSize, 0), 
            new Point3f(-halfSize, -halfSize, 0)
        };

        
        // quest 카메라 내부 파라이터 받기
        var intrinsics = passthroughCameraAccess.Intrinsics;

        // Debug.Log($"CameraTest Sensor: {intrinsics.SensorResolution}");
        // Debug.Log($"CameraTest Current: {passthroughCameraAccess.CurrentResolution}");
        // Debug.Log($"CameraTest Focal: {intrinsics.FocalLength}");
        // Debug.Log($"CameraTest Principal: {intrinsics.PrincipalPoint}");

        // 메타 패스스루 비율은 1280x1280이나, 카메라로 받는 실제 비율이 다를경우 그 값을 계산해 추후 카메라 정보 보정에 사용
        float cropY = (intrinsics.SensorResolution.y - passthroughCameraAccess.CurrentResolution.y) / 2f;
        
        // openCV용 카메라 정보 행렬 만들기
        cameraMatrix = new double[,]
        {
            {intrinsics.FocalLength.x, 0, intrinsics.PrincipalPoint.x},
            {0, intrinsics.FocalLength.y, intrinsics.PrincipalPoint.y - cropY},
            {0, 0, 1}
        };

        distCoeffs = new double[] {0, 0, 0, 0};

    }


    void Update()
    {
        
        timer += Time.deltaTime;
        if (passthroughCameraAccess == null) return;

        if (!passthroughCameraAccess.IsPlaying) return;

        // ui rawimage로 카메라 직접 확인이 필요할경우 사용
        // if (!textureAssigned && rawImage != null)
        // {
        //     rawImage.texture = passthroughCameraAccess.GetTexture();
        //     textureAssigned = true;

        //     Debug.Log("camera start");
        // }
        
        // 10hz주사율로 먼저 테스트
        if (timer >= detectionInterval)
        {
            
            // 카메라로부터 컬러를 받음
            var colors = passthroughCameraAccess.GetColors();
            // 컬러32 배열형태로 변형
            Color32[] pixels = colors.ToArray();

            int width = passthroughCameraAccess.CurrentResolution.x;
            int height = passthroughCameraAccess.CurrentResolution.y;

            // 이미지를 cv가 읽을 수 있는 mat형태로 변형
            // mat은 네이티브 메모리를 사용하기때문에 using을 붙여 함수 종료시 알아서 제거도록 사용
            using Mat frame = OpenCvSharp.Unity.PixelsToMat(pixels, width, height);

            // 흑백 변환을 담을 인스턴트 생성
            using Mat gray = new Mat();

            // Debug.Log($"channels = {frame.Channels()}");

            // 흑백으로 변경
            Cv2.CvtColor(frame, gray, ColorConversionCodes.BGR2GRAY);

            // 검출한 정보 담기용 변수
            Point2f[][] corners;
            int[] ids;
            Point2f[][] rejected;

            DetectorParameters parameters = new DetectorParameters();

            // 마커 감지
            CvAruco.DetectMarkers(gray, dictionary, out corners, out ids, parameters, out rejected);

            // 마커 회전 정보
            double[] rvec = null;
            // 마커 위치
            double[] tvec = null;

            // 마커, 코너 인식
            if (ids.Length > 0 && corners.Length > 0)
            {
                // 마커 정보, 코너 정보, 카메라 정보, 외곡정보, 출력받을 변수
                Cv2.SolvePnP(objectPoints, corners[0], cameraMatrix, distCoeffs, ref rvec, ref tvec);
                // ui에 마커 위치 출력
                idsText.text = $"ID: {ids[0]}\nx: {tvec[0]:F3}\ny: {tvec[1]:F3}\nz: {tvec[2]:F3}";

                
                worldRotation = RvecToWorldRotation(rvec);
                // 카드 위치 보정값과 회전을 곱해 위치와 더해 실제 큐브가 나타날 위치를 조정함
                worldPosition = TvecToWorldPos(tvec) + worldRotation * localOffset;
            }
            else
            {
                idsText.text = "ID: n\nx: N\ny: N\nz: N";
            }
            // 보간으로 부드럽게 움직이도록 구성
            cube.transform.position = Vector3.Lerp(cube.transform.position, worldPosition, 0.6f);
            cube.transform.rotation = Quaternion.Slerp(cube.transform.rotation, worldRotation, 0.3f);

            


            timer = 0;
        }

    }

    // 상대 위치를 유니티 월드 좌표로 변환
    private Vector3 TvecToWorldPos(double[] tvec)
    {
        // opencv좌표계에서 유니티 좌표계로 변경하기 위해 y축 반전
        Vector3 localPos = new Vector3((float)tvec[0], -(float)tvec[1], (float)tvec[2]);
        // 카메라 정보 받기
        var cameraPose = passthroughCameraAccess.GetCameraPose();
        // 카메라 위치 + 회전을 반영한 상대위치로 월드위치 계산
        return cameraPose.position + cameraPose.rotation * localPos;
    }

    private Quaternion RvecToWorldRotation(double[] rvec)
    {
        using Mat rvecMat = new Mat(3, 1, MatType.CV_64FC1);

        rvecMat.Set(0, 0, rvec[0]);
        rvecMat.Set(1, 0, rvec[1]);
        rvecMat.Set(2, 0, rvec[2]);

        using Mat rotationMatrix = new Mat();
        Cv2.Rodrigues(rvecMat, rotationMatrix);

        // quaternion용 44행렬 생성
        Matrix4x4 m = Matrix4x4.identity;

        // 매트릭스의 행렬 지정후 가져올 매트릭스의 타입과 행렬 지정해 가져와 float으로 형변환
        m.m00 = (float)rotationMatrix.At<double>(0, 0);
        m.m01 = (float)rotationMatrix.At<double>(0, 1);
        m.m02 = (float)rotationMatrix.At<double>(0, 2);

        m.m10 = (float)rotationMatrix.At<double>(1, 0);
        m.m11 = (float)rotationMatrix.At<double>(1, 1);
        m.m12 = (float)rotationMatrix.At<double>(1, 2);

        m.m20 = (float)rotationMatrix.At<double>(2, 0);
        m.m21 = (float)rotationMatrix.At<double>(2, 1);
        m.m22 = (float)rotationMatrix.At<double>(2, 2);

        Matrix4x4 flipY = Matrix4x4.Scale(new Vector3(1f, -1f, 1f));

        // opencv와 유니티 회전의 방향차이를 맞추기 위해 S * R * S로 좌표계 변환
        m = flipY * m * flipY;

        var cameraPose = passthroughCameraAccess.GetCameraPose();

        // 카메라의 월드 회전까지 곱해줘 회전 맞추기
        Quaternion worldRotation = cameraPose.rotation * m.rotation;

        // 마커 회전에 맞추기 위한 반대 회전값 생성
        Quaternion markerOffset = Quaternion.Euler(0f, 0f, 135f);

        // 행렬의 회전 반환
        return worldRotation * markerOffset;
    }
}
