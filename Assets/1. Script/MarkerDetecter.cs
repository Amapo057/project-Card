using UnityEngine;
using UnityEngine.UI;
using Meta.XR;
using OpenCvSharp;
using OpenCvSharp.Aruco;
using System.Threading;
using Unity.Mathematics;
using System.Diagnostics;
using System;

public class MarkerDetecter : MonoBehaviour
{
    [SerializeField] private PassthroughCameraAccess passthroughCameraAccess;
    [SerializeField] private TMPro.TextMeshProUGUI idsText;
    [SerializeField] private GameObject markerAnchor;
    [SerializeField] private float positionDeadZone = 0.002f;
    [SerializeField] private float rotationDeadZone = 1.0f;

    // [SerializeField] private RawImage rawImage;

    // 쓰레드용 작동 변수
    private Thread cvThread;
    private bool isRunning;

    // 쓰레드 넘겨줄 변수
    private Color32[] latestPixels;
    private int imageWidth;
    private int imageHeight;
    // 새 프레임 정보 넘겼는지 여부
    private bool hasNewFrame = false;
    private readonly object frameLock = new object();

    // 쓰레드에서 받을 변수
    private int[] resultIds;
    private double[] resultTvec;
    private double[] resultRvec;
    private bool hasNewResult = false;
    private readonly object resultLock = new object();


    // 마커의 크기인 19mm의 절반 사이즈를 기준으로 삼음
    private float halfSize = 0.019f/2f;

    // 마커 크기 나타내는 변수
    private Point3f[] objectPoints;

    // 카메라 파라미터 나타내는 변수
    private double[,] cameraMatrix;
    // 렌즈 왜곡정보 변수
    private double[] distCoeffs;
    
    private Dictionary dictionary;

    // 탐지 타이머
    private float timer;
    // 마커 탐지 기준(1f/목표 주사율f)
    private float detectionInterval = 1f/10f;

    // 값 저장
    // 넘겨줄 때 포지션
    private Vector3 latestCameraPosition;
    private Quaternion latestCameraRotation;
    private Vector3 resultCameraPosition;
    private Quaternion resultCameraRotation;
    private Vector3 worldPosition = Vector3.zero;
    private Quaternion worldRotation = Quaternion.identity;
    private DetectorParameters detectorParameters;

    // 데드존용 좌표 변수
    Vector3 detectPos;
    Vector3 currentPos;
    Vector3 stablePos;

    

    void Start()
    {
        // 마커 찾기용 변수 값 넣기
        detectorParameters = new DetectorParameters();
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

        // 스레드 시작
        isRunning = true;
        cvThread = new Thread(CVLoop);
        cvThread.Start();

    }


    void Update()
    {
        // 프레임 타이머
        timer += Time.deltaTime;
        // 패스쓰루 카메라 작동 여부 확인
        if (passthroughCameraAccess == null) return;
        if (!passthroughCameraAccess.IsPlaying) return;

        // 프레임 사용 여부 체크
        bool needsFrame;
        lock (frameLock)
        {
            needsFrame = !hasNewFrame;
        }

        // 지정한 주사율로 제한, 프레임이 사용되었을경우 정보 전달
        if (timer >= detectionInterval && needsFrame)
        {
            Stopwatch sw = Stopwatch.StartNew();
            // passthroug같은 meta sdk는 메인 스레드에서 사용
            // 카메라로부터 정보 받기
            var colors = passthroughCameraAccess.GetColors();
            var cameraPose = passthroughCameraAccess.GetCameraPose();
            sw.Stop();
            double passMs = sw.Elapsed.TotalMilliseconds;

            sw.Restart();

            // 컬러32 배열형태로 변형
            Color32[] pixels = colors.ToArray();
            sw.Stop();

            double arrayMs = sw.Elapsed.TotalMilliseconds;

            idsText.text = $"passMS : {passMs}, array time : {arrayMs}";

            int width = passthroughCameraAccess.CurrentResolution.x;
            int height = passthroughCameraAccess.CurrentResolution.y;

            lock (frameLock)
            {
                latestPixels = pixels;
                imageWidth = width;
                imageHeight = height;

                latestCameraPosition = cameraPose.position;
                latestCameraRotation = cameraPose.rotation;

                hasNewFrame = true;
            }
            timer = 0;
        }
        int[] ids = null;
        double[] tvec = null;
        double[] rvec = null;

        Vector3 cameraPosition = Vector3.zero;
        Quaternion cameraRotation = Quaternion.identity;
        lock (resultLock)
        {
            if (hasNewResult)
            {
                ids = resultIds;
                tvec = resultTvec;
                rvec = resultRvec;

                cameraPosition = resultCameraPosition;
                cameraRotation = resultCameraRotation;

                hasNewResult = false;
            }
        }
        if (ids != null && tvec != null && rvec != null)
        {
            worldPosition = TvecToWorldPos(tvec, cameraPosition, cameraRotation);
            worldRotation = RvecToWorldRotation(rvec, cameraRotation);

            // idsText.text = $"ID: {string.Join(", ", ids)}";
        }
        else
        {
            // idsText.text = "ID: N";
        }

        // 보간으로 부드럽게 움직이도록 구성
        markerAnchor.transform.position = Vector3.Lerp(markerAnchor.transform.position, worldPosition, 0.8f);
        markerAnchor.transform.rotation = Quaternion.Slerp(markerAnchor.transform.rotation, worldRotation, 1f);
    }

    // passthrough로 받은 영상 opencv용으로 전처리
    Mat PreparationCV(Color32[] pixels, int width, int height)
    {
        // 이미지를 cv가 읽을 수 있는 mat형태로 변형
        // mat은 네이티브 메모리를 사용하기때문에 using을 붙여 함수 종료시 알아서 제거도록 사용
        using Mat frame = OpenCvSharp.Unity.PixelsToMat(pixels, width, height);

        // 흑백 변환을 담을 인스턴트 생성
        Mat gray = new Mat();

        // 흑백으로 변경
        Cv2.CvtColor(frame, gray, ColorConversionCodes.BGR2GRAY);

        return gray;
    }

    // 전처리된 흑백 영상을 받아 마커 탐색후 마커 아이디와 위치, 회전 반환
    (int[] ids, double[] tvec, double[] rvec) DetectMarker(Mat gray)
    {
        // 검출한 정보 담기용 변수
        int[] ids;
        Point2f[][] corners;
        Point2f[][] rejected;

        // 마커 감지
        CvAruco.DetectMarkers(gray, dictionary, out corners, out ids, detectorParameters, out rejected);

        // 마커 위치
        double[] tvec = null;
        // 마커 회전 정보
        double[] rvec = null;

        if (ids.Length > 0 && corners.Length > 0)
        {
            // 마커 정보, 코너 정보, 카메라 정보, 외곡정보, 출력받을 변수
            Cv2.SolvePnP(objectPoints, corners[0], cameraMatrix, distCoeffs, ref rvec, ref tvec);
        }        
        return (ids, tvec, rvec);
    }

    // 상대 위치를 유니티 월드 좌표로 변환
    private Vector3 TvecToWorldPos(double[] tvec, Vector3 cameraPosition, Quaternion cameraRotation)
    {
        // opencv좌표계에서 유니티 좌표계로 변경하기 위해 y축 반전
        Vector3 localPos = new Vector3((float)tvec[0], -(float)tvec[1], (float)tvec[2]);

        // 카메라 위치 + 회전을 반영한 상대위치로 월드위치 계산
        return cameraPosition + cameraRotation * localPos;
    }

    // 상대 회전을 유니티 회전인 quaternion으로 변환
    private Quaternion RvecToWorldRotation(double[] rvec, Quaternion cameraRotation)
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

        // 카메라의 월드 회전까지 곱해줘 회전 맞추기
        Quaternion worldRotation = cameraRotation * m.rotation;

        // 마커 회전인 135도 보정
        Quaternion markerOffset = Quaternion.Euler(0f, 0f, 135f);

        // 행렬의 회전 반환
        return worldRotation * markerOffset;
    }

    void CVLoop()
    {
        while (isRunning)
        {
            Color32[] pixels = null;
            int width = 0;
            int height = 0;
            Vector3 cameraPosition = Vector3.zero;
            Quaternion cameraRotation = Quaternion.identity;

            lock (frameLock)
            {
                if (hasNewFrame)
                {
                    pixels = latestPixels;
                    width = imageWidth;
                    height = imageHeight;

                    cameraPosition = latestCameraPosition;
                    cameraRotation = latestCameraRotation;
                    
                    hasNewFrame = false;
                }
            }

            // 프레임 정보 없을시 넘기기
            if(pixels == null)
            {
                Thread.Sleep(1);
                continue;
            }

            using Mat gray = PreparationCV(pixels, width, height);

            var result = DetectMarker(gray);

            lock (resultLock)
            {
                resultIds = result.ids;
                resultTvec = result.tvec;
                resultRvec = result.rvec;

                resultCameraPosition = cameraPosition;
                resultCameraRotation = cameraRotation;

                hasNewResult = true;
            }
        }
    }
    // 코드 종료시 자동 호출
    void OnDestroy()
    {
        // 스레드 종료
        isRunning = false;
        if(cvThread != null && cvThread.IsAlive)
        {
            // cv 종료시 까지 500ms만 대기
            cvThread.Join(500);
        }
    }
}
