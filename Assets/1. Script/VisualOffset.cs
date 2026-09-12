using UnityEngine;

public class VisualOffset : MonoBehaviour
{
    // 카드 위치 보정 오프셋
    [SerializeField] private Vector3 positionOffset = new Vector3(0f, 0.02f, 0.02f);
    // 회전 보정
    [SerializeField] private Vector3 rotationOffset = new Vector3(180f, 0f, -90f);

    void Start()
    {
        OffsetAdjustment(positionOffset, rotationOffset);
        
    }

    // 지정한 오프셋만큼 위치 이동
    void OffsetAdjustment(Vector3 posOffset, Vector3 rotOffset)
    {
        Quaternion rotationQ = Quaternion.Euler(rotOffset);
        
        // 위치에 회전을 곱해 부모가 아닌 자신이 바라보는 방향으로 이동하도록 조정
        // 그냥 포지션이 아닌 localPosition사용
        transform.localPosition = rotationQ * posOffset;
        transform.localRotation = rotationQ;
    }
}
