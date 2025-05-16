using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class MoveToGoalAgent : Agent
{
    [Header("References")]
    [SerializeField] private Transform targetTransform;
    [SerializeField] private Material winMaterial;
    [SerializeField] private Material loseMaterial;
    [SerializeField] private MeshRenderer floorMeshRenderer;

    [SerializeField] private Transform frontLeftWheelTransform;
    [SerializeField] private Transform frontRightWheelTransform;

    [Header("Car Settings")]
    [Tooltip("Prędkość jazdy (m/s)")][SerializeField] private float driveSpeed = 10f;
    [Tooltip("Maksymalny kąt skrętu (°)")][SerializeField] private float maxSteerAngle = 30f;
    [Tooltip("Szybkość zmiany kąta (°/s)")][SerializeField] private float steerSpeed = 180f;
    [Tooltip("Rozstaw osi (m)")][SerializeField] private float wheelBase = 1.5f;

    [Header("Reward settings")]
    [SerializeField] private float progressCoef = 0.3f;   // wzmocnienie za postęp
    [SerializeField] private float timePenalty = -0.001f; // kara za każdy krok
    [SerializeField] private float lineEnterPenalty = -0.5f;   // kara jednorazowa za dotknięcie linii
    [SerializeField] private float lineStayPenalty = -0.01f;  // kara co krok podczas przebywania na linii

    private float currentSteerAngle = 0f;
    private float prevDistance;

    // ===================== EPISODE =====================
    public override void OnEpisodeBegin()
    {
        // reset pozycji (tu: deterministycznie; można losować)
        transform.localPosition = new Vector3(2.75f, 0f, 0f);
        transform.localRotation = Quaternion.Euler(0f, 90f, 0f);
        currentSteerAngle = Random.Range(-1.5f, 1.5f);

        targetTransform.localPosition = new Vector3(8.25f, 0.04f, 6.75f);

        prevDistance = Vector3.Distance(transform.localPosition, targetTransform.localPosition);

        // reset materiału podłoża
        floorMeshRenderer.material = null;
    }

    // ================== OBSERVATIONS ==================
    public override void CollectObservations(VectorSensor sensor)
    {
        // lepsze: podawać różnicę, aby była niezależna od globalnych współrzędnych
        Vector3 toTarget = targetTransform.localPosition - transform.localPosition;

        sensor.AddObservation(toTarget);                // 3 wartości
        sensor.AddObservation(transform.forward);       // 3 wartości
        sensor.AddObservation(currentSteerAngle / maxSteerAngle);
    }

    // ==================== ACTIONS =====================
    public override void OnActionReceived(ActionBuffers actions)
    {
        float steerInput = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float throttleInput = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);

        // ---------- sterowanie pojazdem (bez zmian) ----------
        float targetSteerAngle = steerInput * maxSteerAngle;
        currentSteerAngle = Mathf.MoveTowards(currentSteerAngle, targetSteerAngle, steerSpeed * Time.deltaTime);

        float steerAngleRad = currentSteerAngle * Mathf.Deg2Rad;
        float turnRadius = Mathf.Abs(Mathf.Tan(steerAngleRad)) > 0.001f
            ? wheelBase / Mathf.Tan(steerAngleRad)
            : Mathf.Infinity;
        float angularVelocity = (throttleInput * driveSpeed) / turnRadius;
        float turnDegrees = angularVelocity * Mathf.Rad2Deg * Time.deltaTime;

        transform.Rotate(0f, turnDegrees, 0f);

        Vector3 move = transform.forward * throttleInput * driveSpeed * Time.deltaTime;
        transform.localPosition += move;

        // ---------- NAGRODY/KARY ----------
        AddReward(timePenalty);               // kara za „marnowanie czasu”

        float distance = Vector3.Distance(transform.localPosition, targetTransform.localPosition);
        float progress = prevDistance - distance;       // dodatnie = przybliżył się

        AddReward(progress * progressCoef);  // shaping: nagroda za postęp

        prevDistance = distance;
    }

    // ================== HEURYSTYKA ====================
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var cont = actionsOut.ContinuousActions;
        cont[0] = Input.GetAxisRaw("Horizontal");
        cont[1] = Input.GetAxisRaw("Vertical");
    }

    // ================== KOLIZJE =======================
    private void OnTriggerEnter(Collider other)
    {
        if (other.TryGetComponent<Target>(out Target target))
        {
            SetReward(1f);                             // finalna nagroda
            floorMeshRenderer.material = winMaterial;
            EndEpisode();
        }
        else if (other.TryGetComponent<Pavement>(out Pavement pavement))
        {
            SetReward(-1f);
            floorMeshRenderer.material = loseMaterial;
            EndEpisode();
        }
        else if (other.TryGetComponent<Line>(out Line line))
        {
            AddReward(lineEnterPenalty);               // jednorazowa kara za „najechanie” na linię
        }
    }

    private void OnTriggerStay(Collider other)
    {
        if (other.TryGetComponent<Line>(out Line line))
        {
            AddReward(lineStayPenalty);                // ciągła kara, dopóki stoi na linii
        }
    }
}
