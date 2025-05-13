using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class MoveToGoalAgent : Agent
{
    const float defaultX = -2f;
    const float defaultY = 0f;
    const float defaultZ = -1f;

    [SerializeField] private Transform targetTransform;
    [SerializeField] private Material winMaterial;
    [SerializeField] private Material loseMaterial;
    [SerializeField] private MeshRenderer floorMeshRenderer;

    [Header("Car Settings")]
    [Tooltip("Prędkość jazdy (m/s)")]
    [SerializeField] private float driveSpeed = 3f;
    [Tooltip("Maksymalny kąt skrętu kół (stopnie)")]
    [SerializeField] private float maxSteerAngle = 30f;
    [Tooltip("Szybkość zmiany kąta kół (stopnie/sekundę)")]
    [SerializeField] private float steerSpeed = 180f;
    [Tooltip("Rozstaw osi pojazdu (m)")]
    [SerializeField] private float wheelBase = 1.5f;

    private float currentSteerAngle = 0f;

    public override void OnEpisodeBegin()
    {
        transform.localPosition = new Vector3(defaultX, defaultY, defaultZ);
        transform.localRotation = Quaternion.Euler(0f, 90f, 0f);
        currentSteerAngle = 0f;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(transform.localPosition);
        sensor.AddObservation(targetTransform.localPosition);
        sensor.AddObservation(transform.forward);
        sensor.AddObservation(currentSteerAngle / maxSteerAngle);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float steerInput = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float throttleInput = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);

        // 1) Obliczenie i wygładzenie kąta skrętu kół
        float targetSteerAngle = steerInput * maxSteerAngle;
        currentSteerAngle = Mathf.MoveTowards(
            currentSteerAngle,
            targetSteerAngle,
            steerSpeed * Time.deltaTime
        );

        // 2) Obliczenie promienia skrętu i kąta obrotu pojazdu:
        //    R = wheelBase / tan(δ), a kąt obrotu = (prędkość / R) * dt (w radianach)
        float steerAngleRad = currentSteerAngle * Mathf.Deg2Rad;
        float turnRadius = Mathf.Abs(Mathf.Tan(steerAngleRad)) > 0.001f
            ? wheelBase / Mathf.Tan(steerAngleRad)
            : Mathf.Infinity;
        float angularVelocity = (throttleInput * driveSpeed) / turnRadius;  // rad/s
        float turnDegrees = angularVelocity * Mathf.Rad2Deg * Time.deltaTime;

        // 3) Obrót karoserii:
        transform.Rotate(0f, turnDegrees, 0f);

        // 4) Ruch do przodu/tyłu:
        Vector3 move = transform.forward * throttleInput * driveSpeed * Time.deltaTime;
        transform.localPosition += move;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var cont = actionsOut.ContinuousActions;
        cont[0] = Input.GetAxisRaw("Horizontal");
        cont[1] = Input.GetAxisRaw("Vertical");
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.TryGetComponent<Ball>(out Ball ball))
        {
            SetReward(1f);
            floorMeshRenderer.material = winMaterial;
            EndEpisode();
        }
        else if (other.TryGetComponent<Wall>(out Wall wall))
        {
            SetReward(-1f);
            floorMeshRenderer.material = loseMaterial;
            EndEpisode();
        }
    }
}
