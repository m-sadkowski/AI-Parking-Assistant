using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class MoveToGoalAgent : Agent
{
    [SerializeField] private Transform targetTransform;
    [SerializeField] private Material winMaterial;
    [SerializeField] private Material loseMaterial;
    [SerializeField] private MeshRenderer floorMeshRenderer;

    [SerializeField] private Transform frontLeftWheelTransform;
    [SerializeField] private Transform frontRightWheelTransform;


    [Header("Car Settings")]
    [Tooltip("Prędkość jazdy (m/s)")]
    [SerializeField] private float driveSpeed = 5f;
    [Tooltip("Maksymalny kąt skrętu kół (stopnie)")]
    [SerializeField] private float maxSteerAngle = 30f;
    [Tooltip("Szybkość zmiany kąta kół (stopnie/sekundę)")]
    [SerializeField] private float steerSpeed = 180f;
    [Tooltip("Rozstaw osi pojazdu (m)")]
    [SerializeField] private float wheelBase = 1.5f;

    private float currentSteerAngle = 0f;

    public override void OnEpisodeBegin()
    {
        // transform.localPosition = new Vector3(Random.Range(-3.5f, 0f), 0f, Random.Range(-1.5f, 1.5f));
        transform.localPosition = new Vector3(2.75f, 0f, 0f);
        transform.localRotation = Quaternion.Euler(0f, 90f, 0f);
        currentSteerAngle = Random.Range(-1.5f, 1.5f);
        // targetTransform.localPosition = new Vector3(Random.Range(2f, 4f), 0.5f, Random.Range(-1.5f, 1.5f));
        targetTransform.localPosition = new Vector3(-3.25f + 3.75f * Random.Range(0, 6), 0.04f, Random.Range(0, 2) == 0 ? 7f : 7f);
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

        // 3b) Skręt kół
        Vector3 leftEuler = frontLeftWheelTransform.localEulerAngles;
        Vector3 rightEuler = frontRightWheelTransform.localEulerAngles;

        leftEuler.y = currentSteerAngle;
        rightEuler.y = currentSteerAngle;

        frontLeftWheelTransform.localEulerAngles = leftEuler;
        frontRightWheelTransform.localEulerAngles = rightEuler;

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
        if (other.TryGetComponent<Target>(out Target target))
        {
            SetReward(1f);
            floorMeshRenderer.material = winMaterial;
            EndEpisode();
        }
        else if (other.TryGetComponent<Pavement>(out Pavement pavement) || other.TryGetComponent<ParkedCar>(out ParkedCar parkedCar))
        {
            SetReward(-1f);
            floorMeshRenderer.material = loseMaterial;
            EndEpisode();
        }
    }
}