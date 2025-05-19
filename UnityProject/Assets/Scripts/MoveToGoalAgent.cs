using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using System.Collections.Generic;

public class MoveToGoalAgent : Agent
{
    [SerializeField] private Transform parkingSpot;
    [SerializeField] private Material winMaterial;
    [SerializeField] private Material loseMaterial;
    [SerializeField] private MeshRenderer floorMeshRenderer;
    [SerializeField] private Transform frontLeftWheelTransform;
    [SerializeField] private Transform frontRightWheelTransform;

    [Header("Parked cars")]
    [SerializeField] private Transform parkedCar1;
    [SerializeField] private Transform parkedCar2;
    [SerializeField] private Transform parkedCar3;
    [SerializeField] private Transform parkedCar4;
    [SerializeField] private Transform parkedCar5;
    [SerializeField] private Transform parkedCar6;
    [SerializeField] private Transform parkedCar7;
    [SerializeField] private Transform parkedCar8;
    private List<Transform> parkedCars;
    private List<Vector3> possibleParkingSpots;

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
    private float currentSpeed = 0f;
    private float lastDistanceToGoal = Mathf.Infinity;

    public override void OnEpisodeBegin()
    {
        // Inicjalizacja list
        parkedCars = new List<Transform> { parkedCar1, parkedCar2, parkedCar3, parkedCar4, parkedCar5, parkedCar6, parkedCar7, parkedCar8 };

        possibleParkingSpots = new List<Vector3>();
        for (int i = 0; i < 6; i++)
        {
            possibleParkingSpots.Add(new Vector3(-3.25f + 3.75f * i, 0.04f, 2.3f));
            possibleParkingSpots.Add(new Vector3(-3.25f + 3.75f * i, 0.04f, -11.2f));
        }

        // Losowe ustawienie zaparkowanych aut
        List<Vector3> availableSpots = new List<Vector3>(possibleParkingSpots);
        foreach (Transform car in parkedCars)
        {
            int idx = Random.Range(0, availableSpots.Count);
            car.localPosition = availableSpots[idx];
            availableSpots.RemoveAt(idx);

            float randomYRotation = Random.Range(-10f, 10f);
            car.localRotation = Quaternion.Euler(0f, randomYRotation, 0f);
        }

        // Losowy cel parkowania
        int targetIdx = Random.Range(0, availableSpots.Count);
        parkingSpot.localPosition = availableSpots[targetIdx];
        int signZ = (parkingSpot.localPosition.z > 0) ? 1 : -1;
        parkingSpot.localPosition = new Vector3(parkingSpot.localPosition.x + 3.25f, parkingSpot.localPosition.y, 8f * signZ);
        parkingSpot.localRotation = Quaternion.Euler(0f, signZ > 0 ? 180f : 0f, 0f);

        // Reset pozycji i rotacji agenta
        transform.localPosition = new Vector3(2.75f, 0f, 0f);
        transform.localRotation = Quaternion.Euler(0f, 90f, 0f);

        currentSteerAngle = 0f;
        currentSpeed = 0f;
        lastDistanceToGoal = Vector3.Distance(transform.localPosition, parkingSpot.localPosition);

        floorMeshRenderer.material = null;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        Vector3 relativePos = parkingSpot.localPosition - transform.localPosition;

        // Pozycja względna do celu
        sensor.AddObservation(relativePos.x);
        sensor.AddObservation(relativePos.z);

        // Kąt między kierunkiem pojazdu a celem, znormalizowany [-1,1]
        float angleToGoal = Vector3.SignedAngle(transform.forward, relativePos.normalized, Vector3.up);
        sensor.AddObservation(angleToGoal / 180f);

        // Kąt skrętu kół
        sensor.AddObservation(currentSteerAngle / maxSteerAngle);

        // Prędkość pojazdu (normalizacja do [-1,1])
        sensor.AddObservation(currentSpeed / driveSpeed);

        // Odległości do zaparkowanych aut (względne pozycje)
        foreach (Transform car in parkedCars)
        {
            Vector3 relCarPos = car.localPosition - transform.localPosition;
            sensor.AddObservation(relCarPos.x);
            sensor.AddObservation(relCarPos.z);
        }

        // Czujniki odległości (8 kierunków)
        float[] distances = GetDistanceSensors();
        foreach (var dist in distances)
        {
            sensor.AddObservation(dist);
        }
    }

    private float[] GetDistanceSensors()
    {
        float maxDist = 10f;
        float[] sensorDistances = new float[8];
        Vector3[] directions = new Vector3[]
        {
            transform.forward,
            Quaternion.Euler(0, 45, 0) * transform.forward,
            Quaternion.Euler(0, -45, 0) * transform.forward,
            transform.right,
            -transform.right,
            Quaternion.Euler(0, 135, 0) * transform.forward,
            Quaternion.Euler(0, -135, 0) * transform.forward,
            -transform.forward
        };
        for (int i = 0; i < directions.Length; i++)
        {
            Ray ray = new Ray(transform.position + Vector3.up * 0.5f, directions[i]);
            if (Physics.Raycast(ray, out RaycastHit hit, maxDist))
            {
                sensorDistances[i] = hit.distance / maxDist;
            }
            else
            {
                sensorDistances[i] = 1f;
            }
        }
        return sensorDistances;
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float steerInput = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float throttleInput = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);

        // Sterowanie kierownicą
        float targetSteerAngle = steerInput * maxSteerAngle;
        currentSteerAngle = Mathf.MoveTowards(currentSteerAngle, targetSteerAngle, steerSpeed * Time.deltaTime);

        // Oblicz promień skrętu i obrót pojazdu
        float steerAngleRad = currentSteerAngle * Mathf.Deg2Rad;
        float turnRadius = Mathf.Abs(Mathf.Tan(steerAngleRad)) > 0.001f ? wheelBase / Mathf.Tan(steerAngleRad) : Mathf.Infinity;
        float angularVelocity = (throttleInput * driveSpeed) / turnRadius;  // rad/s
        float turnDegrees = angularVelocity * Mathf.Rad2Deg * Time.deltaTime;

        // Obrót pojazdu
        transform.Rotate(0f, turnDegrees, 0f);

        // Skręt kół wizualnie
        Vector3 leftEuler = frontLeftWheelTransform.localEulerAngles;
        Vector3 rightEuler = frontRightWheelTransform.localEulerAngles;
        leftEuler.y = currentSteerAngle;
        rightEuler.y = currentSteerAngle;
        frontLeftWheelTransform.localEulerAngles = leftEuler;
        frontRightWheelTransform.localEulerAngles = rightEuler;

        // Ruch pojazdu
        Vector3 move = transform.forward * throttleInput * driveSpeed * Time.deltaTime;
        transform.localPosition += move;

        // Aktualizacja prędkości (może być ujemna, jeśli cofa się)
        currentSpeed = throttleInput * driveSpeed;

        // Obliczanie nagród w trakcie epizodu
        CalculateReward();
    }

    private void CalculateReward()
    {
        Vector3 toGoal = parkingSpot.localPosition - transform.localPosition;
        float distance = toGoal.magnitude;

        // Mała nagroda za zmniejszanie odległości do celu
        float proximityReward = Mathf.Clamp(1f - (distance / 20f), 0f, 1f);
        AddReward(proximityReward * 0.01f);

        // Negatywna nagroda za oddalanie się
        if (distance > lastDistanceToGoal)
        {
            AddReward(-0.005f);
        }
        lastDistanceToGoal = distance;

        // Premia za dobre ustawienie kąta przy małej odległości
        if (distance < 0.5f)
        {
            float angleDiff = Mathf.Abs(Vector3.SignedAngle(transform.forward, parkingSpot.forward, Vector3.up));
            float angleReward = 1f - (angleDiff / 90f);
            AddReward(angleReward * 0.1f);
        }

        // Możesz dodać premię za szybkość parkowania, np. mniejszą liczbę kroków (to można zrobić osobno)
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
            AddReward(-1f);
            floorMeshRenderer.material = loseMaterial;
            EndEpisode();
        }
        else if (other.TryGetComponent<Line>(out Line parkingLine))
        {
            AddReward(-0.9f);
            floorMeshRenderer.material = loseMaterial;
            EndEpisode();
        }
    }
}
