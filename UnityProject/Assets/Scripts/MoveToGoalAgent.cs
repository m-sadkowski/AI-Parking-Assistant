using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

public class MoveToGoalAgent : Agent
{
    [SerializeField] private Transform parkingSpot;
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
    [SerializeField] private float driveSpeed = 5f;
    [SerializeField] private float maxSteerAngle = 30f;
    [SerializeField] private float steerSpeed = 180f;
    [SerializeField] private float wheelBase = 1.5f;

    private Rigidbody rb;
    private float previousDistanceToTarget;
    private Vector3 currentVelocity = Vector3.zero;

    // 🔄 Dodane: informacja o byciu na linii
    private bool isOnLine = false;
    private bool parking_try;
    public override void OnEpisodeBegin()
    {
        parkedCars = new List<Transform> { parkedCar1, parkedCar2, parkedCar3, parkedCar4, parkedCar5, parkedCar6, parkedCar7, parkedCar8 };
        parking_try = false;
        possibleParkingSpots = new List<Vector3>();
        for (int i = 0; i < 6; i++)
        {
            possibleParkingSpots.Add(new Vector3(-7f + 3.75f * i, 0.04f, 2.3f));
            possibleParkingSpots.Add(new Vector3(-7f + 3.75f * i, 0.04f, -11.2f));
        }

        List<Vector3> availableSpots = new List<Vector3>(possibleParkingSpots);
        foreach (Transform car in parkedCars)
        {
            int idx = Random.Range(0, availableSpots.Count);
            car.localPosition = availableSpots[idx];
            availableSpots.RemoveAt(idx);
            float randomYRotation = Random.Range(-10f, 10f);
            car.localRotation = Quaternion.Euler(0f, randomYRotation, 0f);
        }

        int targetIdx = Random.Range(0, availableSpots.Count);
        parkingSpot.localPosition = availableSpots[targetIdx];
        int signZ = (parkingSpot.localPosition.z > 0) ? 1 : -1;
        parkingSpot.localPosition = new Vector3(parkingSpot.localPosition.x + 3.75f, parkingSpot.localPosition.y, 9f * signZ);
        parkingSpot.localRotation = Quaternion.Euler(0f, signZ > 0 ? 180f : 0f, 0f);

        transform.localPosition = new Vector3(2.75f, 0f, 0f);
        transform.localRotation = Quaternion.Euler(0f, 90f, 0f);

        rb = GetComponent<Rigidbody>();
        previousDistanceToTarget = Vector3.Distance(transform.localPosition, parkingSpot.localPosition);

        isOnLine = false; // reset
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        Vector3 toTarget = parkingSpot.localPosition - transform.localPosition;

        sensor.AddObservation(transform.localPosition);
        sensor.AddObservation(transform.forward);
        sensor.AddObservation(parkingSpot.localPosition);
        sensor.AddObservation(parkingSpot.forward);
        sensor.AddObservation(toTarget.normalized);
        sensor.AddObservation(toTarget.magnitude);
        sensor.AddObservation(currentVelocity.magnitude); // symulowana prędkość
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float steerInput = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float driveInput = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);

        float steerAngle = steerInput * maxSteerAngle * Mathf.Deg2Rad;
        float velocity = driveInput * driveSpeed;
        currentVelocity = transform.forward * velocity;

        float angularVelocity = 0f;
        if (Mathf.Abs(steerAngle) > 0.01f)
        {
            float turnRadius = wheelBase / Mathf.Tan(steerAngle);
            angularVelocity = velocity / turnRadius;
        }

        transform.position += currentVelocity * Time.deltaTime;
        transform.Rotate(0f, angularVelocity * Mathf.Rad2Deg * Time.deltaTime, 0f, Space.World);

        frontLeftWheelTransform.localRotation = Quaternion.Euler(0f, steerInput * maxSteerAngle, 0f);
        frontRightWheelTransform.localRotation = Quaternion.Euler(0f, steerInput * maxSteerAngle, 0f);

        CalculateReward();
    }

    private void CalculateReward()
    {
        float distanceToTarget = Vector3.Distance(transform.localPosition, parkingSpot.localPosition);
        float angle = Vector3.Angle(transform.forward, parkingSpot.forward);
        float distanceDelta = previousDistanceToTarget - distanceToTarget;
        float forwardAlignment = Vector3.Dot(transform.forward.normalized, (parkingSpot.position - transform.position).normalized);

        if (forwardAlignment < 0)
            AddReward(-0.1f); // kara za jazdę tyłem

        if (distanceDelta > 0f)
            AddReward(distanceDelta * 0.5f); // postęp
        else
            AddReward(distanceDelta * 0.5f); // cofanie = kara

        if (distanceToTarget < 2f && angle < 30f && parking_try == false)
        {
            AddReward(0.2f); // zachęta za próbę zaparkowania
            parking_try = true;
        }
            

        if (distanceToTarget < 0.3f)
            AddReward(-angle / 180f * 0.3f); // kara za zły kąt blisko celu



        if (distanceToTarget < 1.0f && angle < 15f)
        {
            SetReward(15.0f); // duża nagroda za sukces
            if (isOnLine) AddReward(-5f);
            EndEpisode();
        }

        AddReward(-0.001f); // kara za czas

        previousDistanceToTarget = distanceToTarget;
    }


    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var cont = actionsOut.ContinuousActions;
        cont[0] = Input.GetAxisRaw("Horizontal");
        cont[1] = Input.GetAxisRaw("Vertical");
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.TryGetComponent<ParkedCar>(out _))
        {
            AddReward(-1f);
            EndEpisode();
        }
        else if (other.TryGetComponent<Pavement>(out _))
        {
            AddReward(-1f);
            EndEpisode();
        }
        else if (other.TryGetComponent<Line>(out _))
        {
            isOnLine = true;
            //AddReward(-0.3f);
        }
    }

    private void OnTriggerExit(Collider other)
    {
        if (other.TryGetComponent<Line>(out _))
        {
            isOnLine = false;
           // AddReward(0.3f); // bonus za zejście z linii
        }
    }
}
