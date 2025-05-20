﻿using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using System.Collections.Generic;

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
    private float turnDegrees = 0f;

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
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // 16 sensorów - kąty co 22.5 stopnia, długość 10m
        float sensorLength = 10f;
        for (int i = 0; i < 16; i++)
        {
            float angle = i * 22.5f;
            Vector3 dir = Quaternion.Euler(0f, angle, 0f) * transform.forward;
            if (Physics.Raycast(transform.position, dir, out RaycastHit hit, sensorLength))
            {
                sensor.AddObservation(hit.distance / sensorLength);
            }
            else
            {
                sensor.AddObservation(1f);
            }
        }

        // Dodaj aktualną prędkość (normalizowaną)
        sensor.AddObservation(currentSpeed / driveSpeed);

        // Dodaj prędkość kątową
        sensor.AddObservation(turnDegrees / 10f);

        // Pozycja i rotacja względem celu
        Vector3 relativePos = transform.InverseTransformPoint(parkingSpot.localPosition);
        sensor.AddObservation(relativePos.x / 20f);
        sensor.AddObservation(relativePos.z / 20f);

        // Odległość do celu
        float distanceToGoal = Vector3.Distance(transform.localPosition, parkingSpot.localPosition);
        sensor.AddObservation(distanceToGoal / 20f); // Normalizacja do max 20m

        // Różnica kąta
        float angleToGoal = Mathf.DeltaAngle(transform.eulerAngles.y, parkingSpot.eulerAngles.y) / 180f;
        sensor.AddObservation(angleToGoal);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float steerInput = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float throttleInput = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);

        // Dynamiczna prędkość w zależności od odległości do celu
        float distanceToGoal = Vector3.Distance(transform.localPosition, parkingSpot.localPosition);
        float adaptiveSpeed = driveSpeed * Mathf.Clamp(distanceToGoal / 5f, 0.1f, 1f);

        // Sterowanie kierownicą (bardziej płynne)
        float targetSteerAngle = steerInput * maxSteerAngle;
        currentSteerAngle = Mathf.Lerp(currentSteerAngle, targetSteerAngle, 0.1f);

        // Oblicz promień skrętu i obrót pojazdu
        float steerAngleRad = currentSteerAngle * Mathf.Deg2Rad;
        float turnRadius = Mathf.Abs(Mathf.Tan(steerAngleRad)) > 0.001f ? wheelBase / Mathf.Tan(steerAngleRad) : Mathf.Infinity;
        float angularVelocity = (throttleInput * adaptiveSpeed) / turnRadius;
        turnDegrees = angularVelocity * Mathf.Rad2Deg * Time.deltaTime;

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
        Vector3 move = transform.forward * throttleInput * adaptiveSpeed * Time.deltaTime;
        transform.localPosition += move;

        // Aktualizacja prędkości
        currentSpeed = throttleInput * adaptiveSpeed;

        // Obliczanie nagród
        CalculateReward();
    }

    private void CalculateReward()
    {
        float distanceToGoal = Vector3.Distance(transform.localPosition, parkingSpot.localPosition);
        float angleDiff = Mathf.Abs(Mathf.DeltaAngle(transform.eulerAngles.y, parkingSpot.eulerAngles.y));

        // Nagroda za zbliżanie się do celu
        AddReward((lastDistanceToGoal - distanceToGoal) * 0.5f);

        // Kara za zły kąt względem celu
        AddReward(-angleDiff / 180f * 0.1f);

        // Kara za prędkość (bardziej restrykcyjna)
        AddReward(-Mathf.Abs(currentSpeed) * 0.01f);

        lastDistanceToGoal = distanceToGoal;

        // Sprawdzenie warunku sukcesu
        if (distanceToGoal < 0.5f)
        {
            if (angleDiff < 10f && Mathf.Abs(currentSpeed) < 0.1f)
            {
                SetReward(2f); // Większa nagroda za sukces
                EndEpisode();
            }
            else
            {
                AddReward(-0.2f); // Kara za nieprecyzyjne parkowanie
            }
        }
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
            SetReward(2f);
            EndEpisode();
        }
        else if (other.TryGetComponent<Pavement>(out Pavement pavement) || other.TryGetComponent<ParkedCar>(out ParkedCar parkedCar))
        {
            SetReward(-2f); // Większa kara za kolizje
            EndEpisode();
        }
        else if (other.TryGetComponent<Line>(out Line line))
        {
            AddReward(-0.5f);
        }
    }

    private void OnDrawGizmos()
    {
        // Wizualizacja celu
        Gizmos.color = Color.red;
        Gizmos.DrawSphere(parkingSpot.position, 0.5f);

        // Wizualizacja sensorów
        Gizmos.color = Color.cyan;
        for (int i = 0; i < 16; i++)
        {
            float angle = i * 22.5f;
            Vector3 dir = Quaternion.Euler(0f, angle, 0f) * transform.forward;
            Gizmos.DrawRay(transform.position, dir * 10f);
        }
    }
}