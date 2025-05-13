using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using System;

public class MoveToGoalAgent : Agent
{

    const float defaultX = -2f;
    const float defaultY = 0f;
    const float defaultZ = -1f;

    [SerializeField] private Transform targetTransform;

    public override void OnEpisodeBegin() 
    {
        transform.position = new Vector3(defaultX, defaultY, defaultZ);
    }

    public override void CollectObservations(VectorSensor sensor) 
    {
        sensor.AddObservation(transform.position);
        sensor.AddObservation(targetTransform.position);
    }

    public override void OnActionReceived(ActionBuffers actions) 
    {
        float moveX = actions.ContinuousActions[0];
        float moveZ = actions.ContinuousActions[1];

        float moveSpeed = 3f;
        transform.position += new Vector3(moveX, 0, moveZ) * Time.deltaTime * moveSpeed;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> continuousAction = actionsOut.ContinuousActions;
        continuousAction[0] = Input.GetAxisRaw("Horizontal");
        continuousAction[1] = Input.GetAxisRaw("Vertical");
    }

    private void OnTriggerEnter(Collider other) 
    {
        if (other.TryGetComponent<Ball>(out Ball ball))
        {
            SetReward(1f);
            EndEpisode();
        }
        if (other.TryGetComponent<Wall>(out Wall wall))
        {
            SetReward(-1f);
            EndEpisode();
        }
        
    }
}
