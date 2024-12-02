using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class KinematicArrive : MonoBehaviour
{
    [SerializeField] private float maxSpeed = 5f;
    [SerializeField] private float radiusOfSat = 2f;
    [SerializeField] private float timeToTarget = 0.25f;
    [SerializeField] private float steeringSpeed = 5f;
    [SerializeField] private GameObject obstaclePrefab;
    [SerializeField] private float avoidanceDistance = 5f;
    [SerializeField] private float arcRadius = 2f;
    [SerializeField] private int rayCount = 5;

    public Transform[] followers;
    public Vector3[] formationOffsets;

    private Rigidbody rb;
    private Vector3 targetPosition;
    private bool isMoving = false;
    private LayerMask obstacleLayer;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        obstacleLayer = LayerMask.GetMask("Obstacle");
    }

    void Update()
    {
        // Left-click to move formation
        if (Input.GetMouseButtonDown(0) && !isMoving)
        {
            // Get mouse position and convert it to a world point on the plane
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            if (Physics.Raycast(ray, out RaycastHit hit))
            {
                targetPosition = hit.point;
                isMoving = true;
            }
        }

        // Right-click to place obstacles
        if (Input.GetMouseButtonDown(1))
        {
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            if (Physics.Raycast(ray, out RaycastHit hit))
            {
                Instantiate(obstaclePrefab, hit.point, Quaternion.identity);
            }
        }
    }

    private void FixedUpdate()
    {
        if (isMoving)
        {
            MoveLeader();
        }

        MoveFollowers();
    }

    private void MoveLeader()
    {
        // Calculate vector towards the target
        Vector3 direction = targetPosition - transform.position;
        float distance = direction.magnitude;

        // Check for obstacle avoidance
        if (CheckForObstacle(ref direction))
        {
            // Apply the circular arc avoidance
            direction = GetCircularArcPosition(direction);
        }

        // Check if we've arrived at the target
        if (distance <= radiusOfSat)
        {
            rb.velocity = Vector3.zero;
            isMoving = false; // Stop moving
            return;
        }

        // Normalize the direction to get a unit vector
        direction.Normalize();

        // Set speed based on timeToTarget
        Vector3 velocity = direction * (distance / timeToTarget);

        // Clamp speed to maxSpeed
        if (velocity.magnitude > maxSpeed)
        {
            velocity = velocity.normalized * maxSpeed;
        }

        // Rotate the leader to face the direction of movement
        rb.MovePosition(rb.position + velocity * Time.deltaTime);
        Quaternion targetRotation = Quaternion.LookRotation(direction);
        transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, steeringSpeed * Time.deltaTime);
    }

    private void MoveFollowers()
    {
        for (int i = 0; i < followers.Length; i++)
        {
            // Calculate the follower's target position in the formation
            Vector3 leaderPosition = transform.position;
            Vector3 targetPosition = leaderPosition + transform.TransformDirection(formationOffsets[i]);

            // Avoid obstacles by modifying movement
            Vector3 directionToTarget = targetPosition - followers[i].position;
            float distanceToTarget = directionToTarget.magnitude;

            // Check for obstacle avoidance for followers
            if (CheckForObstacle(followers[i].position, ref directionToTarget))
            {
                // Apply the circular arc avoidance
                Vector3 avoidanceTarget = GetCircularArcPosition(directionToTarget);
                directionToTarget = avoidanceTarget - followers[i].position; // Update the direction to the new target
            }

            // Move the follower toward the target position
            if (distanceToTarget > radiusOfSat)
            {
                directionToTarget.Normalize();
                Vector3 followerVelocity = directionToTarget * (distanceToTarget / timeToTarget);

                // Clamp follower velocity to maxSpeed
                if (followerVelocity.magnitude > maxSpeed)
                {
                    followerVelocity = followerVelocity.normalized * maxSpeed;
                }

                // Apply movement and rotation
                Rigidbody followerRb = followers[i].GetComponent<Rigidbody>();
                followerRb.MovePosition(followerRb.position + followerVelocity * Time.deltaTime);
                Quaternion followerRotation = Quaternion.LookRotation(followerVelocity);
                followers[i].rotation = Quaternion.Slerp(followers[i].rotation, followerRotation, steeringSpeed * Time.deltaTime);
            }
        }
    }

    private bool CheckForObstacle(ref Vector3 direction)
    {
        // Cast multiple rays in a fan shape
        for (int i = -rayCount / 2; i <= rayCount / 2; i++)
        {
            // Calculate the angle offset
            float angle = i * (30f / rayCount); // Adjust the angle spread as needed
            Vector3 rayDirection = Quaternion.Euler(0, angle, 0) * direction;

            // Check for obstacles
            if (Physics.Raycast(transform.position, rayDirection.normalized, out RaycastHit hit, avoidanceDistance, obstacleLayer))
            {
                // If an obstacle is detected, return true
                Debug.DrawRay(transform.position, rayDirection.normalized * hit.distance, Color.red);
                return true;
            }
            else
            {
                // Draw rays for debugging
                Debug.DrawRay(transform.position, rayDirection.normalized * avoidanceDistance, Color.blue);
            }
        }

        return false;
    }

    private bool CheckForObstacle(Vector3 position, ref Vector3 direction)
    {
        // Cast multiple rays in a fan shape for followers
        for (int i = -rayCount / 2; i <= rayCount / 2; i++)
        {
            // Calculate the angle offset
            float angle = i * (30f / rayCount); // Adjust the angle spread as needed
            Vector3 rayDirection = Quaternion.Euler(0, angle, 0) * direction;

            // Check for obstacles
            if (Physics.Raycast(position, rayDirection.normalized, out RaycastHit hit, avoidanceDistance, obstacleLayer))
            {
                // If an obstacle is detected, return true
                Debug.DrawRay(position, rayDirection.normalized * hit.distance, Color.red);
                return true;
            }
            else
            {
                // Draw rays for debugging
                Debug.DrawRay(position, rayDirection.normalized * avoidanceDistance, Color.blue);
            }
        }

        return false;
    }

    private Vector3 GetCircularArcPosition(Vector3 direction)
    {
        // Calculate a new position that creates a circular arc around the obstacle
        Vector3 avoidancePoint = transform.position + direction.normalized * avoidanceDistance;

        // Calculate a direction that creates a circular arc around the obstacle
        Vector3 arcDirection = Vector3.Cross(Vector3.up, direction).normalized; // Perpendicular direction
        Vector3 circularArcPosition = avoidancePoint + arcDirection * arcRadius; // Create the arc

        return circularArcPosition - transform.position; // Return the new target position relative to the character
    }
}