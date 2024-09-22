using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using static UnityEngine.GraphicsBuffer;

public class KinematicArrive : MonoBehaviour
{
    [SerializeField] private float maxSpeed = 5f;
    [SerializeField] private float radiusOfSat = 2f;
    [SerializeField] private float timeToTarget = 0.25f;
    [SerializeField] private float steeringSpeed = 5f;

    private Rigidbody rb;
    private Vector3 targetPosition;
    private bool isMoving = false;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
    }

    void Update()
    {
        if (Input.GetMouseButtonDown(0) && !isMoving)
        {
            // get mouse position and convert it to a world point on the plane
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            if (Physics.Raycast(ray, out RaycastHit hit))
            {
                targetPosition = hit.point;
                isMoving = true;
            }
        }
    }

    private void FixedUpdate()
    {
        if (isMoving)
        {
            Move();
        }
    }

    private void Move()
    {
        // calculate vector towards the target
        Vector3 direction = targetPosition - transform.position;
        float distance = direction.magnitude;

        // check if we've arrived at the target
        if (distance <= radiusOfSat)
        {
            rb.velocity = Vector3.zero;
            isMoving = false; // stop moving
            return;
        }

        // normalize the direction to get a unit vector
        direction.Normalize();

        // set speed based on timeToTarget
        Vector3 speed = direction * (distance / timeToTarget);

        // clamp speed to maxSpeed
        if (speed.magnitude > maxSpeed)
        {
            speed = speed.normalized * maxSpeed;
        }

        // set the desired velocity
        Vector3 velocity = speed;
        rb.MovePosition(rb.position + velocity * Time.deltaTime);

        // rotate the player to face the direction of movement
        Quaternion targetRotation = Quaternion.LookRotation(direction);
        transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, steeringSpeed * Time.deltaTime);
    }
}