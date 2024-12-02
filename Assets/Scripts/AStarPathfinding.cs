using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
//using static AStarPathfinding;

public class AStarPathfinding : MonoBehaviour
{
    [SerializeField] private float maxSpeed = 5f;
    [SerializeField] private float radiusOfSat = 0.5f;
    [SerializeField] private float steeringSpeed = 5f;
    [SerializeField] private GameObject obstaclePrefab;
    [SerializeField] private float nodeSize = 1f;

    public Transform[] followers;
    public Vector3[] formationOffsets;

    private Rigidbody rb;
    private Vector3 targetPosition;
    private bool isMoving = false;

    public GameObject ground;
    private Grid grid;
    private List<Vector3> path = new List<Vector3>(); // A* path waypoints
    private int currentWaypointIndex = 0;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        grid = new Grid(ground, nodeSize);
        Debug.Log($"Grid Size: {grid.gridSizeX} x {grid.gridSizeY}");
    }

    void Update()
    {
        // left-click to set target position
        if (Input.GetMouseButtonDown(0) && !isMoving)
        {
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            if (Physics.Raycast(ray, out RaycastHit hit))
            {
                targetPosition = hit.point;
                CalculatePath(transform.position, targetPosition); // calculate A* path
                isMoving = true;
                currentWaypointIndex = 0;
            }
        }

        // right-click to place obstacles
        if (Input.GetMouseButtonDown(1))
        {
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            if (Physics.Raycast(ray, out RaycastHit hit))
            {
                Instantiate(obstaclePrefab, hit.point, Quaternion.identity);
                MarkNodesAsUnwalkable(hit.point); // mark nearby nodes as non-walkable
            }
        }
    }

    private void FixedUpdate()
    {
        if (isMoving)
        {
            MoveAlongPath();
        }
        MoveFollowers();
    }

    private void OnDrawGizmos()
    {
        if (grid == null) return;

        // Draw the grid
        for (int x = 0; x < grid.gridSizeX; x++)
        {
            for (int y = 0; y < grid.gridSizeY; y++)
            {
                Node node = grid.nodes[x, y];

                // Set the color based on whether the node is walkable or not
                Gizmos.color = node.isWalkable ? Color.green : Color.red;

                // Draw the node as a small cube at the node's world position
                Gizmos.DrawWireCube(node.worldPosition, new Vector3(nodeSize, 0f, nodeSize));
            }
        }

        // If path exists, draw it as a line
        if (path != null && path.Count > 0)
        {
            Gizmos.color = Color.blue;
            for (int i = 0; i < path.Count - 1; i++)
            {
                Gizmos.DrawLine(path[i], path[i + 1]);
            }
        }

        // Optionally, you can draw the start and end positions for clarity
        if (path.Count > 0)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawSphere(path[0], 0.2f); // Start
            Gizmos.DrawSphere(path[path.Count - 1], 0.2f); // End
        }
    }

    private void MarkNodesAsUnwalkable(Vector3 position)
    {
        float nodeRadius = grid.nodeSize;

        for (int x = 0; x < grid.gridSizeX; x++)
        {
            for (int y = 0; y < grid.gridSizeY; y++)
            {
                Node node = grid.nodes[x, y];
                float distance = Vector3.Distance(position, node.worldPosition);

                if (distance <= nodeRadius + 1f)
                {
                    node.isWalkable = false;
                }
            }
        }
    }

    private void MoveAlongPath()
    {
        if (path == null || path.Count == 0 || currentWaypointIndex >= path.Count)
        {
            isMoving = false;
            rb.velocity = Vector3.zero;
            return;
        }

        Vector3 waypoint = path[currentWaypointIndex];
        Vector3 direction = waypoint - transform.position;
        float distance = direction.magnitude;

        Vector3 velocity = Vector3.zero;
        // if the character is close enough to the current waypoint, move to the next
        if (direction.sqrMagnitude > radiusOfSat * radiusOfSat)
        {
            direction.Normalize();
            velocity = direction * maxSpeed;
            rb.MovePosition(rb.position + velocity * Time.fixedDeltaTime);
        }
        else
        {
            currentWaypointIndex++; // move to the next waypoint
        }

        rb.MovePosition(rb.position + velocity * Time.fixedDeltaTime);

        // smoothly rotate towards the waypoint
        if (velocity != Vector3.zero)
        {
            Quaternion targetRotation = Quaternion.LookRotation(velocity);
            transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, steeringSpeed * Time.deltaTime);
        }
    }

    private void MoveFollowers()
    {
        for (int i = 0; i < followers.Length; i++)
        {
            Vector3 leaderPosition = transform.position;
            Vector3 targetPosition = leaderPosition + transform.TransformDirection(formationOffsets[i]);
            Vector3 direction = targetPosition - followers[i].position;

            if (direction.magnitude > radiusOfSat)
            {
                direction.Normalize();
                Vector3 followerVelocity = direction * maxSpeed;

                Rigidbody followerRb = followers[i].GetComponent<Rigidbody>();
                followerRb.MovePosition(followerRb.position + followerVelocity * Time.deltaTime);

                Quaternion targetRotation = Quaternion.LookRotation(direction);
                followers[i].rotation = Quaternion.Slerp(followers[i].rotation, targetRotation, steeringSpeed * Time.deltaTime);
            }
        }
    }

    public void CalculatePath(Vector3 startPosition, Vector3 targetPosition)
    {
        Node startNode = grid.GetNodeFromWorldPosition(startPosition);
        Node targetNode = grid.GetNodeFromWorldPosition(targetPosition);
        Debug.Log($"Start Node: ({startNode.gridX}, {startNode.gridY}), Target Node: ({targetNode.gridX}, {targetNode.gridY})");

        List<Node> openList = new List<Node>();
        HashSet<Node> closedList = new HashSet<Node>();
        openList.Add(startNode);

        while (openList.Count > 0)
        {
            Node currentNode = openList[0];
            for (int i = 1; i < openList.Count; i++)
            {
                if (openList[i].fCost < currentNode.fCost || (openList[i].fCost == currentNode.fCost && openList[i].hCost < currentNode.hCost))
                {
                    currentNode = openList[i];
                }
            }

            openList.Remove(currentNode);
            closedList.Add(currentNode);

            if (currentNode == targetNode)
            {
                RetracePath(startNode, targetNode);
                return;
            }

            foreach (Node neighbor in grid.GetNeighbors(currentNode))
            {
                if (!neighbor.isWalkable || closedList.Contains(neighbor)) continue;

                int newMovementCostToNeighbor = currentNode.gCost + GetDistance(currentNode, neighbor);
                if (newMovementCostToNeighbor < neighbor.gCost || !openList.Contains(neighbor))
                {
                    neighbor.gCost = newMovementCostToNeighbor;
                    neighbor.hCost = GetDistance(neighbor, targetNode);
                    neighbor.parent = currentNode;

                    if (!openList.Contains(neighbor))
                    {
                        openList.Add(neighbor);
                    }
                }
            }
        }
    }

    private void RetracePath(Node startNode, Node endNode)
    {
        List<Vector3> waypoints = new List<Vector3>();
        Node currentNode = endNode;

        while (currentNode != startNode)
        {
            waypoints.Add(currentNode.worldPosition);
            currentNode = currentNode.parent;
        }
        waypoints.Reverse();

        // update the path for the character
        path = waypoints;
        Debug.Log($"Path calculated: {path.Count} waypoints");
    }

    private int GetDistance(Node a, Node b)
    {
        int dstX = Mathf.Abs(a.gridX - b.gridX);
        int dstY = Mathf.Abs(a.gridY - b.gridY);

        return dstX + dstY;
    }

    public class Node
    {
        public Vector3 worldPosition;
        public bool isWalkable;
        public int gridX, gridY;

        public int gCost; // cost from start node
        public int hCost; // heuristic cost to target node
        public int fCost => gCost + hCost;

        public Node parent; // for path reconstruction

        public Node(Vector3 worldPosition, bool isWalkable, int gridX, int gridY)
        {
            this.worldPosition = worldPosition;
            this.isWalkable = isWalkable;
            this.gridX = gridX;
            this.gridY = gridY;
        }
    }

    public class Grid
    {
        public Node[,] nodes;
        public int gridSizeX, gridSizeY;
        public float nodeSize;
        public Vector3 originPosition;

        public Grid(GameObject plane, float nodeSize)
        {
            this.nodeSize = nodeSize;

            // calculate grid dimensions based on the plane's size
            Vector3 planeScale = plane.transform.localScale;
            float planeWidth = planeScale.x * 10f;
            float planeHeight = planeScale.z * 10f;

            gridSizeX = Mathf.RoundToInt(planeWidth / nodeSize);
            gridSizeY = Mathf.RoundToInt(planeHeight / nodeSize);

            // get the origin position (bottom-left corner of the plane)
            originPosition = plane.transform.position - new Vector3((planeWidth - 0.75f) / 2, 0, (planeHeight - 0.75f) / 2);

            // create nodes
            nodes = new Node[gridSizeX, gridSizeY];
            for (int x = 0; x < gridSizeX; x++)
            {
                for (int y = 0; y < gridSizeY; y++)
                {
                    Vector3 worldPosition = originPosition + new Vector3(x * nodeSize, 0, y * nodeSize);
                    nodes[x, y] = new Node(worldPosition, true, x, y); // mark nodes as walkable initially
                }
            }
        }

        public Node GetNodeFromWorldPosition(Vector3 worldPosition)
        {
            int x = Mathf.Clamp(Mathf.FloorToInt((worldPosition.x - originPosition.x) / nodeSize), 0, gridSizeX - 1);
            int y = Mathf.Clamp(Mathf.FloorToInt((worldPosition.z - originPosition.z) / nodeSize), 0, gridSizeY - 1);
            return nodes[x, y];
        }

        public List<Node> GetNeighbors(Node node)
        {
            List<Node> neighbors = new List<Node>();

            for (int dx = -1; dx <= 1; dx++)
            {
                for (int dy = -1; dy <= 1; dy++)
                {
                    if (dx == 0 && dy == 0) continue; // skip the current node

                    int x = node.gridX + dx;
                    int y = node.gridY + dy;

                    if (x >= 0 && x < gridSizeX && y >= 0 && y < gridSizeY)
                    {
                        neighbors.Add(nodes[x, y]);
                    }
                }
            }

            return neighbors;
        }
    }
}
