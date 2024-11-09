using UnityEngine;
using System.Collections.Generic;

public class AStarPathfinder
{
    private class Node
    {
        public Vector3 position;
        public Node parent;
        public float gCost; // Cost from start to this node
        public float hCost; // Heuristic cost from this node to the target
        public float fCost => gCost + hCost; // Total cost

        public Node(Vector3 position, Node parent, float gCost, float hCost)
        {
            this.position = position;
            this.parent = parent;
            this.gCost = gCost;
            this.hCost = hCost;
        }
    }

    public List<Vector3> FindPath(Vector3 start, Vector3 target, LayerMask obstacleLayer, float gridResolution)
    {
        // Open and closed lists
        List<Node> openList = new List<Node>();
        HashSet<Vector3> closedSet = new HashSet<Vector3>();

        Node startNode = new Node(start, null, 0, Vector3.Distance(start, target));
        openList.Add(startNode);

        while (openList.Count > 0)
        {
            // Get the node with the lowest fCost
            Node currentNode = GetLowestFCostNode(openList);

            // If we've reached the target
            if (Vector3.Distance(currentNode.position, target) < gridResolution)
            {
                return RetracePath(currentNode); // Retrace path from end to start
            }

            openList.Remove(currentNode);
            closedSet.Add(currentNode.position);

            // Check each neighbor of the current node
            foreach (Vector3 neighborPos in GetNeighbors(currentNode.position, gridResolution))
            {
                if (closedSet.Contains(neighborPos) || Physics.CheckSphere(neighborPos, gridResolution / 2, obstacleLayer))
                {
                    // Ignore neighbors in the closed set or with obstacles
                    continue;
                }

                float newGCost = currentNode.gCost + Vector3.Distance(currentNode.position, neighborPos);
                Node neighborNode = openList.Find(node => node.position == neighborPos);

                if (neighborNode == null)
                {
                    // Add new neighbor node to open list
                    neighborNode = new Node(neighborPos, currentNode, newGCost, Vector3.Distance(neighborPos, target));
                    openList.Add(neighborNode);
                }
                else if (newGCost < neighborNode.gCost)
                {
                    // Update neighbor with a better path
                    neighborNode.gCost = newGCost;
                    neighborNode.parent = currentNode;
                }
            }
        }

        return new List<Vector3>(); // No path found
    }

    private Node GetLowestFCostNode(List<Node> nodes)
    {
        Node lowestFCostNode = nodes[0];
        foreach (Node node in nodes)
        {
            if (node.fCost < lowestFCostNode.fCost)
            {
                lowestFCostNode = node;
            }
        }
        return lowestFCostNode;
    }

    private List<Vector3> GetNeighbors(Vector3 position, float gridResolution)
    {
        List<Vector3> neighbors = new List<Vector3>();

        // Define possible neighbor positions (up, down, left, right, front, back in 3D space)
        Vector3[] directions = {
            Vector3.forward, Vector3.back, Vector3.right, Vector3.left,
            Vector3.up, Vector3.down
        };

        foreach (Vector3 direction in directions)
        {
            Vector3 neighborPos = position + direction * gridResolution;
            neighbors.Add(neighborPos);
        }

        return neighbors;
    }

    private List<Vector3> RetracePath(Node endNode)
    {
        List<Vector3> path = new List<Vector3>();
        Node currentNode = endNode;

        while (currentNode != null)
        {
            path.Add(currentNode.position);
            currentNode = currentNode.parent;
        }

        path.Reverse(); // Reverse the path to start from the beginning
        return path;
    }
}
