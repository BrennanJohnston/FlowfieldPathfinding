using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FlowField {
    const float DIAGONAL_COST = 1.4f;
    const float NORMAL_COST   = 1.0F;
    List<List<List<List<NodeData>>>> flowField;
    List<List<bool>> obstructions;
    List<NodeCoordinate> obstructedNodeCoordinates;
    float worldWidth, worldHeight, nodeWidth;
    Vector3 worldPosition;
    Comparer<NodeData> nodeComparer = Comparer<NodeData>.Create((NodeData a, NodeData b) => a.Cost > b.Cost ? 1 : a.Cost < b.Cost ? -1 : 0);
    LayerMask obstructionLayerMask;
    Dictionary<GameObject, List<NodeCoordinate>> obstructionNodeDictionary;
    Vector3 obstructionCheckBoxDimensions;

    struct NodeCoordinate {
        int x, y;
        public NodeCoordinate(int xCoord, int yCoord) {
            x = xCoord;
            y = yCoord;
        }

        public int X {
            get { return x; }
        }

        public int Y {
            get { return y; }
        }
    }

    private class NodeData {
        NodeCoordinate coordinate;
        Vector3 pathVector;
        bool obstructed;
        bool visited;
        float cost;
        NodeData nextPathNode;

        public NodeData(int xCoordinate, int yCoordinate) {
            coordinate = new NodeCoordinate(xCoordinate, yCoordinate);
            pathVector = new Vector3(0, 0, 0);
            obstructed = false;
            visited = false;
            cost = 0.0f;
            nextPathNode = null;
        }

        public int X {
            get { return coordinate.X; }
        }

        public int Y {
            get { return coordinate.Y; }
        }

        public Vector3 PathVector {
            get { return pathVector; }
            set {
                    value.y = 0.0f;
                    pathVector = value.normalized;
                }
        }

        public bool Obstructed {
            get { return obstructed; }
            set { obstructed = value; }
        }

        public bool Visited {
            get { return visited; }
            set { visited = value; }
        }

        public float Cost {
            get { return cost; }
            set { cost = value; }
        }

        public NodeData NextPathNode {
            get { return nextPathNode; }
            set { nextPathNode = value; }
        }

        public int Compare(NodeData a, NodeData b) {
            return a.Cost > b.Cost ? 1 : a.Cost < b.Cost ? -1 : 0;
        }
    }

    public FlowField(Vector3 worldFlowFieldPosition, float gridWorldWidth, float gridWorldHeight, float nodeWorldWidth, LayerMask obstructionMask) {
        int width = Math.Max(1, (int)(gridWorldWidth / nodeWorldWidth));
        int height = Math.Max(1, (int)(gridWorldHeight / nodeWorldWidth));
        flowField = new List<List<List<List<NodeData>>>>();
        obstructions = new List<List<bool>>();
        obstructedNodeCoordinates = new List<NodeCoordinate>();
        obstructionLayerMask = obstructionMask;
        obstructionNodeDictionary = new Dictionary<GameObject, List<NodeCoordinate>>();
        obstructionCheckBoxDimensions = new Vector3(nodeWorldWidth / 2 * 0.75f, 50.0f, nodeWorldWidth / 2 * 0.75f);

        for (int solveX = 0; solveX < width; solveX++) {
            List<List<List<NodeData>>> solveCol = new List<List<List<NodeData>>>();
            List<bool> obstructionCol = new List<bool>();
            flowField.Add(solveCol);
            obstructions.Add(obstructionCol);
            for(int solveY = 0; solveY < height; solveY++) {
                List<List<NodeData>> currentFlowfield = new List<List<NodeData>>();
                solveCol.Add(currentFlowfield);
                obstructionCol.Add(false);
                for (int x = 0; x < width; x++) {
                    List<NodeData> currentCol = new List<NodeData>();
                    List<bool> currentObstructionCol = new List<bool>();
                    currentFlowfield.Add(currentCol);
                    for (int y = 0; y < height; y++) {
                        NodeData node = new NodeData(x, y);
                        node.Cost = float.PositiveInfinity;
                        currentObstructionCol.Add(false);
                        currentCol.Add(node);
                    }
                }
            }
        }

        nodeWidth = nodeWorldWidth;
        worldWidth = width * nodeWidth;
        worldHeight = height * nodeWidth;
        worldPosition = worldFlowFieldPosition;
    }

    public Vector3 GetPathVectorByPosition(Vector3 positionInWorld, Vector3 goalPositionInWorld) {
        NodeCoordinate coordinates = WorldPositionToCoordinate(positionInWorld);
        NodeCoordinate goalCoordinates = WorldPositionToCoordinate(goalPositionInWorld);


        return flowField[goalCoordinates.X][goalCoordinates.Y][coordinates.X][coordinates.Y].PathVector;
    }

    public void SolveForPosition(Vector3 positionInWorld) {

        positionInWorld.x = Mathf.Clamp(positionInWorld.x, worldPosition.x, worldPosition.x + worldWidth);
        positionInWorld.z = Mathf.Clamp(positionInWorld.z, worldPosition.z, worldPosition.z + worldHeight);

        NodeCoordinate coordinates = WorldPositionToCoordinate(positionInWorld);
        SolveForCoordinate(coordinates.X, coordinates.Y);
    }

    public void SolveForCoordinate(int x, int y) {
        x = Math.Clamp(x, 0, flowField.Count);
        y = Math.Clamp(y, 0, flowField[0].Count);

        // Obtain reference to flowfield for this specific x, y pair.
        List<List<NodeData>> fieldData = flowField[x][y];

        // Generate list copy for modification in case of ignoring specific obstructed nodes
        List<List<bool>> modifiedObstructionBools = new List<List<bool>>();
        for(int i = 0; i < obstructions.Count; i++) {
            List<bool> currentCol = obstructions[i];
            modifiedObstructionBools.Add(new List<bool>());
            for(int k = 0; k < currentCol.Count; k++) {
                modifiedObstructionBools[i].Add(currentCol[k]);
            }
        }

        // Initialize all vertices Costs to infinity and Predecessors to null
        for (int w = 0; w < fieldData.Count; w++) {
            List<NodeData> currentCol = fieldData[w];
            for(int h = 0; h < currentCol.Count; h++) {
                NodeData currentNode = currentCol[h];
                currentNode.PathVector = new Vector3(0, 0, 0);
                currentNode.Cost = float.PositiveInfinity;
                currentNode.NextPathNode = null;
                currentNode.Visited = false;
            }
        }

        NodeData destinationNode = fieldData[x][y];
        destinationNode.Cost = 0.0f;

        // Set ignore Collider if destination node is obstructed
        Collider[] collisions = Physics.OverlapBox(CoordinateToWorldPosition(x, y), obstructionCheckBoxDimensions, Quaternion.identity, obstructionLayerMask);
        Collider ignoreCollider = null;
        if(collisions.Length > 0) {
            ignoreCollider = collisions[0];
            bool containsObject = obstructionNodeDictionary.ContainsKey(ignoreCollider.gameObject);
            if(containsObject) {
                List<NodeCoordinate> associatedObstructedNodes = obstructionNodeDictionary[ignoreCollider.gameObject];
                for(int i = 0; i < associatedObstructedNodes.Count; i++) {
                    modifiedObstructionBools[associatedObstructedNodes[i].X][associatedObstructedNodes[i].Y] = false;
                }
            }
        }

        // Create PriorityQueue for unvisited nodes
        //SortedSet<NodeData> unvisitedNodes = new SortedSet<NodeData>(nodeComparer);
        Utils.PriorityQueue<NodeData, NodeData> unvisitedNodes = new Utils.PriorityQueue<NodeData, NodeData>(nodeComparer);

        // Add destination node to PriorityQueue
        if(!modifiedObstructionBools[destinationNode.X][destinationNode.Y])
            unvisitedNodes.Enqueue(destinationNode, destinationNode);

        int visitCount = 0;

        // List of nodes that are touching unobstructed nodes (edge of obstructing objects)
        // Calculating vectors for these differently
        List<NodeCoordinate> obstructedBorderNodes = new List<NodeCoordinate>();

        // Begin Dijkstra loop
        while(unvisitedNodes.Count > 0) {
            // Get unvisited node with the lowest Cost
            NodeData lowestCostUnvisitedNode = unvisitedNodes.Dequeue();
            /*
            bool worked = unvisitedNodes.Remove(lowestCostUnvisitedNode);
            if(!worked) {
                Debug.Log("Had to break");
                Debug.Log("unvisitedNodes.Count: " + unvisitedNodes.Count);
                break;
            }
            */
            lowestCostUnvisitedNode.Visited = true;
            visitCount++;

            Vector3 currentNodeVector = new Vector3(lowestCostUnvisitedNode.X, 0, lowestCostUnvisitedNode.Y);
            //Debug.Log("here 1: " + unvisitedNodes.Count);
            // Get neighbor nodes
            List<NodeData> neighbors = GetNeighborNodes(lowestCostUnvisitedNode.X, lowestCostUnvisitedNode.Y, fieldData);
            foreach (NodeData neighbor in neighbors) {
                //if (neighbor.Visited) continue;

                bool isDiagonal = lowestCostUnvisitedNode.X != neighbor.X && lowestCostUnvisitedNode.Y != neighbor.Y;
                float edgeCost = NORMAL_COST;
                if (isDiagonal) edgeCost = DIAGONAL_COST;
                //Debug.Log("here 3");
                float potentialCost = lowestCostUnvisitedNode.Cost + edgeCost;
                if (potentialCost < neighbor.Cost && !modifiedObstructionBools[neighbor.X][neighbor.Y]) {
                    neighbor.Cost = potentialCost;
                    neighbor.NextPathNode = lowestCostUnvisitedNode;

                    if (!neighbor.Visited) {
                        neighbor.Visited = true;
                        //unvisitedNodes.Add(neighbor);
                        unvisitedNodes.Enqueue(neighbor, neighbor);
                    }

                    // Update vector for this node
                    Vector3 neighborVector = new Vector3(neighbor.X, 0, neighbor.Y);
                    neighbor.PathVector = (currentNodeVector - neighborVector).normalized;
                } else if (!modifiedObstructionBools[lowestCostUnvisitedNode.X][lowestCostUnvisitedNode.Y] && modifiedObstructionBools[neighbor.X][neighbor.Y]) {
                    obstructedBorderNodes.Add(new NodeCoordinate(neighbor.X, neighbor.Y));
                }
            }
        }

        // Set PathVectors for obstructed nodes
        foreach(NodeCoordinate nodeCoordinate in obstructedNodeCoordinates) {
            NodeData obstructedNode = fieldData[nodeCoordinate.X][nodeCoordinate.Y];
            Vector3 nodeWorldPosition = CoordinateToWorldPosition(nodeCoordinate.X, nodeCoordinate.Y);
            nodeWorldPosition.x += nodeWidth / 2;
            nodeWorldPosition.z += nodeWidth / 2;
            Collider[] collidersHit = Physics.OverlapBox(nodeWorldPosition, obstructionCheckBoxDimensions, Quaternion.identity, obstructionLayerMask);
            foreach(Collider collider in collidersHit) {
                if (collider == ignoreCollider) continue;
                Vector3 newPathVector = (nodeWorldPosition - collider.transform.position);
                obstructedNode.PathVector = newPathVector;
                break;
            }
        }

        // Set PathVectors for obstructed nodes that are on the border of unobstructed nodes
        foreach(NodeCoordinate nodeCoordinate in obstructedBorderNodes) {
            NodeData obstructedNode = fieldData[nodeCoordinate.X][nodeCoordinate.Y];
            List<NodeData> neighbors = GetNeighborNodes(obstructedNode.X, obstructedNode.Y, fieldData);
            Vector3 averageVector = new Vector3();
            int vectorCount = 0;
            foreach(NodeData neighbor in neighbors) {
                if (!modifiedObstructionBools[neighbor.X][neighbor.Y]) {
                    averageVector += neighbor.PathVector;
                    vectorCount++;
                }
            }

            averageVector /= vectorCount;
            obstructedNode.PathVector = averageVector;
        }
    }

    static List<NodeData> GetNeighborNodes(int x, int y, List<List<NodeData>> flowField) {
        List<NodeData> neighbors = new List<NodeData>();
        int minX = Mathf.Max(0, x - 1);
        int maxX = Mathf.Min(flowField.Count, x + 2);
        for(int w = minX; w < maxX; w++) {
            List<NodeData> currentCol = flowField[w];
            int minY = Mathf.Max(0, y - 1);
            int maxY = Mathf.Min(currentCol.Count, y + 2);
            for(int h = minY; h < maxY; h++) {
                if (w != x || h != y)
                    neighbors.Add(currentCol[h]);
            }
        }

        return neighbors;
    }

    NodeCoordinate WorldPositionToCoordinate(Vector3 positionInWorld) {
        positionInWorld -= worldPosition;
        int x = (int)Mathf.Clamp((positionInWorld.x / nodeWidth), 0, flowField.Count - 1);
        int y = (int)Mathf.Clamp((positionInWorld.z / nodeWidth), 0, flowField[0].Count - 1);

        return new NodeCoordinate(x, y);
    }

    Vector3 CoordinateToWorldPosition(int coordX, int coordY) {
        Vector3 position = new Vector3(coordX * nodeWidth, 0.0f, coordY * nodeWidth);
        position += worldPosition;
        return position;
    }

    public void DetectObstructions() {
        float nodeWidthHalf = nodeWidth / 2.0f;
        obstructedNodeCoordinates.Clear();
        for(int x = 0; x < obstructions.Count; x++) {
            for(int y = 0; y < obstructions[x].Count; y++) {
                Vector3 worldPos = CoordinateToWorldPosition(x, y);
                worldPos.x += nodeWidthHalf;
                worldPos.z += nodeWidthHalf;

                Collider[] collisions = Physics.OverlapBox(worldPos, obstructionCheckBoxDimensions, Quaternion.identity, obstructionLayerMask);
                obstructions[x][y] = collisions.Length > 0;
                if (obstructions[x][y]) {
                    foreach (Collider collider in collisions) {
                        GameObject colliderGameObject = collider.gameObject;
                        bool containsGameObject = obstructionNodeDictionary.ContainsKey(colliderGameObject);
                        if(!containsGameObject) {
                            obstructionNodeDictionary.Add(colliderGameObject, new List<NodeCoordinate>());
                        }

                        obstructionNodeDictionary[colliderGameObject].Add(new NodeCoordinate(x, y));
                    }

                    obstructedNodeCoordinates.Add(new NodeCoordinate(x, y));
                    Debug.DrawLine(worldPos, worldPos + new Vector3(nodeWidthHalf * 0.5f, 0.0f, nodeWidthHalf * 0.5f), Color.red, 2000.0f);
                    Debug.DrawLine(worldPos, worldPos + new Vector3(nodeWidthHalf * 0.5f, 0.0f, -nodeWidthHalf * 0.5f), Color.red, 2000.0f);
                    Debug.DrawLine(worldPos, worldPos + new Vector3(-nodeWidthHalf * 0.5f, 0.0f, -nodeWidthHalf * 0.5f), Color.red, 2000.0f);
                    Debug.DrawLine(worldPos, worldPos + new Vector3(-nodeWidthHalf * 0.5f, 0.0f, nodeWidthHalf * 0.5f), Color.red, 2000.0f);
                }
            }
        }
    }

    public void DrawField(Vector3 goalPosition, float duration) {
        NodeCoordinate coordinates = WorldPositionToCoordinate(goalPosition);
        List<List<NodeData>> fieldData = flowField[coordinates.X][coordinates.Y];
        float nodeWidthHalf = nodeWidth / 2.0f;

        for(int w = 0; w < fieldData.Count; w++) {
            List<NodeData> currentCol = fieldData[w];
            for(int h = 0; h < currentCol.Count; h++) {
                NodeData currentNode = currentCol[h];
                Vector3 startPos = new Vector3(w * nodeWidth + nodeWidthHalf, 0.0f, h * nodeWidth + nodeWidthHalf) + worldPosition;
                Vector3 flowVector = currentNode.PathVector * nodeWidthHalf;
                Debug.DrawLine(startPos, startPos + flowVector, Color.white, duration);
            }
        }
    }
}