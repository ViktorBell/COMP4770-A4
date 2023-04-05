using System.Collections.Generic;
using System.Linq;
using EasyAI.Navigation.Nodes;
using UnityEngine;


namespace EasyAI.Navigation
{
    /// <summary>
    /// A* pathfinding.
    /// </summary>
    public static class AStar
    {
        /// <summary>
        /// Perform A* pathfinding.
        /// </summary>
        /// <param name="current">The starting position.</param>
        /// <param name="goal">The end goal position.</param>
        /// <param name="connections">All node connections in the scene.</param>
        /// <returns>The path of nodes to take to get from the starting position to the ending position.</returns>
        public static List<Vector3> Perform(Vector3 current, Vector3 goal, List<Connection> connections)
        {
            // TODO - Assignment 4 - Implement A* pathfinding.

            List<Vector3> pathToTake = new();

            AStarNode bestNode = null;
            // Instantiates a list of Astar nodes which will hold all of the potential successors of the current node (at this point the starting node)
            List<AStarNode> successorNodes = new() { new(current, goal) };

            // selectedNode is set to the start position and it is added to the open list
            // AStarNode selectedNode = new AStarNode(current, goal, null);


            // Adds the starting node to the successor node list
            //successorNodes.Add(selectedNode);

            // Loop as long as there are open nodes.
            while (successorNodes.Any(n => n.IsOpen))
            {
                AStarNode selectedNode = successorNodes.Where(n => n.IsOpen).OrderBy(n => n.CostF).ThenBy(n => n.CostH).First();

                // Clost the current node.
                selectedNode.Close();


                if (bestNode == null || selectedNode.CostF < bestNode.CostF)
                {
                    bestNode = selectedNode;
                }


                // Loops through every connection in the nodesConnectedToCurrent list and populates the successorNodes list with Astar nodes also generates the implicit open list
                foreach (Connection nodeConnection in connections.Where(c => c.A == selectedNode.Position || c.B == selectedNode.Position))
                {
                    // Get the other position in the connection so we do not work with the exact same node again and get stuck.
                    Vector3 position = nodeConnection.A == selectedNode.Position ? nodeConnection.B : nodeConnection.A;

                    /*
                    Vector3 position;
                    if (nodeConnnection.A == current)
                    {
                        position = nodeConnnection.B;

                    }
                    else
                    {
                        position = nodeConnnection.A;
                    }
                    */

                    // Create a new node to compare against.
                    AStarNode successor = new(position, goal, selectedNode);

                    // If this node is equal to the goal then set it and clear the list, allows for changes in goal while travelling to a destination without the program crashing
                    if (position == goal)
                    {
                        bestNode = successor;
                        successorNodes.Clear();
                        break;
                    }


                    //AStarNode duplicateNode = successorNodes.Any(node => node.Position == position)

                    // See if we have a node for this position as we can only have one node per position.
                    AStarNode existing = successorNodes.FirstOrDefault(n => n.Position == position);
                    // If no new node, easy, just add it.
                    if (!successorNodes.Any(node => node.Position == position)) //!successorNodes.Any(node => node.Position == position)
                    {
                        successorNodes.Add(successor);
                        continue;
                    }

                    // If it did already exist and the new path has a shorter cost, replace it and make sure it is open to revisit. 
                    if (existing.CostF <= successor.CostF)
                    {
                        continue;
                    }
                    existing.UpdatePrevious(selectedNode);
                    existing.Open();
                }


            }

            if (bestNode == null)
            {
                return new() { current, goal };
            }

            
            // Starting from the end Node (goal position) the path is 'walked back along' by the loop until there is no longer a pervious node meaning that we have reached the start of the path
            while (bestNode != null)
            {
                pathToTake.Add(bestNode.Position);

                // Set selected node to previous node
                bestNode = bestNode.Previous;

            }

            // Finally adds the starting node to the end of the pathToTake
            //pathToTake.Add(bestNode.Position);

            // Orders the path to take from Start position to End Position
            pathToTake.Reverse();

            return pathToTake;
        }
    }
}