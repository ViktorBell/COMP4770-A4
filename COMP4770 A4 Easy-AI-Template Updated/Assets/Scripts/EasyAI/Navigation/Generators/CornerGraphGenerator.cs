﻿using EasyAI.Navigation.Nodes;
using UnityEngine;

namespace EasyAI.Navigation.Generators
{
    /// <summary>
    /// Convex corner graph placement for nodes.
    /// </summary>
    public class CornerGraphGenerator : NodeGenerator
    {
        [SerializeField]
        [Min(0)]
        [Tooltip("How far away from corners should the nodes be placed.")]
        private int cornerNodeSteps = 3;
    
        /// <summary>
        /// Place nodes at convex corners.
        /// </summary>
        public override void Generate()
        {
            // Check all X coordinates, skipping the padding required.
            for (int x = cornerNodeSteps * 2; x < NodeArea.RangeX - cornerNodeSteps * 2; x++)
            {
                // Check all Z coordinates, skipping the padding required.
                for (int z = cornerNodeSteps * 2; z < NodeArea.RangeZ - cornerNodeSteps * 2; z++)
                {
                    // If this space is open it cannot be a corner so continue.
                    if (NodeArea.IsOpen(x, z))
                    {
                        continue;
                    }
                
                    // Otherwise it could be a corner so check in all directions.
                    UpperUpper(x, z);
                    UpperLower(x, z);
                    LowerUpper(x, z);
                    LowerLower(x, z);
                }
            }
        }

        /// <summary>
        /// Check coordinates for an open corner to place a node in the positive X and positive Z directions.
        /// </summary>
        /// <param name="x">The X coordinate of the potential corner.</param>
        /// <param name="z">The Z coordinate of the potential corner.</param>
        private void UpperUpper(int x, int z)
        {
            // Ensure green square as seen in the sldies of the size of corner node steps is open and place node.
            // Done in the positive X and positive Z direction.
           
            // If the adjacent X or Z nodes are not open, return as it is not a convex corner.
            if (!NodeArea.IsOpen(x + 1, z) || !NodeArea.IsOpen(x, z + 1))
            {
                return;
            }

            // Loop through all X coordinates to check the required space to place a node.
            for (int x1 = x + 1; x1 <= x + 1 + cornerNodeSteps * 2; x1++)
            {
                // Loop through all Z coordinates to check the required space to place a node.
                for (int z1 = z + 1; z1 <= z + 1 + cornerNodeSteps * 2; z1++)
                {
                    // If the node is not open return as there is no enough space to place the node.
                    if (!NodeArea.IsOpen(x1, z1))
                    {
                        return;
                    }
                }
            }

            // Place the node at the given offset from the convex corner.
            NodeArea.AddNode(x + 1 + cornerNodeSteps, z + 1 + cornerNodeSteps);

        }
        
        private void UpperLower(int x, int z)
        {
            // Ensure green square as seen in the sldies of the size of corner node steps is open and place node.
            // Done in the positive X and negative Z direction.


            // If the adjacent X or Z nodes are not open, return as it is not a convex corner.
            if (!NodeArea.IsOpen(x + 1, z) || !NodeArea.IsOpen(x, z - 1))
            {
                return;
            }

            // Loop through all X coordinates to check the required space to place a node.
            for (int x1 = x + 1; x1 <= x + 1 + cornerNodeSteps * 2; x1++)
            {
                // Loop through all Z coordinates to check the required space to place a node.
                for (int z1 = z - 1; z1 >= z - 1 - cornerNodeSteps * 2; z1--)
                {
                    // If the node is not open return as there is no enough space to place the node.
                    if (!NodeArea.IsOpen(x1, z1))
                    {
                        return;
                    }
                }
            }

            // Place the node at the given offset from the convex corner.
            NodeArea.AddNode(x + 1 + cornerNodeSteps, z - 1 - cornerNodeSteps);

        }


        private void LowerUpper(int x, int z)
        {
            // Ensure green square as seen in the sldies of the size of corner node steps is open and place node.
            // Done in the negative X and upper Z direction.


            // If the adjacent X or Z nodes are not open, return as it is not a convex corner.
            if (!NodeArea.IsOpen(x - 1, z) || !NodeArea.IsOpen(x, z + 1))
            {
                return;
            }

            // Loop through all X coordinates to check the required space to place a node.
            for (int x1 = x - 1; x1 >= x - 1 - cornerNodeSteps * 2; x1--)
            {
                // Loop through all Z coordinates to check the required space to place a node.
                for (int z1 = z + 1; z1 <= z + 1 + cornerNodeSteps * 2; z1++)
                {
                    // If the node is not open return as there is no enough space to place the node.
                    if (!NodeArea.IsOpen(x1, z1))
                    {
                        return;
                    }
                }
            }

            // Place the node at the given offset from the convex corner.
            NodeArea.AddNode(x - 1 - cornerNodeSteps, z + 1 + cornerNodeSteps);
        }





        private void LowerLower(int x, int z)
        {
            // Ensure green square as seen in the sldies of the size of corner node steps is open and place node.
            // Done in the negative X and negative Z direction.


            // If the adjacent X or Z nodes are not open, return as it is not a convex corner.
            if (!NodeArea.IsOpen(x - 1, z) || !NodeArea.IsOpen(x, z - 1))
            {
                return;
            }

            // Loop through all X coordinates to check the required space to place a node.
            for (int x1 = x - 1; x1 >= x - 1 - cornerNodeSteps * 2; x1--)
            {
                // Loop through all Z coordinates to check the required space to place a node.
                for (int z1 = z - 1; z1 >= z - 1 - cornerNodeSteps * 2; z1--)
                {
                    // If the node is not open return as there is no enough space to place the node.
                    if (!NodeArea.IsOpen(x1, z1))
                    {
                        return;
                    }
                }
            }

            // Place the node at the given offset from the convex corner.
            NodeArea.AddNode(x - 1 - cornerNodeSteps, z - 1 - cornerNodeSteps);

        }

    }
       

}
        
        
    
