#pragma once

#include <functional>
#include <queue>
#include <vector>
#include <iostream>
#include <memory>

namespace PathFinding
{
  // An enumeration type to represent the status of the 
  // pathfinder at any given time.
  enum class PathFinderStatus
  {
    NOT_INITIALIZED = 0,
    SUCCESS,
    FAILURE,
    RUNNING,
  };

  // The abstract PathFinder class that implements the core
  // pathfinding related codes.
  class PathFinder
  {
  public:  // The Node class. 
  // It is an abstract class that provides the base class
  // for any type of vertex that you want to implement in
  // your path finding problem.
    class Node
    {
    public:
      Node() {}

      virtual bool operator==(const Node& other) const = 0;
      /// <summary>
      /// The Heuristic cost between this node to another node.
      /// The derived class needs to implement this function
      /// based on the type of map used.
      /// </summary>
      /// <param name="other">The other node</param>
      /// <returns></returns>
      virtual float GetHeuristicCost(const Node& other) const = 0;
      /// <summary>
      /// The Node Traversal cost between this node to another node.
      /// The derived class needs to implement this function
      /// based on the type of map used.
      /// </summary>
      /// <param name="other"></param>
      /// <returns></returns>
      virtual float GetNodeTraversalCost(const Node& other) const = 0;

      // Get the neighbours for this node. 
      // This is the most important function that 
      // your concrete vertex class should implement.
      virtual std::vector<const Node*> GetNeighbours() const = 0;

      virtual ~Node() {}
    };

  protected:
    // The PathFinderNode class.
    // This class equates to a node in a the tree generated
    // by the pathfinder in its search for the most optimal
    // path. Do not confuse this with the Node class on top.
    // This class encapsulates a Node and hold other attributes
    // needed for the search traversal.
    // The pathfinder creates instances of this class at runtime
    // while doing the search.
    class PathFinderNode
    {
    public:
      // The parent of this node.
      const PathFinderNode* Parent;

      // The Node that this PathFinderNode is pointing to.
      const Node* Location;

      // The various costs.
      float Fcost;
      float GCost;
      float Hcost;

      // The constructor.
      // It takes in the Node, the parent, the gvost and the hcost.
      PathFinderNode(
        const Node* location,
        const PathFinderNode* parent,
        float gCost,
        float hCost)
      {
        Location = location;
        Parent = parent;
        Hcost = hCost;
        SetGCost(gCost);
      }

      // Set the gcost. 
      void SetGCost(float c)
      {
        GCost = c;
        Fcost = GCost + Hcost;
      }

      // overloaded < operator
      bool operator < (const PathFinderNode& pfnode) const
      {
        if (Fcost <= pfnode.Fcost)
        {
          return true;
        }
        return false;
      }
      // overloaded > operator
      bool operator > (const PathFinderNode& pfnode) const
      {
        if (Fcost > pfnode.Fcost)
        {
          return true;
        }
        return false;
      }
    };
  protected:
    // Add a property that holds the current status of the
    // pathfinder. By default it is set to NOT_INITIALIZED.
    // Also note that we have made the set to private to 
    // ensure that only this class can change and set
    // the status.
    PathFinderStatus mStatus;// = PathFinderStatus.NOT_INITIALIZED;

    // Add properties for the start and goal nodes.
    const Node* mStart;
    const Node* mGoal;

    std::shared_ptr<PathFinderNode> mCurrentNode;

    typedef std::vector<std::shared_ptr<PathFinderNode>> ClosedList;
    typedef std::vector<std::shared_ptr<PathFinderNode>> OpenList;

    OpenList mOpenList;
    ClosedList mClosedList;
  public:

    inline PathFinderStatus GetStatus() const
    {
      return mStatus;
    }

    // Stage 1. Initialize the serach.
    // Initialize a new search.
    // Note that a search can only be initialized if 
    // the path finder is not already running.
    bool Initialize(const Node* start, const Node* goal)
    {
      if (mStatus == PathFinderStatus::RUNNING)
      {
        // Path finding is already in progress.
        return false;
      }
      // Reset the variables.
      Reset();

      // Set the start and the goal nodes for this search.
      mStart = start;
      mGoal = goal;

      // Calculate the H cost for the start.
      float H = mStart->GetHeuristicCost(*mGoal);

      // Create a root node with its parent as null.
      mCurrentNode = std::make_shared<PathFinderNode>(mStart, static_cast<const PathFinderNode*>(0), 0.0f, H);

      // add this root node to our open list.
      mOpenList.push_back(mCurrentNode);

      // set the status of the pathfinder to RUNNING.
      mStatus = PathFinderStatus::RUNNING;

      return true;
    }

    // Stage 2: Step until success or failure
    // Take a search step. The user must continue to call this method 
    // until the mStatus is either SUCCESS or FAILURE.
    PathFinderStatus Step()
    {
      // Add the current node to the closed list.
      mClosedList.push_back(mCurrentNode);

      //// Call the delegate to inform any subscribers.
      //onAddToClosedList ? .Invoke(mCurrentNode);

      if (mOpenList.empty())
      {
        // we have exhausted our search. No solution is found.
        mStatus = PathFinderStatus::FAILURE;
        return mStatus;
      }

      int least_cost_index = GetLeastCostNodeIndex(mOpenList);

      mCurrentNode = mOpenList[least_cost_index];
      mOpenList.erase(mOpenList.begin() + least_cost_index);

      // Check if the node contains the mGoal cell.
      if (*mCurrentNode->Location == *mGoal)
      {
        mStatus = PathFinderStatus::SUCCESS;
        return mStatus;
      }

      // Find the neighbours.
      std::vector<const Node*> neighbours = mCurrentNode->Location->GetNeighbours();

      // Traverse each of these neighbours for possible expansion.
      for (int i = 0; i < neighbours.size(); ++i)
      {
        AlgorithmSpecificImplementation(neighbours[i]);
      }
      mStatus = PathFinderStatus::RUNNING;
      //onRunning ? .Invoke();
      return mStatus;
    }

    // Get the path if a path is found.
    // Returns the array of nodes from goal to start.
    std::vector<const Node*> GetReversePath() const
    {
      std::vector<const Node*> path;
      if (mStatus == PathFinderStatus::SUCCESS)
      {
        const PathFinderNode* currNode = mCurrentNode.get();
        while (currNode != 0)
        {
          path.push_back(currNode->Location);
          currNode = currNode->Parent;
        }
      }
      return path;
    }

  protected:
    virtual void AlgorithmSpecificImplementation(const Node* cell) = 0;

    // Reset the internal variables for a new search.
    void Reset()
    {
      if (mStatus == PathFinderStatus::RUNNING)
      {
        // Cannot reset path finder. Path finding in progress.
        return;
      }

      mCurrentNode = 0;

      mOpenList = OpenList();
      mClosedList.clear();

      mStatus = PathFinderStatus::NOT_INITIALIZED;
    }

  protected:
    
    // A helper method to find the least cost node from a std::vector
    int GetLeastCostNodeIndex(const std::vector<std::shared_ptr<PathFinderNode>>& myList) const
    {
      int best_index = 0;
      float best_priority = (*myList[0]).Fcost;
      for (int i = 1; i < myList.size(); i++)
      {
        if (best_priority > (*myList[i]).Fcost)
        {
          best_priority = (*myList[i]).Fcost;
          best_index = i;
        }
      }

      return best_index;
    }

    // A helper method to check if a value of T is in a list.
    // If it is then return the index of the item where the
    // value is. Otherwise return -1.
    int IsInList(const std::vector<std::shared_ptr<PathFinderNode>>& myList, const Node& cell) const
    {
      auto it = std::find_if(
        myList.begin(),
        myList.end(),
        [&cell](const std::shared_ptr<PathFinderNode>& obj)
        {
          return *obj->Location == cell;
        });

      auto index = -1;
      if (it != myList.end())
      {
        index = static_cast<int>(std::distance(myList.begin(), it));
      }
      return index;
    }
  };

  // The AstarPathFinder.
  class AStarPathFinder : public PathFinder
  {
  protected:
    void AlgorithmSpecificImplementation(const Node* cell) override
    {
      // first of all check if the node is already in the closedlist.
      // if so then we do not need to continue search for this node.
      if (IsInList(mClosedList, (*cell)) == -1)
      {
        // The cell does not exist in the closed list.

        // Calculate the cost of the node from its parent.
        // Remember G is the cost from the start till now.
        // So to get G we will get the G cost of the currentNode
        // and add the cost from currentNode to this cell.
        // We can actually implement a function to calculate the cost 
        // between two adjacent cells. 

        float G =
          (*mCurrentNode).GCost +
          mCurrentNode->Location->GetNodeTraversalCost(*cell);

        float H = cell->GetHeuristicCost(*mGoal);

        // Check if the cell is already there in the open list.
        int idOList = IsInList(mOpenList, (*cell));
        if (idOList == -1)
        {
          // The cell does not exist in the open list.
          // We will add the cell to the open list.
          auto n = std::make_shared<PathFinderNode>(cell, mCurrentNode.get(), G, H);
          mOpenList.push_back(n);
        }
        else
        {
          // if the cell exists in the openlist then check if the G cost 
          // is less than the one already in the list.
          float oldG = mOpenList[idOList]->GCost;
          if (G < oldG)
          {
            // change the parent and update the cost to the new G
            mOpenList[idOList]->Parent = mCurrentNode.get();
            mOpenList[idOList]->SetGCost(G);
          }
        }
      }
    }
  };
}
