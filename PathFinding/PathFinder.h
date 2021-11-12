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
  enum PathFinderStatus
  {
    NOT_INITIALIZED = 0,
    SUCCESS,
    FAILURE,
    RUNNING,
  };

  // The Node class. 
  // It is an abstract class that provides the base class
  // for any type of vertex that you want to implement in
  // your path finding problem.
  template <typename T> 
  class Node
  {
  public:
    // We store a reference to the T as Value.
    T Value;

      // The constructor for the Node class.
    Node(T value)
    {
      Value = value;
    }

    // Get the neighbours for this node. 
    // This is the most important function that 
    // your concrete vertex class should implement.
    virtual std::vector<std::shared_ptr<Node<T>>> GetNeighbours() = 0;
  };

  // The abstract PathFinder class that implements the core
  // pathfinding related codes.
  template <typename T> 
  class PathFinder
  {
  public:

    // Create a delegate that defines the signature
    // for calculating the cost between two 
    // Nodes (T which makes a Node)
    template <typename T> 
    class CostFunction
    {
    public:
      virtual float operator()(const T& a, const T& b) = 0;
    };

    // The PathFinderNode class.
    // This class equates to a node in a the tree generated
    // by the pathfinder in its search for the most optimal
    // path. Do not confuse this with the Node class on top.
    // This class encapsulates a Node and hold other attributes
    // needed for the search traversal.
    // The pathfinder creates instances of this class at runtime
    // while doing the search.
    template <typename T> 
    class PathFinderNode
    {
    public:
      // The parent of this node.
      std::shared_ptr<PathFinderNode> Parent;

      // The Node that this PathFinderNode is pointing to.
      std::shared_ptr<Node<T>> Location;

      // The various costs.
      float Fcost;
      float GCost;
      float Hcost;

      // The constructor.
      // It takes in the Node, the parent, the gvost and the hcost.
      PathFinderNode(std::shared_ptr<Node<T>> location,
        PathFinderNode parent,
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
      bool operator < (const PathFinderNode<T>& pfnode) 
      {
        if (Fcost <= pfnode.Fcost)
        {
          return true;
        }
        return false;
      }
      // overloaded > operator
      bool operator > (const PathFinderNode<T>& pfnode)
      {
        if (Fcost > pfnode.Fcost)
        {
          return true;
        }
        return false;
      }
    };


    // Stage 1. Initialize the serach.
    // Initialize a new search.
    // Note that a search can only be initialized if 
    // the path finder is not already running.
    bool Initialize(std::shared_ptr<Node<T>> start, std::shared_ptr<Node<T>> goal)
    {
      if (Status == PathFinderStatus.RUNNING)
      {
        // Path finding is already in progress.
        return false;
      }

      // Reset the variables.
      Reset();

      // Set the start and the goal nodes for this search.
      Start = start;
      Goal = goal;

      // Calculate the H cost for the start.
      float H = (*HeuristicCost)(Start.Value, Goal.Value);

      // Create a root node with its parent as null.
      PathFinderNode root = new PathFinderNode(Start, 0, 0f, H);

      // add this root node to our open list.
      mOpenList.push(root);

      // set the current node to root node.
      CurrentNode = root;

      //// Invoke the deletages to inform the caller if the delegates are not null.
      //onChangeCurrentNode ? .Invoke(CurrentNode);
      //onStarted ? .Invoke();

      // set the status of the pathfinder to RUNNING.
      Status = PathFinderStatus.RUNNING;

      return true;
    }
    // Stage 2: Step until success or failure
    // Take a search step. The user must continue to call this method 
    // until the Status is either SUCCESS or FAILURE.
    PathFinderStatus Step()
    {
      // Add the current node to the closed list.
      mClosedList.push(CurrentNode);

      //// Call the delegate to inform any subscribers.
      //onAddToClosedList ? .Invoke(CurrentNode);

      if (mOpenList.empty())
      {
        // we have exhausted our search. No solution is found.
        Status = PathFinderStatus.FAILURE;
        //onFailure ? .Invoke();
        return Status;
      }

      // Get the least cost element from the open list.
      // This becomes our new current node. 
      
      // You can use top to access the least cost element.
      // Then remove the top from the list by calling pop.
      CurrentNode = mOpenList.top();
      mOpenList.pop();

      // Check if the node contains the Goal cell.
      if(CurrentNode.Location.Value == Goal.Value)
      {
        Status = PathFinderStatus.SUCCESS;
        return Status;
      }

      // Find the neighbours.
      std::vector<std::shared_ptr<Node<T>>> neighbours = CurrentNode.Location.GetNeighbours();

      // Traverse each of these neighbours for possible expansion.
      for (int i = 0; i < neighbours.size(); ++i)
      {
        AlgorithmSpecificImplementation(*neighbours[i]);
      }
      Status = PathFinderStatus.RUNNING;
      //onRunning ? .Invoke();
      return Status;
    }

  protected:
    virtual void AlgorithmSpecificImplementation(std::shared_ptr<Node<T>> cell) = 0;

    // Reset the internal variables for a new search.
    void Reset()
    {
      if (Status == PathFinderStatus.RUNNING)
      {
        // Cannot reset path finder. Path finding in progress.
        return;
      }

      CurrentNode = 0;

      mOpenList = OpenList();
      mClosedList.clear();

      Status = PathFinderStatus.NOT_INITIALIZED;
    }

  protected:

    // A helper method to check if a value of T is in a list.
    // If it is then return the index of the item where the
    // value is. Otherwise return -1.
    int IsInList(std::vector<PathFinderNode<T>> myList, T cell)
    {
      auto it = std::find_if(
        myList.begin(),
        myList.end(),
        [&cell](const PathFinderNode<T>& obj)
        {
          return obj.Location.Value == cell;
        });

      auto index = -1;
      if (it != myList.end())
      {
        index = std::distance(myList.begin(), it);
      }
      return index;
    }
    int IsInPQ(std::priority_queue<PathFinderNode<T>> myList, T cell)
    {
      auto it = std::find_if(
        myList.begin(),
        myList.end(),
        [&cell](const PathFinderNode<T>& obj)
        {
          return obj.Location.Value == cell;
        });

      auto index = -1;
      if (it != myList.end())
      {
        index = std::distance(myList.begin(), it);
      }
      return index;
    }

    // Add a property that holds the current status of the
    // pathfinder. By default it is set to NOT_INITIALIZED.
    // Also note that we have made the set to private to 
    // ensure that only this class can change and set
    // the status.
    PathFinderStatus Status;// = PathFinderStatus.NOT_INITIALIZED;

    // Add properties for the start and goal nodes.
    std::shared_ptr<Node<T>> Start;
    std::shared_ptr<Node<T>> Goal;

    // The property to access the CurrentNode that the
    // pathfinder is now at.
    std::shared_ptr<PathFinderNode<T>> CurrentNode;// { get; private set; }

    std::shared_ptr<CostFunction<T>> HeuristicCost;
    std::shared_ptr<CostFunction<T>> NodeTraversalCost;

    // The open list for the path finder.
    typedef std::priority_queue<
      std::shared_ptr<PathFinderNode<T>>,
      std::vector<std::shared_ptr<PathFinderNode<T>>>,
      std::greater<std::shared_ptr<PathFinderNode<T>>>> OpenList;

    typedef std::vector<PathFinderNode<T>> ClosedList;
    OpenList mOpenList;
    ClosedList mClosedList;
  };

  // The AstarPathFinder.
  template <typename T> 
  class AStarPathFinder : public PathFinder<T>
  {
  protected:
    void AlgorithmSpecificImplementation(std::shared_ptr<Node<T>> cell) override
    {
      // first of all check if the node is already in the closedlist.
      // if so then we do not need to continue search for this node.
      if (IsInList(this.mClosedList, cell.Value) == -1)
      {
        // The cell does not exist in the closed list.

        // Calculate the cost of the node from its parent.
        // Remember G is the cost from the start till now.
        // So to get G we will get the G cost of the currentNode
        // and add the cost from currentNode to this cell.
        // We can actually implement a function to calculate the cost 
        // between two adjacent cells. 

        float G = this.CurrentNode.GCost + NodeTraversalCost(
          this.CurrentNode.Location.Value, cell.Value);

        float H = HeuristicCost(cell.Value, this.Goal.Value);

        // Check if the cell is already there in the open list.
        int idOList = IsInPQ(this.mOpenList, cell.Value);
        if (idOList == -1)
        {
          // The cell does not exist in the open list.
          // We will add the cell to the open list.

          std::shared_ptr<PathFinder::PathFinderNode<T>> n(new PathFinder::PathFinderNode<T>(cell, this.CurrentNode, G, H));
          this.mOpenList.push(n);
        }
        else
        {
          // if the cell exists in the openlist then check if the G cost 
          // is less than the one already in the list.
          float oldG = this.mOpenList[idOList].GCost;
          if (G < oldG)
          {
            // change the parent and update the cost to the new G
            this.mOpenList[idOList].Parent = this.CurrentNode;
            this.mOpenList[idOList].SetGCost(G);
          }
        }
      }
    }
  };
}
