// code by jl
package ch.ethz.idsc.owly.glc.core;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.PriorityQueue;

import ch.ethz.idsc.owly.data.tree.Nodes;
import ch.ethz.idsc.owly.glc.adapter.GoalTrajectoryRegionQuery;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.Tensor;

public abstract class AbstractAnyTrajectoryPlanner extends AbstractTrajectoryPlanner implements AnyPlannerInterface {
  protected AbstractAnyTrajectoryPlanner( //
      Tensor eta, //
      StateIntegrator stateIntegrator, //
      TrajectoryRegionQuery obstacleQuery, //
      GoalInterface goalInterface //
  ) {
    super(eta, stateIntegrator, obstacleQuery, goalInterface);
  }

  /** Includes all the functionality of the RootSwitch
   * (deleting of the useless nodes and relabling of modified Domains)
   * @param state the new Rootstate
   * @return The value,by which the depth limit needs to be increased as of the RootSwitch */
  @Override
  public final int switchRootToState(Tensor state) {
    GlcNode newRoot = this.getNode(convertToKey(state));
    int increaseDepthBy = 0;
    // TODO not nice, as we jump from state to startnode
    if (newRoot != null) {
      increaseDepthBy = switchRootToNode(newRoot);
    } else {
      System.err.println("***RESET***");
      System.out.println("This domain  is not labelled yet:");
      System.out.println(state);
      if (!domainMap().isEmpty()) {
        this.deleteSubtreeOf(getRoot());
        this.domainMap().clear();
        this.queue().clear();
      }
      this.insertRoot(state);
    }
    return increaseDepthBy;
  }

  /** Includes all the functionality of the RootSwitch
   * (deleting of the useless nodes and relabling of modified Domains)
   * @param newRoot Node to Switch
   * @return The value,by which the depth limit needs to be increased as of the RootSwitch */
  // public abstract int switchRootToNode(GlcNode newRoot);
  protected final void insertNodeInTree(GlcNode parent, GlcNode node) {
    parent.insertEdgeTo(node);
    final Tensor domainKey = convertToKey(node.state());
    final boolean replaced = insert(domainKey, node);
    if (replaced) {
      System.err.println("No formerLabel existed, but sth. was replaced");
      throw new RuntimeException();
    }
  }

  /** @param baseNode Node, of which all children and itself should be deleted
   * @return the Collection of Nodes, which should be deleted */
  protected final Collection<GlcNode> deleteSubtreeOf(GlcNode baseNode) {
    Collection<GlcNode> deleteTreeCollection = Nodes.ofSubtree(baseNode);
    // -- GOAL: goalNode deleted?
    {
      Optional<GlcNode> optional = getBest();
      if (optional.isPresent())
        if (deleteTreeCollection.contains(optional.get()))
          setBestNull();
    }
    // -- QUEUE: Deleting Nodes from Queue
    queue().removeAll(deleteTreeCollection);
    // -- DOMAINMAP: Removing Nodes (DeleteTree) from DomainMap
    domainMap().values().removeAll(deleteTreeCollection);
    // -- EDGE: Removing Edges between Nodes in DeleteTree
    // TODO edge removal of all nodes needed?
    // Minimum needed:
    // baseRoot.parent().removeEdgeTo(baseRoot);
    // oldRoot has no parent, therefore is skipped
    deleteTreeCollection.remove(baseNode);
    // TODO make parralel? If parralel, run in below exceptions
    deleteTreeCollection.stream().forEach(tempNode -> tempNode.parent().removeEdgeTo(tempNode));
    deleteTreeCollection.add(baseNode);
    if (!baseNode.isLeaf())
      throw new RuntimeException();
    return deleteTreeCollection;
  }

  /** Changes the Goal of the current planner:
   * rechecks the tree if expanding is needed, updates Merit of Nodes in Queue
   * @param newCostFunction modified Costfunction for heuristic
   * @param newGoal New GoalRegion
   * @return boolean, true if Goal was already found in oldTree */
  @Override
  public boolean changeToGoal(final GoalInterface newGoal) {
    this.goalInterface = newGoal;
    GlcNode root = getRoot();
    Collection<GlcNode> treeCollection = Nodes.ofSubtree(root);
    Collection<GlcNode> compareCollection = new ArrayList<>();
    compareCollection.addAll(treeCollection); // TODO: Copied or just referernced
    setBestNull();
    // -- TREE
    // Updating the merit of the entire tree
    long tic = System.nanoTime();
    // Changing the Merit in Queue for each Node
    // TODO JAN: Does this make !treeCollection.equals(compareCollection)==true? if values are changed in treeCollection?
    treeCollection.stream().parallel() //
        .forEach(glcNode -> glcNode.setMinCostToGoal(newGoal.minCostToGoal(glcNode.state())));
    if (false) {
      // if (!treeCollection.equals(compareCollection)) {// TODO JONAS smart way to check if before line modified sth.
      System.err.println("checking for domainlabel changes due to heuristic change,  Treesize: " + treeCollection.size());
      // TODO JONAS for optimiality if Heuristic was changed, check candidates in domains
      RelabelingDomains();
    }
    treeCollection = Nodes.ofSubtree(root);
    // RESORTING OF LIST
    List<GlcNode> list = new LinkedList<>(queue());
    queue().clear();
    queue().addAll(list);
    long toc = System.nanoTime();
    System.out.println("Updated Merit of Tree with " + treeCollection.size() + " nodes in: " //
        + ((toc - tic) * 1e-9) + "s");
    // --
    // -- GOALCHECK TREE
    tic = System.nanoTime();
    // TODO JAN: can parallelize?
    Iterator<GlcNode> treeCollectionIterator = treeCollection.iterator();
    while (treeCollectionIterator.hasNext()) { // goes through entire tree
      GlcNode current = treeCollectionIterator.next();
      if (current.parent() != null) { // checking nodes backwards
        final List<StateTime> nodeTrajectory = //
            getStateIntegrator().trajectory(current.parent().stateTime(), current.flow());
        if (!newGoal.isDisjoint(nodeTrajectory)) // current Node in Goal
          offerDestination(current); // overwrites worse Goal, but does not stop
      }
    }
    // treeCollection.stream().parallel().filter(node->!newGoal.is))
    toc = System.nanoTime();
    System.out.println("Checked current tree for goal in "//
        + (toc - tic) * 1e-9 + "s");
    if (getBest().isPresent()) {
      System.out.println("New Goal was found in current tree --> No new search needed");
      return true;
    }
    System.out.println("**Goalswitch finished**");
    return false;
  }

  abstract void RelabelingDomains();

  /** Finds the rootNode, by following the parents
   * from a random root Node in the tree/DomainMap
   * @return rootNode, which was found from random GlcNode in the tree */
  /* package */ final GlcNode getRoot() {
    Iterator<GlcNode> node = domainMap().values().iterator();
    if (node.hasNext())
      return Nodes.rootFrom(node.next());
    return null;
    // if domainmap empty: no tree exists
  }

  @Override
  public List<StateTime> trajectoryToBest() {
    Optional<GlcNode> tempBest = getBestOrElsePeek();
    if (tempBest.isPresent())
      return GlcNodes.getPathFromRootTo(tempBest.get());
    return null;
  }

  @Override
  public List<TrajectorySample> detailedTrajectoryToBest() {
    Optional<GlcNode> optional = getBestOrElsePeek();
    if (optional.isPresent())
      return GlcTrajectories.connect(getStateIntegrator(), Nodes.listFromRoot(getBest().get()));
    return null;
  }

  // TODO JONAS: Smarter way to get furthest Node?
  // maybe in a package: return StateTime and EndNode
  /** Looks for the Node, which is the furthest in the GoalRegion,
   * @return node with highest merit in GoalRegion */
  @Override
  public Optional<StateTime> getFurthestGoalState() {
    final TrajectoryRegionQuery trq = this.getGoalQuery();
    Optional<StateTime> furthest = Optional.ofNullable(null);
    PriorityQueue<GlcNode> queue = new PriorityQueue<>(Collections.reverseOrder(NodeMeritComparator.INSTANCE)); // highest merit first
    List<StateTime> listStateTimeInGoal = new ArrayList<>();
    if (trq instanceof GoalTrajectoryRegionQuery) {
      final GoalTrajectoryRegionQuery tempGtrq = new GoalTrajectoryRegionQuery((GoalTrajectoryRegionQuery) trq);
      listStateTimeInGoal.addAll(tempGtrq.getAllDiscoveredMembersStateTimeInGoal()); // getting ST of EndNodes
      for (StateTime stateTimeInGoal : listStateTimeInGoal) {
        StateTime endNodeStateTime = tempGtrq.getEndNode(stateTimeInGoal);
        Tensor domainKey = convertToKey(endNodeStateTime.x());
        Optional<GlcNode> endNode = Optional.ofNullable(getNode(domainKey)); // getting EndNodes
        if (endNode.isPresent()) {
          // TODO JONAS find individuel Cost for each StateTime
          // comparing Cost of GoalStates with EndNodeCost
          queue.add(endNode.get());
        }
      }
      GlcNode endNode = null;
      if (!queue.isEmpty()) {
        endNode = queue.element();
        GlcNode parent = endNode.parent();
        final List<StateTime> trajectoryThroughGoal = //
            this.getStateIntegrator().trajectory(parent.stateTime(), endNode.flow());
        final int index = this.getGoalQuery().firstMember(trajectoryThroughGoal);
        if (index == -1)
          throw new RuntimeException(); // Trajectory through Goal should find firstMember
        furthest = Optional.ofNullable(trajectoryThroughGoal.get(index));
      }
    }
    return furthest;
  }

  /** Recieved the furthest Node, where the coming trajectory was in Goal, similar to getBest()
   * @return furthest Node in Goal (highest merit, but in Goal) */
  @Override
  public Optional<GlcNode> getFurthestGoalNode() {
    final TrajectoryRegionQuery trq = this.getGoalQuery();
    final Optional<StateTime> furthestState = getFurthestGoalState();
    Optional<GlcNode> furthestNode = Optional.empty();
    if (trq instanceof GoalTrajectoryRegionQuery && furthestState.isPresent()) {
      final GoalTrajectoryRegionQuery gtrq = (GoalTrajectoryRegionQuery) trq;
      final StateTime endState = gtrq.getEndNode(furthestState.get());
      if (endState == null)
        throw new RuntimeException(); // the furthestState should be in the Map as it was taken from it
      furthestNode = Optional.ofNullable(getNode(convertToKey(endState.x())));
    }
    return furthestNode;
  }
}
