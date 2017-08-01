// code by jl
package ch.ethz.idsc.owly.glc.core;

import java.util.Collection;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;
import java.util.Optional;

import ch.ethz.idsc.owly.data.tree.Nodes;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.TrajectoryGoalManager;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RealScalar;
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

  @Override
  final void offerDestination(GlcNode node, List<StateTime> connector) {
    best.put(node, connector); // always put new GoalNodes in Map
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
      int size = best.size();
      boolean test = best.keySet().removeAll(deleteTreeCollection);
      if (test)
        System.out.println("min. 1 GoalNode removed from best, " + (size - best.size()) + "/" + size);
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
  public final boolean changeToGoal(final GoalInterface newGoal) {
    setGoalInterface(newGoal);
    GlcNode root = getRoot();
    Collection<GlcNode> treeCollection = Nodes.ofSubtree(root);
    setBestNull();
    // -- TREE
    // Updating the merit of the entire tree
    long tic = System.nanoTime();
    // Changing the Merit in Queue for each Node
    // TODO JAN: Does this make !treeCollection.equals(compareCollection)==true? if values are changed in treeCollection?
    treeCollection.stream().parallel() //
        .forEach(glcNode -> glcNode.setMinCostToGoal(newGoal.minCostToGoal(glcNode.state())));
    // if (false) {
    if (newGoal.minCostToGoal(root.state()) != RealScalar.ZERO) {
      // TODO JONAS smart way to check if before line modified sth.
      System.err.println("checking for domainlabel changes due to heuristic change,  Treesize: " + treeCollection.size());
      // TODO JONAS for optimiality if Heuristic was changed, check candidates in domains
      RelabelingDomains();
    }
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
    treeCollection = Nodes.ofSubtree(root);
    // TODO JONAS: would a sorted list make sense, as GoalCheck could stop if 1 Goal was found
    // only for "normal" goals not for TrajectoryGoalmanager
    boolean treeFound = GoalCheckTree(treeCollection);
    toc = System.nanoTime();
    System.out.println("Checked current tree for goal in "//
        + (toc - tic) * 1e-9 + "s");
    if (treeFound) {
      System.out.println("New Goal was found in current tree --> No new search needed");
      System.out.println("**Goalswitch finished**");
      return true;
    }
    System.out.println("**Goalswitch finished**");
    return false;
  }

  abstract void RelabelingDomains();

  /** Checks the tree in the collection if some Nodes are in the Goal
   * 
   * @param treeCollection of Nodes, which should be checked if they are in the goal
   * @return true if a Node in the Goal was found in this Collection */
  abstract boolean GoalCheckTree(Collection<GlcNode> treeCollection);

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
  public final List<StateTime> trajectoryToBest() {
    Optional<GlcNode> tempBest = getBestOrElsePeek();
    if (tempBest.isPresent())
      return GlcNodes.getPathFromRootTo(tempBest.get());
    return null;
  }

  @Override
  public final List<TrajectorySample> detailedTrajectoryToBest() {
    Optional<GlcNode> optional = getBestOrElsePeek();
    if (optional.isPresent())
      return GlcTrajectories.connect(getStateIntegrator(), Nodes.listFromRoot(getBest().get()));
    return null;
  }

  /** Looks for the Node, which is the furthest in the GoalRegion,
   * @return node with highest merit in GoalRegion */
  @Override
  // TODO JONAS: modify similar to finalgoal
  public final Optional<StateTime> getFurthestGoalState(List<Region> goalRegions) {
    Optional<GlcNode> key = getFurthestGoalNode(goalRegions);
    if (key.isPresent()) {
      List<StateTime> bestTrajectory = best.get(key.get());
      int index = getGoalQuery().firstMember(bestTrajectory);
      if (index >= 0)
        return Optional.ofNullable(bestTrajectory.get(index));
    }
    return Optional.empty();
  }

  // @Override
  // public final Optional<GlcNode> getFurthestGoalNode() {
  // if (!best.isEmpty())
  // return Optional.ofNullable(best.lastKey());
  // return Optional.empty();
  // }
  @Override
  public final Optional<GlcNode> getFurthestGoalNode(List<Region> goalRegions) {
    ListIterator<Region> iter = goalRegions.listIterator(goalRegions.size());
    DomainQueue regionQueue = new DomainQueue(); // priority queue over merit of GlcNodes
    while (iter.hasPrevious()) { // goes through all regions from last to first
      SimpleTrajectoryRegionQuery strq = new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(iter.previous()));// go through Regions from the last to first:
      for (GlcNode tempBest : best.keySet()) {
        List<StateTime> trajectory = best.get(tempBest);
        if (strq.firstMember(trajectory) >= 0)
          regionQueue.add(tempBest); // saves members in PQ
      }
      if (!regionQueue.isEmpty())
        break; // leave while loop when Nodes where found in latest Goalregion
    }
    return Optional.ofNullable(regionQueue.peek());
  }

  @Override
  public final Optional<GlcNode> getFinalGoalNode() {
    if (getGoalQuery() instanceof TrajectoryGoalManager) {
      List<Region> goalRegions = ((TrajectoryGoalManager) getGoalQuery()).getGoalRegionList();
      Optional<GlcNode> furthest = getFurthestGoalNode(goalRegions);
      if (furthest.isPresent())
        return furthest;
    }
    return getBestOrElsePeek();
  }
}
