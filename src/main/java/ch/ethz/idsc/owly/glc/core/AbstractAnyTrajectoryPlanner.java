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
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.Tensor;

public abstract class AbstractAnyTrajectoryPlanner extends AbstractStandardTrajectoryPlanner implements AnyPlannerInterface {
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
    // TODO JONAS Check if Goal is reachable
    this.goalInterface = newGoal;
    GlcNode root = getRoot();
    setBestNull();
    // -- GOALCHECK TREE
    long tic = System.nanoTime();
    Collection<GlcNode> treeCollection = Nodes.ofSubtree(root);
    System.out.println("treesize for goal checking: " + treeCollection.size());
    // TODO can parallelize?
    Iterator<GlcNode> treeCollectionIterator = treeCollection.iterator();
    while (treeCollectionIterator.hasNext()) { // goes through entire tree
      GlcNode current = treeCollectionIterator.next();
      List<StateTime> currentState = new ArrayList<>();
      currentState.add(current.stateTime());
      if (!newGoal.isDisjoint(currentState)) // current Node in Goal
        offerDestination(current); // overwrites worse Goal
    }
    long toc = System.nanoTime();
    System.out.println("Checked current tree for goal in "//
        + (toc - tic) * 1e-9 + "s");
    if (getBest().isPresent()) {
      System.out.println("New Goal was found in current tree --> No new search needed");
      return true;
    }
    // -- TREE
    // Updating the merit of the entire tree
    tic = System.nanoTime();
    // Changing the Merit in Queue for each Node
    List<GlcNode> list = new LinkedList<>(queue());
    treeCollection.stream().parallel() //
        .forEach(glcNode -> glcNode.setMinCostToGoal(newGoal.minCostToGoal(glcNode.state())));
    queue().clear();
    queue().addAll(list);
    toc = System.nanoTime();
    System.out.println("Updated Merit of Tree with " + list.size() + " nodes in: " //
        + ((toc - tic) * 1e-9) + "s");
    System.out.println("**Goalswitch finished**");
    return false;
  }

  /** Finds the rootNode, by following the parents
   * from a random root Node in the tree/DomainMap
   * @return rootNode, which was found from random GlcNode in the tree */
  /* package */ final GlcNode getRoot() {
    Iterator<GlcNode> node = domainMap().values().iterator();
    if (node.hasNext())
      return Nodes.rootFrom(node.next());
    throw new RuntimeException();
    // if domainmap empty: no tree exists
    // TODO what to do if No Tree exists?
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

  /** Looks for the Node, which is the furthest in the GoalRegion,
   * @return node with highest merit in GoalRegion */
  public Optional<GlcNode> getFurthestGoalNode() {
    TrajectoryRegionQuery trq = this.getGoalQuery();
    PriorityQueue<GlcNode> queue = new PriorityQueue<>(Collections.reverseOrder(NodeMeritComparator.INSTANCE));
    List<StateTime> listStateTime = new ArrayList<>();
    if (trq instanceof SimpleTrajectoryRegionQuery) {
      SimpleTrajectoryRegionQuery strq = (SimpleTrajectoryRegionQuery) trq;
      listStateTime.addAll(strq.getDiscoveredMembers());
    }
    for (StateTime entry : listStateTime) {
      Tensor domainKey = convertToKey(entry.x());
      GlcNode node = getNode(domainKey);
      System.out.println("Size of listStateTime: " + listStateTime.size());
      queue.add(node);
    }
    return Optional.ofNullable(queue.peek());
  }
}
