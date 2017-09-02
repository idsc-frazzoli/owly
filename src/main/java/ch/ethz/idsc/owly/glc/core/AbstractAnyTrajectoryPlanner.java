// code by jl
package ch.ethz.idsc.owly.glc.core;

import java.util.Collection;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;
import java.util.Optional;

import ch.ethz.idsc.owly.data.tree.Nodes;
import ch.ethz.idsc.owly.glc.adapter.HeuristicQ;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.TrajectoryGoalManager;
import ch.ethz.idsc.owly.math.region.EmptyRegion;
import ch.ethz.idsc.owly.math.region.InvertedRegion;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.Scalar;
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
  protected synchronized final void offerDestination(GlcNode node, List<StateTime> connector) {
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
    best.keySet().removeAll(deleteTreeCollection);
    // -- QUEUE: Deleting Nodes from Queue
    // removal from queue might lead to non convergence to optimal Solution, when R is increased
    queue().removeAll(deleteTreeCollection);
    // -- DOMAINMAP: Removing Nodes (DeleteTree) from DomainMap
    domainMap().values().removeAll(deleteTreeCollection);
    // -- EDGE: Removing Edges between Nodes in DeleteTree
    // TODO edge removal of all nodes needed?
    // better for garbage collector, otherwise child<->parent pair might keep itself in existence
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

  @Override
  public final boolean changeToGoal(final GoalInterface newGoal) {
    return changeToGoal(newGoal, new InvertedRegion(EmptyRegion.INSTANCE));
    // creates always true Region, as a helper =>no change
  }

  @Override
  public final boolean changeToGoal(final GoalInterface newGoal, Region goalCheckHelp) {
    System.out.println("*** GOALSWITCH ***");
    long tictotal = System.nanoTime();
    {
      // boolean noHeuristic = ((getGoalInterface() instanceof NoHeuristic) && (newGoal instanceof NoHeuristic));
      // boolean noHeuristic = !getGoalInterface().hasHeuristic() && !newGoal.hasHeuristic();
      boolean noHeuristic = !HeuristicQ.of(getGoalInterface()) && !HeuristicQ.of(newGoal);
      setGoalInterface(newGoal);
      long tic = System.nanoTime();
      @SuppressWarnings("unused")
      GlcNode root = getRoot();
      // Collection<GlcNode> treeCollection = Nodes.ofSubtree(root);
      Collection<GlcNode> treeCollection = domainMap().values();
      setBestNull();
      // -- RESORTING OF TREE
      if (!noHeuristic) {
        // treeCollection.stream().parallel() //
        // .forEach(glcNode -> glcNode.setMinCostToGoal(getGoalInterface().minCostToGoal(glcNode.state())));
        relabelingDomains();
        List<GlcNode> list = new LinkedList<>(queue());
        queue().clear();
        queue().addAll(list);
      }
      long toc = System.nanoTime();
      System.out.println("Relabeled Tree with " + treeCollection.size() + " nodes in: " //
          + ((toc - tic) * 1e-9) + "s");
    }
    {
      // --
      // -- GOALCHECK TREE
      boolean goalInTreeFound = false;
      long tic = System.nanoTime();
      // old check for debugging
      // goalInTreeFound = goalCheckTree();
      // Scalar timeDiffOld = RealScalar.of((System.nanoTime() - tic) * 1e-9);
      // Collection<GlcNode> oldBest = new ArrayList<>(best.keySet());
      // setBestNull();
      tic = System.nanoTime();
      goalInTreeFound = goalCheckTree(goalCheckHelp);
      // DEBUGING
      // Scalar timeDiffNew = RealScalar.of((System.nanoTime() - tic) * 1e-9);
      // tic = System.nanoTime();
      // System.err.println("The NEW GoalCheck needed: " //
      // + timeDiffNew.divide(timeDiffOld).multiply(RealScalar.of(100)).number().intValue()//
      // + "% of the time of the OLD");
      // if (!best.isEmpty() || !oldBest.isEmpty()) {
      // System.err.println("OldVersion found: " + oldBest.size() + " GoalNodes: ");
      // for (GlcNode node : oldBest)
      // System.out.println(node.state());
      // System.err.println("NewVersion found: " + best.size() + " GoalNodes");
      // for (GlcNode node : best.keySet())
      // System.out.println(node.state());
      // }
      // if (!(oldBest.containsAll(best.keySet()) && best.keySet().containsAll(oldBest))) {
      // System.err.println("Not the same GoalNodes found in both runs");
      // throw new RuntimeException();
      // }
      // INFORMATION
      System.out.println("Checked current tree for goal in "//
          + (System.nanoTime() - tic) * 1e-9 + "s");
      if (goalInTreeFound) {
        System.err.println("FOUND GOAL IN TREE");
        System.out.println("*** Goalswitch finished in " + (System.nanoTime() - tictotal) * 1e-9 + "s ***");
        return true;
      }
    }
    System.out.println("*** Goalswitch finished in " + (System.nanoTime() - tictotal) * 1e-9 + "s ***");
    return false;
  }

  /** Checks if relabeling is needed for all domains with their Candidates and relabels those.
   * Trees which are suboptimal are deleted */
  abstract void relabelingDomains();

  /** Checks the tree in the collection if some Nodes are in the Goal
   * @param goalCheckHelp a Region, which includes ALL Nodes, which could have a leaving trajectory in the Goal
   * @return true if a Node in the Goal was found in this Collection */
  abstract boolean goalCheckTree(final Region goalCheckHelp);

  abstract boolean goalCheckTree();

  @Override
  abstract public void obstacleUpdate(TrajectoryRegionQuery newObstacle);

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

  @Override
  public final Optional<GlcNode> existsInTree(StateTime stateTime) {
    GlcNode label = domainMap().get(convertToKey(stateTime.state()));
    if (label == null)
      return Optional.empty();
    if (label.stateTime().state().equals(stateTime.state()))
      return Optional.ofNullable(label);
    return Optional.empty();
  }

  @Override
  public final Optional<StateTime> getFurthestGoalState() {
    Optional<GlcNode> key = getFurthestGoalNode();
    if (key.isPresent()) {
      List<StateTime> bestTrajectory = best.get(key.get());
      int index = getGoalInterface().firstMember(bestTrajectory);
      if (index >= 0)
        return Optional.ofNullable(bestTrajectory.get(index));
    }
    return Optional.empty();
  }

  /** @return furthest Node (lowest cost in highest list index), whose incoming trajectory is in GoalRegion,
   * or Optional.empty() if no such node has been identified yet */
  @Override
  protected final Optional<GlcNode> getFurthestGoalNode() {
    if (!(getGoalInterface() instanceof TrajectoryGoalManager))
      throw new RuntimeException(); // can only run on for TrajectoryGoalManager
    List<Region> goalRegions = ((TrajectoryGoalManager) getGoalInterface()).getGoalRegionList();
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

  public final Scalar getGoalCost() {
    GlcNode root = getRoot();
    if (getBest().isPresent())
      return getBest().get().costFromRoot().subtract(root.costFromRoot());
    return null;
  }
}
