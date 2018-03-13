// code by jl
package ch.ethz.idsc.owl.glc.any;

import java.util.Collection;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;

import ch.ethz.idsc.owl.data.Stopwatch;
import ch.ethz.idsc.owl.data.tree.Nodes;
import ch.ethz.idsc.owl.glc.adapter.GlcNodes;
import ch.ethz.idsc.owl.glc.adapter.GlcTrajectories;
import ch.ethz.idsc.owl.glc.adapter.HeuristicQ;
import ch.ethz.idsc.owl.glc.core.AbstractTrajectoryPlanner;
import ch.ethz.idsc.owl.glc.core.ControlsIntegrator;
import ch.ethz.idsc.owl.glc.core.GlcNode;
import ch.ethz.idsc.owl.glc.core.GoalInterface;
import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owl.math.region.InvertedRegion;
import ch.ethz.idsc.owl.math.region.Region;
import ch.ethz.idsc.owl.math.region.Regions;
import ch.ethz.idsc.owl.math.state.StateIntegrator;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.owl.math.state.TrajectorySample;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

public abstract class AbstractAnyTrajectoryPlanner extends AbstractTrajectoryPlanner implements AnyPlannerInterface {
  protected transient ControlsIntegrator controlsIntegrator;
  private final Collection<Flow> controls;
  public transient Stopwatch subTreeDeleterWatch = Stopwatch.stopped();
  public transient Stopwatch integratorWatch1 = Stopwatch.stopped();
  public transient Stopwatch integratorWatch2 = Stopwatch.stopped();

  protected AbstractAnyTrajectoryPlanner( //
      Tensor eta, //
      StateIntegrator stateIntegrator, //
      Collection<Flow> controls, //
      TrajectoryRegionQuery obstacleQuery, //
      GoalInterface goalInterface //
  ) {
    super(eta, stateIntegrator, obstacleQuery, goalInterface);
    controlsIntegrator = new ControlsIntegrator( //
        stateIntegrator, //
        () -> controls.stream().parallel(), //
        goalInterface);
    this.controls = controls;
  }

  @Override
  protected synchronized final void offerDestination(GlcNode node, List<StateTime> connector) {
    best.put(node, connector); // always put new GoalNodes in Map
  }

  /** Includes all the functionality of the RootSwitch
   * (deleting of the useless nodes and relabling of modified Domains)
   * @param stateTime the new Rootstate
   * @return The value,by which the depth limit needs to be increased as of the RootSwitch */
  @Override
  public final int switchRootToState(StateTime stateTime) {
    Optional<GlcNode> newRoot = getNode(convertToKey(stateTime));
    int increaseDepthBy = 0;
    // TODO JONAS not nice, as we jump from state to startnode
    if (newRoot.isPresent()) {
      increaseDepthBy = switchRootToNode(newRoot.get());
    } else {
      System.err.println("***RESET***");
      System.out.println("This domain is not labelled yet:");
      System.out.println(stateTime.toInfoString());
      if (!domainMap().isEmpty()) {
        deleteSubtreeOf(getRoot());
        domainMap().clear();
        queue().clear();
      }
      // we did not follow the planned path ==> new Motion planning problem,
      // also often used to start a new MPP
      insertRoot(stateTime);
    }
    return increaseDepthBy;
  }

  protected final void insertNodeInTree(GlcNode parent, GlcNode node) {
    parent.insertEdgeTo(node);
    final Tensor domainKey = convertToKey(node.stateTime());
    final boolean replaced = insert(domainKey, node);
    if (replaced) {
      System.err.println("No formerLabel existed, but sth. was replaced");
      throw new RuntimeException();
    }
  }

  /** @param baseNode Node, of which all children and itself should be deleted
   * @return the Collection of Nodes, which should be deleted */
  protected final Collection<GlcNode> deleteSubtreeOf(GlcNode baseNode) {
    subTreeDeleterWatch.start();
    Collection<GlcNode> deleteTreeCollection = new HashSet<>();
    Nodes.ofSubtree(baseNode, deleteTreeCollection);
    // {
    // domainMap().values().removeIf(deleteTreeCollection::contains);
    // }
    // Collection<GlcNode> deleteTreeCollection = Nodes.ofSubtree(baseNode);
    // // -- GOAL: goalNode deleted?
    best.keySet().removeAll(deleteTreeCollection);
    // // -- QUEUE: Deleting Nodes from Queue
    // // removal from queue might lead to non convergence to optimal Solution, when R is increased
    queue().removeAll(deleteTreeCollection);
    // // -- DOMAINMAP: Removing Nodes (DeleteTree) from DomainMap
    for (GlcNode node : deleteTreeCollection)
      domainMap().remove(convertToKey(node.stateTime()));
    // boolean test = domainMap().values().removeAll(deleteTreeCollection);
    // if (test)
    // throw new RuntimeException();
    // -- EDGE: Removing Edges between Nodes in DeleteTree
    // TODO JONAS edge removal of all nodes needed?
    // better for garbage collector, otherwise child<->parent pair might keep itself in existence
    // Minimum needed:
    // baseRoot.parent().removeEdgeTo(baseRoot);
    // oldRoot has no parent, therefore is skipped
    deleteTreeCollection.remove(baseNode);
    // if parallel, run in below exceptions
    deleteTreeCollection.stream().forEach(tempNode -> tempNode.parent().removeEdgeTo(tempNode));
    deleteTreeCollection.add(baseNode);
    subTreeDeleterWatch.stop();
    if (!baseNode.isLeaf())
      throw new RuntimeException();
    return deleteTreeCollection;
  }

  @Override
  public final boolean changeToGoal(final GoalInterface newGoal) {
    return changeToGoal(newGoal, new InvertedRegion(Regions.emptyRegion()));
    // creates always true Region, as a helper =>no change
  }

  @Override
  public final boolean changeToGoal(final GoalInterface newGoal, Region<Tensor> goalCheckHelp) {
    System.out.println("*** GOALSWITCH ***");
    long tictotal = System.nanoTime();
    {
      // boolean noHeuristic = ((getGoalInterface() instanceof NoHeuristic) && (newGoal instanceof NoHeuristic));
      // boolean noHeuristic = !getGoalInterface().hasHeuristic() && !newGoal.hasHeuristic();
      boolean noHeuristic = !HeuristicQ.of(getGoalInterface()) && !HeuristicQ.of(newGoal);
      changeGoalInterface(newGoal);
      long tic = System.nanoTime();
      Collection<GlcNode> treeCollection = domainMap().values();
      setBestNull();
      // -- RESORTING OF TREE
      if (!noHeuristic) {
        treeCollection.stream().parallel() //
            .forEach(glcNode -> glcNode.setMinCostToGoal(getGoalInterface().minCostToGoal(glcNode.state())));
        // relabelingDomains();
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
      tic = System.nanoTime();
      goalInTreeFound = goalCheckTree(goalCheckHelp);
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

  protected void changeGoalInterface(GoalInterface newGoal) {
    setGoalInterface(newGoal);
    controlsIntegrator = new ControlsIntegrator(stateIntegrator, () -> controls.stream().parallel(), newGoal);
  }

  /** Checks if relabeling is needed for all domains with their Candidates and relabels those.
   * Trees which are suboptimal are deleted */
  abstract void relabelingDomains();

  /** Checks the tree in the collection if some Nodes are in the Goal
   * @param goalCheckHelp a Region, which includes ALL Nodes, which could have a leaving trajectory in the Goal
   * @return true if a Node in the Goal was found in this Collection */
  abstract boolean goalCheckTree(final Region<Tensor> goalCheckHelp);

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
    GlcNode label = domainMap().get(convertToKey(stateTime));
    if (Objects.isNull(label)) {
      return Optional.empty();
    }
    if (label.stateTime().state().equals(stateTime.state())) // check if node=label
      return Optional.ofNullable(label);
    System.out.println("Label is unequal :" + label.stateTime().toInfoString());
    return Optional.empty();
  }

  @Override
  public final Optional<StateTime> getFurthestGoalState() {
    Optional<GlcNode> key = getFurthestGoalNode();
    if (key.isPresent())
      return getGoalInterface().firstMember(best.get(key.get()));
    return Optional.empty();
  }

  public final Scalar getTrajectoryCost() {
    GlcNode root = getRoot();
    if (getBest().isPresent())
      return getBest().get().costFromRoot().subtract(root.costFromRoot());
    return null;
  }

  @Override
  public void printTimes() {
    System.out.println("Times for the AnyPlanner");
    System.out.println("Integrator took: " + integratorWatch.display_seconds());
    System.out.println(" Integratorwatch1: " + integratorWatch1.display_seconds());
    System.out.println(" Integratorwatch2: " + integratorWatch2.display_seconds());
    System.out.println("processing C took: " + processCWatch.display_seconds());
    System.out.println(" deleting subtrees took " + subTreeDeleterWatch.display_seconds());
    integratorWatch = Stopwatch.stopped();
    processCWatch = Stopwatch.stopped();
    subTreeDeleterWatch = Stopwatch.stopped();
    integratorWatch1 = Stopwatch.stopped();
    integratorWatch2 = Stopwatch.stopped();
  }
}
