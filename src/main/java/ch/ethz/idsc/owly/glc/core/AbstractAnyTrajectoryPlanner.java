// code by jl
package ch.ethz.idsc.owly.glc.core;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

import ch.ethz.idsc.owly.data.tree.Nodes;
import ch.ethz.idsc.owly.math.state.CostFunction;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;

/* package */ abstract class AbstractAnyTrajectoryPlanner extends TrajectoryPlanner {
  final StateIntegrator stateIntegrator;
  TrajectoryRegionQuery goalQuery;
  final TrajectoryRegionQuery obstacleQuery;
  /* not final */ CostFunction costFunction;

  protected AbstractAnyTrajectoryPlanner(Tensor eta, //
      StateIntegrator stateIntegrator, //
      CostFunction costFunction, //
      TrajectoryRegionQuery goalQuery, //
      TrajectoryRegionQuery obstacleQuery) {
    super(eta);
    this.stateIntegrator = stateIntegrator;
    this.costFunction = costFunction;
    this.goalQuery = goalQuery;
    this.obstacleQuery = obstacleQuery;
  }

  /** Includes all the functionality of the RootSwitch
   * (deleting of the useless nodes and relabling of modified Domains)
   * @param state the new Rootstate
   * @return The value,by which the depth limit needs to be increased as of the RootSwitch */
  public final int switchRootToState(Tensor state) {
    GlcNode newRoot = this.getNode(convertToKey(state));
    int increaseDepthBy = 0;
    // TODO not nice, as we jump from state to startnode
    if (newRoot != null)
      increaseDepthBy = switchRootToNode(newRoot);
    else {
      System.out.println("This domain  is not labelled yet:");
      System.out.println(state);
      throw new RuntimeException();
      // TODO: should replan everything, as we left old trajectory
    }
    return increaseDepthBy;
  }

  /** Includes all the functionality of the RootSwitch
   * (deleting of the useless nodes and relabling of modified Domains)
   * @param newRoot Node to Switch
   * @return The value,by which the depth limit needs to be increased as of the RootSwitch */
  public abstract int switchRootToNode(GlcNode newRoot);

  protected final Collection<GlcNode> deleteChildrenOf(GlcNode oldRoot) {
    Collection<GlcNode> deleteTreeCollection = Nodes.ofSubtree(oldRoot);
    // -- GOAL: goal deleted?
    if (best != null)
      if (deleteTreeCollection.contains(best))
        best = null;
    System.out.println("Nodes to be deleted: " + deleteTreeCollection.size());
    // -- QUEUE: Deleting Nodes from Queue
    queue().removeAll(deleteTreeCollection);
    // -- DOMAINMAP: Removing Nodes (DeleteTree) from DomainMap
    domainMap().values().removeAll(deleteTreeCollection);
    // --
    // -- EDGE: Removing Edges between Nodes in DeleteTree
    // TODO: edge removal Needed?
    // oldRoot has no parent, therefore is skipped
    deleteTreeCollection.remove(oldRoot);
    // TODO: parralizable?
    deleteTreeCollection.forEach(tempNode -> tempNode.parent().removeEdgeTo(tempNode));
    deleteTreeCollection.add(oldRoot);
    return deleteTreeCollection;
  }

  @Override
  public final List<TrajectorySample> detailedTrajectoryTo(GlcNode node) {
    return GlcTrajectories.connect(stateIntegrator, Nodes.fromRoot(node));
  }

  @Override
  public final TrajectoryRegionQuery getObstacleQuery() {
    return obstacleQuery;
  }

  @Override
  public final TrajectoryRegionQuery getGoalQuery() {
    return goalQuery;
  }

  @Override
  protected final GlcNode createRootNode(Tensor x) {
    return GlcNode.of(null, new StateTime(x, RealScalar.ZERO), RealScalar.ZERO, //
        costFunction.minCostToGoal(x));
  }

  /** Changes the Goal of the current planner:
   * rechecks the tree if expanding is needed, updates Merit of Nodes in Queue
   * @param newCostFunction modified Costfunction for heuristic
   * @param newGoal New GoalRegion
   * @return boolean, true if Goal was already found in oldTree */
  public boolean changeGoal(CostFunction newCostFunction, TrajectoryRegionQuery newGoal) {
    this.goalQuery = newGoal;
    this.costFunction = newCostFunction;
    // -- GOALCHECK BEST
    // TODO needed? as tree check will find it anyways, (maybe a better best), Pros: maybe timegain
    if (best != null) {
      List<StateTime> bestState = new ArrayList<>();
      bestState.add(best.stateTime());
      if (!newGoal.isDisjoint(bestState)) {
        offerDestination(best);
        System.out.println("Old Goal is in new Goalregion");
        return true;
      } // Old Goal is in new Goalregion
    }
    // Best is either not in newGoal or Null
    best = null;
    // -- GOALCHECK TREE
    {
      long tic = System.nanoTime();
      Collection<GlcNode> TreeCollection = Nodes.ofSubtree(getNodesfromRootToGoal().get(0));
      System.out.println("treesize for goal checking: " + TreeCollection.size());
      // TODO more efficient way then going through entire tree?
      Iterator<GlcNode> TreeCollectionIterator = TreeCollection.iterator();
      while (TreeCollectionIterator.hasNext()) {
        GlcNode current = TreeCollectionIterator.next();
        List<StateTime> currentState = new ArrayList<>();
        currentState.add(current.stateTime());
        if (!newGoal.isDisjoint(currentState)) { // current Node in Goal
          offerDestination(current);
          long toc = System.nanoTime();
          System.out.println("New Goal was found in current tree --> No new search needed");
          System.out.println("Checked current tree for goal in "//
              + (toc - tic) * 1e-9 + "s");
          return true;
        }
      }
      long toc = System.nanoTime();
      System.out.println("Checked current tree for goal in "//
          + (toc - tic) * 1e-9 + "s");
    }
    // -- QUEUE
    // Updating the Queue
    long tic = System.nanoTime();
    // Changing the Merit in Queue for each Node
    List<GlcNode> list = new LinkedList<>(queue());
    queue().clear();
    list.stream().parallel() //
        .forEach(glcNode -> glcNode.setMinCostToGoal(costFunction.minCostToGoal(glcNode.state())));
    queue().addAll(list);
    long toc = System.nanoTime();
    System.out.println("Updated Merit of Queue with " + list.size() + " nodes in: " //
        + ((toc - tic) * 1e-9) + "s");
    System.out.println("**Goalswitch finished**");
    return false;
  }
}
