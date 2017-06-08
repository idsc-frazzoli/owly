// code by jl
package ch.ethz.idsc.owly.glc.core;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import ch.ethz.idsc.owly.data.tree.Nodes;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.CostFunction;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;

/** TODO assumptions in order to use SimpleAnyTrajectoryPlanner */
public class SimpleAnyTrajectoryPlanner extends AbstractAnyTrajectoryPlanner {
  private final StateIntegrator stateIntegrator;
  private final Collection<Flow> controls;
  private TrajectoryRegionQuery goalQuery;
  private final TrajectoryRegionQuery obstacleQuery;
  // private final Map<Tensor, DomainQueue> domainCandidateMap = new HashMap<>();

  // private final Queue<Node> queue = new PriorityQueue<>(NodeMeritComparator.instance);
  public SimpleAnyTrajectoryPlanner( //
      Tensor eta, //
      StateIntegrator stateIntegrator, //
      Collection<Flow> controls, //
      CostFunction costFunction, //
      TrajectoryRegionQuery goalQuery, //
      TrajectoryRegionQuery obstacleQuery //
  ) {
    super(eta, stateIntegrator, costFunction, goalQuery, obstacleQuery);
    this.stateIntegrator = stateIntegrator;
    this.controls = controls;
    this.goalQuery = goalQuery;
    this.obstacleQuery = obstacleQuery;
  }

  @Override
  public void expand(final GlcNode node) {
    // TODO count updates in cell based on costs for benchmarking
    Map<GlcNode, List<StateTime>> connectors = //
        SharedUtils.integrate(node, controls, stateIntegrator, costFunction);
    // --
    DomainQueueMap domainQueueMap = new DomainQueueMap(); // holds candidates from insertion
    for (GlcNode next : connectors.keySet()) { // <- order of keys is non-deterministic
      final Tensor domainKey = convertToKey(next.state());
      final GlcNode former = getNode(domainKey);
      if (former != null) {
        // is already some node present from previous exploration ?
        if (Scalars.lessThan(next.merit(), former.merit())) // new node is potentially better than previous one
          domainQueueMap.insert(domainKey, next); // no Relabeling, therefore do not need to keep worse nodes
      } else // candidate is considered without comparison to any former node
        domainQueueMap.insert(domainKey, next);
    }
    // save candidates in CandidateMap for RootSwitchlater
    processCandidates(node, connectors, domainQueueMap);
  }

  private void processCandidates( //
      GlcNode node, Map<GlcNode, List<StateTime>> connectors, DomainQueueMap candidates) {
    for (Entry<Tensor, DomainQueue> entry : candidates.map.entrySet()) {
      final Tensor domain_key = entry.getKey();
      final DomainQueue domainQueue = entry.getValue();
      while (!domainQueue.isEmpty()) {
        final GlcNode next = domainQueue.poll();
        // collision check only if new node is better
        if (obstacleQuery.isDisjoint(connectors.get(next))) {// better node not collision
          node.insertEdgeTo(next);
          insert(domain_key, next);
          if (!goalQuery.isDisjoint(connectors.get(next)))
            offerDestination(next);
          break;
        }
      }
    }
  }

  @Override
  public int switchRootToNode(GlcNode newRoot) {
    if (newRoot.isRoot()) {
      System.out.println("node is already root");
      return 0;
    }
    // needed for change of depthlimit
    int increaseDepthBy = newRoot.reCalculateDepth();
    // -- DEBUGING Values
    // -- Collecting oldTree
    GlcNode oldRoot = getNodesfromRootToGoal().get(0);
    int oldDomainMapSize = domainMap().size();
    int oldQueueSize = queue().size();
    // -- BASIC REROOTING
    // removes the new root from the child list of its parent
    // Disconnecting newRoot from Old Tree and collecting DeleteTree
    final GlcNode parent = newRoot.parent();
    parent.removeEdgeTo(newRoot);
    // Collecting deleteTree
    Collection<GlcNode> deleteTreeCollection = Nodes.ofSubtree(oldRoot);
    // -- GOAL: goaldeleted?
    if (best != null) {
      if (deleteTreeCollection.contains(best)) // check if goalnode was deleted
        best = null;
    }
    // -- QUEUE: Deleting Nodes from Queue
    if (queue().removeAll(deleteTreeCollection)) // removing from queue;
      System.out.println("Removed " + (oldQueueSize - queue().size()) + " nodes from Queue");
    // -- DOMAINMAP: Removing Nodes (DeleteTree) from DomainMap
    domainMap().values().removeAll(deleteTreeCollection);
    System.out.println(oldDomainMapSize - domainMap().size() + " out of " + oldDomainMapSize + //
        " Domains removed from DomainMap = " + domainMap().size());
    System.out.println("**Rootswitch finished**");
    return increaseDepthBy;
  }

  /** Changes the Goal of the current planner:
   * rechecks the tree if expanding is needed, updates Merit of Nodes in Queue
   * @param newCostFunction modified Costfunction for heuristic
   * @param newGoal New GoalRegion */
  @Override
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
          System.out.println("New Goal was found in current tree --> No new search needed");
          long toc = System.nanoTime();
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