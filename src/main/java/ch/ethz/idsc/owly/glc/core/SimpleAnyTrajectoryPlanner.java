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

  @Override // from ExpandInterface
  public void expand(final GlcNode node) {
    // TODO count updates in cell based on costs for benchmarking
    Map<GlcNode, List<StateTime>> connectors = //
        SharedUtils.integrate(node, controls, stateIntegrator, costFunction);
    // Set<Tensor> domainsNeedingUpdate = new HashSet<>();
    // Map<Tensor, DomainQueue> candidates = new HashMap<>();
    CandidatePairQueueMap candidates = new CandidatePairQueueMap();
    for (GlcNode next : connectors.keySet()) { // <- order of keys is non-deterministic
      CandidatePair nextCandidate = new CandidatePair(node, next);
      final Tensor domain_key = convertToKey(next.state());
      candidates.insert(domain_key, nextCandidate);
      // ALL Candidates are saved in CandidateList
    }
    // save candidates in CandidateMap for RootSwitchlater
    processCandidates(node, connectors, candidates);
  }

  private void processCandidates( //
      GlcNode node, Map<GlcNode, List<StateTime>> connectors, CandidatePairQueueMap candidates) {
    for (Entry<Tensor, CandidatePairQueue> entry : candidates.map.entrySet()) {
      final Tensor domain_key = entry.getKey();
      final CandidatePairQueue candidateQueue = entry.getValue();
      if (candidateQueue != null && best == null) {
        while (!candidateQueue.isEmpty()) {
          final CandidatePair nextCandidatePair = candidateQueue.element();
          final GlcNode formerLabel = getNode(domain_key);
          final GlcNode next = nextCandidatePair.getCandidate();
          if (formerLabel != null) {
            if (Scalars.lessThan(next.merit(), formerLabel.merit())) {
              // collision check only if new node is better
              if (obstacleQuery.isDisjoint(connectors.get(next))) {// better node not collision
                // remove former Label from QUEUE
                queue().remove(formerLabel);
                // formerLabel disconnecting
                formerLabel.parent().removeEdgeTo(formerLabel);
                node.insertEdgeTo(next);
                insert(domain_key, next);
                if (!goalQuery.isDisjoint(connectors.get(next)))
                  offerDestination(next);
                candidateQueue.remove();
                break;
              }
            }
            // candidateQueue.remove();
          } else {
            // candidateQueue.remove();
            if (obstacleQuery.isDisjoint(connectors.get(next))) {
              node.insertEdgeTo(next);
              insert(domain_key, next);
              if (!goalQuery.isDisjoint(connectors.get(next)))
                offerDestination(next);
              candidateQueue.remove();
              break;
            }
          }
          candidateQueue.remove();
        }
      }
    }
  }

  @Override
  public int switchRootToNode(GlcNode newRoot) {
    GlcNode oldRoot = getNodesfromRootToGoal().get(0);
    if (newRoot.isRoot()) {
      System.out.println("node is already root");
      return 0;
    }
    int increaseDepthBy = newRoot.reCalculateDepth();
    int oldDomainMapSize = domainMap().size();
    int oldQueueSize = queue().size();
    int oldTreeSize = Nodes.ofSubtree(oldRoot).size();
    System.out.println("changing to root:" + newRoot.state());
    // removes the new root from the child list of its parent
    final GlcNode parent = newRoot.parent();
    parent.removeEdgeTo(newRoot);
    // Collecting deletetree
    {
      Collection<GlcNode> deleteTreeCollection = Nodes.ofSubtree(oldRoot);
      // -- GOAL deleted?
      if (best != null) {
        if (deleteTreeCollection.contains(best)) // check if goalnode was deleted
          best = null;
      }
      // -- QUEUE removal
      // TODO Parallizable?
      if (queue().removeAll(deleteTreeCollection)) // removing from queue;
        System.out.println("Removed " + (oldQueueSize - queue().size()) + " nodes from Queue");
      // -- DOMAIN removal
      domainMap().values().removeAll(deleteTreeCollection);
      // -- EDGE: Removing Edges between Nodes in DeleteTree
      // TODO paralizable?
      for (GlcNode tempNode : deleteTreeCollection) {
        if (!tempNode.isRoot())
          tempNode.parent().removeEdgeTo(tempNode);
      }
      System.out.println(oldDomainMapSize - domainMap().size() + " out of " + oldDomainMapSize + //
          " Domains removed from DomainMap = " + domainMap().size());
      System.out.println(deleteTreeCollection.size() + " out of " + oldTreeSize//
          + " Nodes removed from Tree = " + Nodes.ofSubtree(getNodesfromRootToGoal().get(0)).size());
    }
    // --
    System.out.println(0 + " Nodes added to Queue");
    System.out.println("**Rootswitch finished**");
    return increaseDepthBy;
  }

  /** @param newGoal is the new RegionQuery for the new Goalregion */
  public void setGoalQuery(CostFunction newCostFunction, TrajectoryRegionQuery newGoal) {
    this.goalQuery = newGoal;
    costFunction = newCostFunction;
    if (best != null) {
      List<StateTime> bestList = new ArrayList<>();
      bestList.add(best.stateTime());
      if (!newGoal.isDisjoint(bestList)) {
        offerDestination(best);
        System.out.println("Goal was already found in the existing tree");
        return;
      } // Old Goal is in new Goalregion
    }
    // Best is either not in newGoal or Null
    best = null;
    // Checking if goal is already in tree
    // TODO check if tree has right size TreeCollection
    {
      long tic = System.nanoTime();
      Collection<GlcNode> TreeCollection = Nodes.ofSubtree(getNodesfromRootToGoal().get(0));
      System.out.println("treesize for goal checking:" + TreeCollection.size());
      // TODO more efficient way then going through entire tree?
      Iterator<GlcNode> TreeCollectionIterator = TreeCollection.iterator();
      while (TreeCollectionIterator.hasNext()) {
        GlcNode current = TreeCollectionIterator.next();
        List<StateTime> currentList = new ArrayList<>();
        List<StateTime> bestList = new ArrayList<>();
        bestList.add(current.stateTime());
        if (!newGoal.isDisjoint(currentList)) { // current Node in Goal
          System.out.println("New Goal was found in current tree");
          offerDestination(current);
        }
      }
      long toc = System.nanoTime();
      System.out.println("Checked current tree for goal in "//
          + (toc - tic) * 1e-9 + "s");
    }
    // -- Updating the Queue
    long tic = System.nanoTime();
    // Changing the Merit in Queue for each Node
    List<GlcNode> list = new LinkedList<>(queue());
    queue().clear();
    list.stream().parallel() //
        .forEach(glcNode -> glcNode.setMinCostToGoal(costFunction.minCostToGoal(glcNode.state())));
    // list.stream().parallel().// Changing the depth of queue
    // forEach(glcNode -> glcNode.reCalculateDepth());
    queue().addAll(list);
    long toc = System.nanoTime();
    System.out.println("Updated Merit & Depth of Queue with " + list.size() + " nodes in: " //
        + ((toc - tic) * 1e-9) + "s");
    System.out.println("Goal switch finished");
  }
}