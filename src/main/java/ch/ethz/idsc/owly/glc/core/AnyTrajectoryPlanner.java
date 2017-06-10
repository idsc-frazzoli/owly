// code by jl
package ch.ethz.idsc.owly.glc.core;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.PriorityQueue;
import java.util.Set;

import ch.ethz.idsc.owly.data.tree.Nodes;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.CostFunction;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;

/** An anytime, Resolution asymptotically optimal and probabilitistic complete MotionPlanning Algorithm,
 * after: [B. Paden] A Generalized Label Correcting Method for Optimal Kinodynamic Motion Planning
 * Assumptions: -All states of all obstacles are known at all times
 * -No new Obstacles are discovered */
public class AnyTrajectoryPlanner extends AbstractAnyTrajectoryPlanner {
  private final Collection<Flow> controls;
  // CandidateMap saves neglected/pruned Nodes in a bucket for each domain
  private final Map<Tensor, Set<CandidatePair>> candidateMap = //
      new HashMap<>();

  public AnyTrajectoryPlanner( //
      Tensor eta, //
      StateIntegrator stateIntegrator, //
      Collection<Flow> controls, //
      CostFunction costFunction, //
      TrajectoryRegionQuery goalQuery, //
      TrajectoryRegionQuery obstacleQuery //
  ) {
    super(eta, stateIntegrator, costFunction, goalQuery, obstacleQuery);
    this.controls = controls;
    this.goalQuery = goalQuery;
  }

  @Override // from ExpandInterface
  public void expand(final GlcNode node) {
    // TODO count updates in cell based on costs for benchmarking
    Map<GlcNode, List<StateTime>> connectors = //
        SharedUtils.integrate(node, controls, stateIntegrator, costFunction);
    CandidatePairQueueMap candidatePairQueueMap = new CandidatePairQueueMap();
    for (GlcNode next : connectors.keySet()) { // <- order of keys is non-deterministic
      // ALL Candidates are saved in temporary CandidateList
      CandidatePair nextCandidate = new CandidatePair(node, next);
      final Tensor domainKey = convertToKey(next.state());
      candidatePairQueueMap.insert(domainKey, nextCandidate);
    }
    // saving the candidates in the corresponding Buckets
    for (Entry<Tensor, CandidatePairQueue> entry : candidatePairQueueMap.map.entrySet()) {
      if (!candidateMap.containsKey(entry.getKey()))
        candidateMap.put(entry.getKey(), new HashSet<CandidatePair>());
      candidateMap.get(entry.getKey()).addAll(entry.getValue());
    }
    processCandidates(node, connectors, candidatePairQueueMap);
    nodeAmountCheck(node);
  }

  // TODO BUG: if big numbers of nodes are expanded, nodes =/= domains
  private void processCandidates( //
      GlcNode node, Map<GlcNode, List<StateTime>> connectors, CandidatePairQueueMap candidatePairQueueMap) {
    for (Entry<Tensor, CandidatePairQueue> entry : candidatePairQueueMap.map.entrySet()) {
      final Tensor domainKey = entry.getKey();
      final CandidatePairQueue candidateQueue = entry.getValue();
      if (candidateQueue != null && best == null) {
        while (!candidateQueue.isEmpty()) {
          // retrieving the Candidates
          final CandidatePair nextCandidatePair = candidateQueue.element();
          final GlcNode formerLabel = getNode(domainKey);
          final GlcNode next = nextCandidatePair.getCandidate();
          if (formerLabel != null) {
            if (Scalars.lessThan(next.merit(), formerLabel.merit())) {
              // collision check only if new node is better
              if (obstacleQuery.isDisjoint(connectors.get(next))) {// better node not collision
                // current label back in bucket for this domains,
                CandidatePair formerCandidate = new CandidatePair(formerLabel.parent(), formerLabel);
                candidateMap.get(domainKey).add(formerCandidate);
                // Removing the formerLabel from the Queue, if in it
                queue().remove(formerLabel);
                // removing the nextCandidate from bucket of this domain
                // formerLabel disconnecting
                formerLabel.parent().removeEdgeTo(formerLabel);
                // adding next to tree and DomainMap
                node.insertEdgeTo(next);
                insert(domainKey, next);
                candidateMap.get(domainKey).remove(nextCandidatePair);
                candidateQueue.remove();
                // GOAL check
                if (!goalQuery.isDisjoint(connectors.get(next)))
                  offerDestination(next);
                break;
              }
            }
          } else { // No formerLabel, so definitely adding a Node
            if (obstacleQuery.isDisjoint(connectors.get(next))) {
              // removing the nextCandidate from bucket of this domain
              // adding next to tree and DomainMap
              node.insertEdgeTo(next);
              insert(domainKey, next);
              candidateMap.get(domainKey).remove(nextCandidatePair);
              candidateQueue.remove();
              // GOAL check
              if (!goalQuery.isDisjoint(connectors.get(next)))
                offerDestination(next);
              break;
            }
          }
          // remove from CandidateMap and TemporaryQueue as was not better/or in Collision
          candidateMap.get(domainKey).remove(nextCandidatePair); // !not if new environment
          candidateQueue.remove();
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
    int increasedDepthBy = newRoot.reCalculateDepth();
    // -- DEBUGING Values
    // -- Collecting Oldtree
    GlcNode oldRoot = Nodes.rootFrom(getBestOrElsePeek());
    Collection<GlcNode> oldTreeCollection = Nodes.ofSubtree(oldRoot);
    int oldDomainMapSize = domainMap().size();
    long oldtotalCandidates = candidateMap.values().stream().flatMap(Collection::stream).count();
    int oldQueueSize = queue().size();
    int nodesWithoutDomain = 0;
    Collection<GlcNode> debugCollection = new ArrayList<GlcNode>(oldTreeCollection);
    debugCollection.removeIf(temp -> (domainMap().get(convertToKey(temp.state()))).equals(temp));
    nodesWithoutDomain = debugCollection.size();
    for (GlcNode tempNode : debugCollection) {
      System.out.println("additional Nodes not in a Domain: state: " + tempNode.state() + "cost" + tempNode.costFromRoot());
      if (!tempNode.isRoot())
        System.out.println("has a parent");
      if (tempNode.isLeaf())
        System.out.println("is leaf");
      if (tempNode.flow() != null)
        System.out.println("has flow");
    }
    if (nodesWithoutDomain != 0)
      throw new RuntimeException();
    // -- BASIC REROOTING
    // removes the new root from the child list of its parent
    // Disconnecting newRoot from Old Tree and collecting DeleteTree
    newRoot.parent().removeEdgeTo(newRoot);
    Collection<GlcNode> deleteTreeCollection = deleteChildrenOf(oldRoot);
    // -- DEBUGING
    System.out.println("Removed " + (oldQueueSize - queue().size()) + " out of " + oldQueueSize + " nodes from Queue = " + queue().size());
    System.out.println(oldDomainMapSize - domainMap().size() + " out of " + oldDomainMapSize + //
        " Domains removed from DomainMap = " + domainMap().size());
    final GlcNode root = Nodes.rootFrom(getBestOrElsePeek());
    Collection<GlcNode> newTreeCollection = Nodes.ofSubtree(root);
    System.out.println(deleteTreeCollection.size() + " out of " + oldTreeCollection.size()//
        + " Nodes removed from Tree = " + newTreeCollection.size());
    // -- CANDIDATEMAP: Deleting Candidates, if Origin is included in DeleteTree
    // TODO What is the time gain by parallization?
    candidateMap.entrySet().parallelStream().forEach( //
        CandidateSet -> CandidateSet.getValue().removeIf(cp -> deleteTreeCollection.contains(cp.getOrigin())));
    // -- DEBUGING
    long newtotalCandidates = candidateMap.values().stream().flatMap(Collection::stream).count();
    System.out.println(oldtotalCandidates - newtotalCandidates + " of " + oldtotalCandidates + //
        " Candidates removed from CandidateList ");
    // -- RELABELING:
    int addedNodesToQueue = 0;
    for (GlcNode formerLabel : deleteTreeCollection) {
      Tensor domainKey = convertToKey(formerLabel.state());
      // if a bucket exists do Relabeling otherwise not
      if (candidateMap.containsKey(domainKey)) {
        Set<CandidatePair> tempCandidateSet = candidateMap.get(domainKey);
        // TODO modify constructor of CandidatePairQueue so below statement works
        // CandidatePairQueue candidateQueue = new CandidatePairQueue(tempCandidateSet);
        PriorityQueue<CandidatePair> candidateQueue = new PriorityQueue<>(tempCandidateSet);
        while (!candidateQueue.isEmpty()) {
          // retrieving the Candidates
          final CandidatePair nextCandidatePair = candidateQueue.element();
          final GlcNode next = nextCandidatePair.getCandidate();
          final GlcNode nextParent = nextCandidatePair.getOrigin();
          // check if Delete was properly conducted
          if (deleteTreeCollection.contains(nextParent)) {
            System.out.println("parent of node will be deleted --> not in QUEUE ");
            break;
          }
          final List<StateTime> connector = //
              stateIntegrator.trajectory(nextParent.stateTime(), next.flow());
          if (obstacleQuery.isDisjoint(connector)) { // no collision
            nextParent.insertEdgeTo(next);
            // data structure changing statement shall not be in if clause:
            final boolean replaced = insert(domainKey, next);
            // DomainMap at this key should be empty
            candidateMap.get(domainKey).remove(nextCandidatePair);
            candidateQueue.remove();
            if (replaced) {
              System.out.println("Something was replaced --> BUG");
              throw new RuntimeException();
            }
            addedNodesToQueue++;
            if (!goalQuery.isDisjoint(connector))
              offerDestination(next);
            break; // leaves the while loop, but not the for loop
          }
          // remove from CandidateMap and TemporaryQueue as was not better/or in Collision
          candidateMap.get(domainKey).remove(nextCandidatePair);// !not if new environment
          candidateQueue.remove();
        }
      }
    }
    System.out.println(addedNodesToQueue + " Nodes added to Domain = " + domainMap().size());
    System.out.println("**Rootswitch finished**");
    return increasedDepthBy;
  }
}
