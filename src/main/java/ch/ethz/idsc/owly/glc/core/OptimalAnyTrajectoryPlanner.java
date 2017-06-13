// code by jl
package ch.ethz.idsc.owly.glc.core;

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
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;

/** An anytime, Resolution asymptotically optimal and probabilitistic complete MotionPlanning Algorithm,
 * after: [B. Paden] A Generalized Label Correcting Method for Optimal Kinodynamic Motion Planning
 * Assumptions: -All states of all obstacles are known at all times
 * -No new Obstacles are discovered */
public class OptimalAnyTrajectoryPlanner extends AbstractAnyTrajectoryPlanner {
  private final Collection<Flow> controls;
  // CandidateMap saves neglected/pruned Nodes in a bucket for each domain
  private final Map<Tensor, Set<CandidatePair>> candidateMap = //
      new HashMap<>();

  public OptimalAnyTrajectoryPlanner( //
      Tensor eta, //
      StateIntegrator stateIntegrator, //
      Collection<Flow> controls, //
      TrajectoryRegionQuery obstacleQuery, //
      GoalInterface goalInterface //
  ) {
    super(eta, stateIntegrator, obstacleQuery, goalInterface);
    this.controls = controls;
  }

  @Override // from ExpandInterface
  public void expand(final GlcNode node) {
    // TODO count updates in cell based on costs for benchmarking
    Map<GlcNode, List<StateTime>> connectors = //
        SharedUtils.integrate(node, controls, getStateIntegrator(), goalInterface);
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
    DebugUtils.nodeAmountCheck(getBestOrElsePeek(), node, domainMap().size());
  }

  // TODO BUG: if big numbers of nodes are expanded, nodes =/= domains
  private void processCandidates( //
      GlcNode nextParent, Map<GlcNode, List<StateTime>> connectors, CandidatePairQueueMap candidatePairQueueMap) {
    for (Entry<Tensor, CandidatePairQueue> entry : candidatePairQueueMap.map.entrySet()) {
      final Tensor domainKey = entry.getKey();
      final CandidatePairQueue candidateQueue = entry.getValue();
      if (candidateQueue != null && !getBest().isPresent()) {
        while (!candidateQueue.isEmpty()) {
          // retrieving the Candidates
          final CandidatePair nextCandidatePair = candidateQueue.element();
          final GlcNode formerLabel = getNode(domainKey);
          final GlcNode next = nextCandidatePair.getCandidate();
          if (formerLabel != null) {
            if (Scalars.lessThan(next.merit(), formerLabel.merit())) {
              // collision check only if new node is better
              if (getObstacleQuery().isDisjoint(connectors.get(next))) {// better node not collision
                final Collection<GlcNode> subDeleteTree = deleteSubtreeOf(formerLabel);
                if (subDeleteTree.size() > 1)
                  System.err.println("Pruned Tree of Size: " + subDeleteTree.size());
                // adding the formerLabel as formerCandidate to bucket
                CandidatePair formerCandidate = new CandidatePair(formerLabel.parent(), formerLabel);
                candidateMap.get(domainKey).add(formerCandidate);
                // formerLabel disconnecting
                formerLabel.parent().removeEdgeTo(formerLabel);
                // adding next to tree and DomainMap
                insertNodeInTree(nextParent, next);
                // nextParent.insertEdgeTo(next);
                // final boolean replaced = insert(domainKey, next);
                // if (replaced) {
                // System.err.println("No formerLabel existed, but sth. was replaced");
                // throw new RuntimeException();
                // }
                // removing the nextCandidate from bucket of this domain
                candidateMap.get(domainKey).remove(nextCandidatePair);
                candidateQueue.remove();
                // GOAL check
                if (!goalInterface.isDisjoint(connectors.get(next)))
                  offerDestination(next);
                break;
              }
            }
          } else { // No formerLabel, so definitely adding a Node
            if (getObstacleQuery().isDisjoint(connectors.get(next))) {
              // removing the nextCandidate from bucket of this domain
              // adding next to tree and DomainMap
              nextParent.insertEdgeTo(next);
              boolean replaced = insert(domainKey, next);
              if (replaced) {
                System.err.println("No formerLabel existed, but sth. was replaced");
                throw new RuntimeException();
              }
              candidateMap.get(domainKey).remove(nextCandidatePair);
              candidateQueue.remove();
              // GOAL check
              if (!goalInterface.isDisjoint(connectors.get(next)))
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
    // -- BASIC REROOTING AND DELETING
    // removes the new root from the child list of its parent
    // Disconnecting newRoot from Old Tree and collecting DeleteTree
    newRoot.parent().removeEdgeTo(newRoot);
    Collection<GlcNode> deleteTreeCollection = deleteSubtreeOf(oldRoot);
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
    int candidateMapBeforeSize = candidateMap.size();
    candidateMap.entrySet().parallelStream().forEach( //
        CandidateSet -> CandidateSet.getValue().removeIf(cp -> deleteTreeCollection.contains(cp.getOrigin())));
    candidateMap.values().removeIf(bucket -> bucket.isEmpty());
    System.out.println("CandidateMap before " + candidateMapBeforeSize + " and after: " + candidateMap.size());
    // -- DEBUGING
    long newtotalCandidates = candidateMap.values().stream().flatMap(Collection::stream).count();
    System.out.println(oldtotalCandidates - newtotalCandidates + " of " + oldtotalCandidates + //
        " Candidates removed from CandidateList ");
    // -- RELABELING:
    int addedNodesToQueue = 0;
    for (GlcNode formerLabel : deleteTreeCollection) { // going through domains
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
              getStateIntegrator().trajectory(nextParent.stateTime(), next.flow());
          if (getObstacleQuery().isDisjoint(connector)) { // no collision
            if (formerLabel.parent() != null)
              formerLabel.parent().removeEdgeTo(formerLabel);
            nextParent.insertEdgeTo(next);
            // TODO: check if Candidates are small trees?
            final boolean replaced = insert(domainKey, next);
            if (replaced) {// DomainMap at this key should be empty
              System.out.println("Something was replaced --> BUG");
              throw new RuntimeException();
            }
            candidateMap.get(domainKey).remove(nextCandidatePair);
            candidateQueue.remove();
            addedNodesToQueue++;
            if (!goalInterface.isDisjoint(connector))
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
    // TODO Update cost from root of all nodes in tree
    //
    System.out.println("**Rootswitch finished**");
    return increasedDepthBy;
  }
}
