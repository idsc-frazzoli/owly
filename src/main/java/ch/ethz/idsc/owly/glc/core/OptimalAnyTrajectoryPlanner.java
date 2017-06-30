// code by jl
package ch.ethz.idsc.owly.glc.core;

import java.util.Collection;
import java.util.Collections;
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
    // DebugUtils.nodeAmountCheck(getBestOrElsePeek(), node, domainMap().size());
  }

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
                  // TODO add leafs of Subtree to Queue instead of deleting subtree
                  // not needed for optimality
                  System.err.println("Pruned Tree of Size: " + subDeleteTree.size());
                // adding the formerLabel as formerCandidate to bucket
                CandidatePair formerCandidate = new CandidatePair(formerLabel.parent(), formerLabel);
                if (!formerCandidate.getCandidate().isLeaf()) {
                  System.err.println("The Candidate in the bucket has children");
                  throw new RuntimeException();
                }
                candidateMap.get(domainKey).add(formerCandidate);
                // formerLabel disconnecting
                if (formerLabel.parent() != null)
                  formerLabel.parent().removeEdgeTo(formerLabel);
                // adding next to tree and DomainMap
                insertNodeInTree(nextParent, next);
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
          // remove from TemporaryQueue as was not better/or in Collision
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
    GlcNode oldRoot = getRoot();
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
    System.out.println("Removed " + (oldQueueSize - queue().size()) + " out of " + oldQueueSize + //
        " Nodes from Queue = " + queue().size());
    // --
    System.out.println(oldDomainMapSize - domainMap().size() + " out of " + oldDomainMapSize + //
        " Domains removed from DomainMap = " + domainMap().size());
    final GlcNode rootNode = getRoot();
    Collection<GlcNode> newTreeCollection = Nodes.ofSubtree(rootNode);
    System.out.println(deleteTreeCollection.size() + " out of " + oldTreeCollection.size()//
        + " Nodes removed from Tree = " + newTreeCollection.size());
    int candidateMapBeforeSize = candidateMap.size();
    // --
    // -- CANDIDATEMAP:
    // Deleting Candidates, if origin is not connected to root
    candidateMap.entrySet().parallelStream().forEach(candidateSet -> candidateSet.getValue()//
        .removeIf(cp -> !Nodes.rootFrom(cp.getOrigin()).equals(rootNode)));
    // Deleting CandidateBuckets, if they are empty
    candidateMap.values().removeIf(bucket -> bucket.isEmpty());
    // -- DEBUGING
    long newtotalCandidates = candidateMap.values().parallelStream().flatMap(Collection::stream).count();
    System.out.println(oldtotalCandidates - newtotalCandidates + " of " + oldtotalCandidates + //
        " C. removed from CL. with Origin in deleteTree " + newtotalCandidates);
    System.out.println("CandidateMap before " + candidateMapBeforeSize + //
        " and after: " + candidateMap.size());
    // --
    // -- RELABELING:
    int addedNodesToQueue = 0;
    for (GlcNode formerLabel : deleteTreeCollection) { // going through domains
      Tensor domainKey = convertToKey(formerLabel.state());
      // if a bucket exists do Relabeling otherwise not
      if (candidateMap.containsKey(domainKey)) {
        Set<CandidatePair> tempCandidateSet = candidateMap.get(domainKey);
        // Merit of Candidates can have changed--> update
        tempCandidateSet.parallelStream().forEach(cp -> //
        cp.getCandidate().setMinCostToGoal(goalInterface.minCostToGoal(cp.getCandidate().state())));
        // TODO modify constructor of CandidatePairQueue so below statement works
        // CandidatePairQueue candidateQueue = new CandidatePairQueue(tempCandidateSet);
        PriorityQueue<CandidatePair> candidateQueue = new PriorityQueue<>(tempCandidateSet);
        while (!candidateQueue.isEmpty()) {
          // retrieving the Candidates
          final CandidatePair nextCandidatePair = candidateQueue.element();
          final GlcNode next = nextCandidatePair.getCandidate();
          final GlcNode nextParent = nextCandidatePair.getOrigin();
          // CandidateOrigin needs to be part of tree
          if (!nextParent.isRoot()) {
            // check if Delete was properly conducted
            if (deleteTreeCollection.contains(nextParent)) {
              System.err.println("parent of node will be deleted --> not in QUEUE ");
              throw new RuntimeException();
            }
            final List<StateTime> connector = //
                getStateIntegrator().trajectory(nextParent.stateTime(), next.flow());
            if (getObstacleQuery().isDisjoint(connector)) { // no collision
              if (formerLabel.parent() != null)
                formerLabel.parent().removeEdgeTo(formerLabel);
              insertNodeInTree(nextParent, next);
              candidateMap.get(domainKey).remove(nextCandidatePair);
              candidateQueue.remove();
              // BUG
              addedNodesToQueue++;
              if (!goalInterface.isDisjoint(connector))
                offerDestination(next);
              break; // leaves the while loop, but not the for loop
            }
          }
          // remove from TemporaryQueue as was not better/or in Collision
          candidateQueue.remove();
        }
      }
    }
    System.out.println(addedNodesToQueue + " Nodes added to Domain = " + domainMap().size());
    System.out.println("**Rootswitch finished**");
    return increasedDepthBy;
  }

  public Map<Tensor, Set<CandidatePair>> getCandidateMap() {
    return Collections.unmodifiableMap(candidateMap);
  }

  @Override
  public String infoString() {
    StringBuilder stringBuilder = new StringBuilder(super.infoString() + ", ");
    stringBuilder.append("OptimalAnyPlanner");
    return stringBuilder.toString();
  }
}
