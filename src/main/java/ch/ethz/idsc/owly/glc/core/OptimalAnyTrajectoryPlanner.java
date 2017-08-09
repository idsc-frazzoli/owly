// code by jl
package ch.ethz.idsc.owly.glc.core;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.stream.Collectors;

import ch.ethz.idsc.owly.data.tree.Nodes;
import ch.ethz.idsc.owly.glc.adapter.TrajectoryGoalManager;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.Region;
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
        SharedUtils.integrate(node, controls, getStateIntegrator(), getGoalInterface(), true);
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
  }

  private void processCandidates( //
      GlcNode nextParent, Map<GlcNode, List<StateTime>> connectors, CandidatePairQueueMap candidatePairQueueMap) {
    for (Entry<Tensor, CandidatePairQueue> entry : candidatePairQueueMap.map.entrySet()) {
      final Tensor domainKey = entry.getKey();
      final CandidatePairQueue candidateQueue = entry.getValue();
      if (candidateQueue != null) {
        while (!candidateQueue.isEmpty()) {
          // retrieving the Candidates
          final CandidatePair nextCandidatePair = candidateQueue.poll();
          final GlcNode formerLabel = getNode(domainKey);
          final GlcNode next = nextCandidatePair.getCandidate();
          if (formerLabel != null) {
            if (Scalars.lessThan(next.merit(), formerLabel.merit())) {
              // collision check only if new node is better
              if (getObstacleQuery().isDisjoint(connectors.get(next))) {// better node not collision
                // TODO Needs to be checked with theory, removal from queue is unsure.
                final Collection<GlcNode> subDeleteTree = deleteSubtreeOf(formerLabel);
                if (subDeleteTree.size() > 1)
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
                // removing the nextCandidate from bucket of this domain as new is label
                candidateMap.get(domainKey).remove(nextCandidatePair);
                // GOAL check
                if (!getGoalInterface().isDisjoint(connectors.get(next)))
                  offerDestination(next, connectors.get(next));
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
              // removing the nextCandidate from bucket of this domain as new is label
              candidateMap.get(domainKey).remove(nextCandidatePair);
              // GOAL check
              if (!getGoalInterface().isDisjoint(connectors.get(next)))
                offerDestination(next, connectors.get(next));
              break;
            }
          }
          // remove from TemporaryQueue as was not better/or in Collision
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
        " C. removed from CL. with Origin in deleteTree: " + newtotalCandidates);
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
        cp.getCandidate().setMinCostToGoal(getGoalInterface().minCostToGoal(cp.getCandidate().state())));
        PriorityQueue<CandidatePair> candidateQueue = new PriorityQueue<>(tempCandidateSet);
        while (!candidateQueue.isEmpty()) {
          // retrieving the Candidates
          final CandidatePair nextCandidatePair = candidateQueue.poll();
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
              addedNodesToQueue++;
              if (!getGoalInterface().isDisjoint(connector))
                offerDestination(next, connector);
              break; // leaves the while loop, but not the for loop
            }
          }
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
    if (getGoalQuery() instanceof TrajectoryGoalManager)
      stringBuilder.append(", with a TrajectoryGoalManger");
    return stringBuilder.toString();
  }

  @Override
  void RelabelingDomains() {
    GlcNode root = getRoot();
    List<GlcNode> treeList = new ArrayList<GlcNode>(Nodes.ofSubtree(root));
    Collections.sort(treeList, NodeDepthComparator.INSTANCE);
    int deletedNodes = 0;
    int replacedNodes = 0;
    for (GlcNode current : treeList) {
      // iterating through all Nodes in Tree starting at lowest depth
      Tensor domainKey = convertToKey(current.state());
      if (getNode(domainKey) != null) { // this node could have been deleted from tree in prev iteration
        if (!getNode(domainKey).equals(current)) {
          throw new RuntimeException(); // current should be labeling its own domain
        }
        if (candidateMap.containsKey(domainKey)) {
          Set<CandidatePair> tempCandidateSet = candidateMap.get(domainKey);
          tempCandidateSet.parallelStream().forEach(cp -> //
          cp.getCandidate().setMinCostToGoal(getGoalInterface().minCostToGoal(cp.getCandidate().state())));
          PriorityQueue<CandidatePair> candidateQueue = new PriorityQueue<>(tempCandidateSet);
          while (!candidateQueue.isEmpty()) {
            final CandidatePair nextCandidatePair = candidateQueue.poll();
            final GlcNode possibleCandidateNode = nextCandidatePair.getCandidate();
            if (Scalars.lessThan(possibleCandidateNode.merit(), current.merit())) {
              // collision check only if new node is better
              final GlcNode possibleCandidateOrigin = nextCandidatePair.getOrigin();
              if (Nodes.listFromRoot(possibleCandidateOrigin).get(0) == root) {
                final List<StateTime> connector = //
                    getStateIntegrator().trajectory(possibleCandidateOrigin.stateTime(), possibleCandidateNode.flow());
                if (getObstacleQuery().isDisjoint(connector)) {
                  Collection<GlcNode> deleteTree = deleteSubtreeOf(current);
                  deletedNodes = deletedNodes + deleteTree.size() - 1; // -1 as this node was replaced
                  replacedNodes++;
                  if (current.parent() != null)
                    current.parent().removeEdgeTo(current);
                  insertNodeInTree(possibleCandidateOrigin, possibleCandidateNode);
                  candidateMap.get(domainKey).remove(nextCandidatePair);
                  if (!getGoalInterface().isDisjoint(connector))
                    offerDestination(possibleCandidateNode, connector);
                  break; // leaves the Candidate Queue while loop if a better was found
                }
              } else { // CandidateOrigin is not connected to tree anymore --> Remove this Candidate from Map
                candidateMap.get(convertToKey(possibleCandidateNode.state())).remove(nextCandidatePair);
              }
            } else {
              break; // if no better Candidates are found leave while of Candidates loop -> Speedgain
            }
          }
        }
      }
    }
    long candidateMapBeforeSize = candidateMap.size();
    long oldtotalCandidates = candidateMap.values().parallelStream().flatMap(Collection::stream).count();
    System.out.println("replaced Nodes: " + replacedNodes + " deleted Nodes: " + deletedNodes + ", during relabel");// stopped iterating over tree
    candidateMap.entrySet().parallelStream().forEach(candidateSet -> candidateSet.getValue()//
        .removeIf(cp -> !Nodes.rootFrom(cp.getOrigin()).equals(root)));
    // Deleting CandidateBuckets, if they are empty
    candidateMap.values().removeIf(bucket -> bucket.isEmpty());
    long newtotalCandidates = candidateMap.values().parallelStream().flatMap(Collection::stream).count();
    System.out.println(oldtotalCandidates - newtotalCandidates + " of " + oldtotalCandidates + //
        " C. removed from CL. with Origin in deleteTree: " + newtotalCandidates);
    System.out.println("CandidateMap before " + candidateMapBeforeSize + //
        " and after: " + candidateMap.size());
  }

  @Override
  protected boolean GoalCheckTree(final Collection<GlcNode> treeCollection, final Region possibleGoalNodesRegion) {
    // Smart way: uses 15% -> 30% of the time of normal implemenation
    // Smart way: uses 30% -> 40% of the time of paralel implementation
    // , tested with R2GlcAnyCircleDemo
    List<GlcNode> possibleGoalNodes = new ArrayList<>();
    possibleGoalNodes = treeCollection.stream().parallel()//
        .filter(node -> possibleGoalNodesRegion.isMember(node.state()))//
        .collect(Collectors.toList());
    // checking only Nodes, which could reach the Goal
    System.out.println("Total Nodes in Tree: " + treeCollection.size() + " possibleGoalNodes: " + possibleGoalNodes.size());
    return GoalCheckTree(possibleGoalNodes);
  }

  @Override
  protected boolean GoalCheckTree(final Collection<GlcNode> treeCollection) {
    // Parallel: 15%-50% Speedgain, tested with R2GlcConstTimeHeuristicAnyDemo,
    // TODO JAN: why does parallel give different result? then non parallel? e.g. R2GlcAnyCircleDemo
    treeCollection.stream().forEach(node -> {
      if (!node.isRoot()) {
        final List<StateTime> trajectory = getStateIntegrator().trajectory(node.parent().stateTime(), node.flow());
        if (!getGoalQuery().isDisjoint(trajectory))
          offerDestination(node, trajectory);
      }
    });
    return getBest().isPresent();
  }
}
