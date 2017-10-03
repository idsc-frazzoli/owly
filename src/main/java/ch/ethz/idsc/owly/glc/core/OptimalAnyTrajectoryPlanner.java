// code by jl
package ch.ethz.idsc.owly.glc.core;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Objects;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.stream.Collectors;

import ch.ethz.idsc.owly.data.tree.Nodes;
import ch.ethz.idsc.owly.glc.adapter.TrajectoryGoalManager;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.EmptyRegion;
import ch.ethz.idsc.owly.math.region.InvertedRegion;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;

/** An anytime, Resolution asymptotically optimal and probabilistic complete MotionPlanning Algorithm,
 * after: [B. Paden] A Generalized Label Correcting Method for Optimal Kinodynamic Motion Planning
 * Assumptions: -All states of all obstacles are known at all times
 * -No new Obstacles are discovered */
public class OptimalAnyTrajectoryPlanner extends AbstractAnyTrajectoryPlanner {
  // CandidateMap saves neglected/pruned Nodes in a bucket for each domain
  private final Map<Tensor, Set<CandidatePair>> candidateMap = new HashMap<>();

  public OptimalAnyTrajectoryPlanner( //
      Tensor eta, //
      StateIntegrator stateIntegrator, //
      Collection<Flow> controls, //
      TrajectoryRegionQuery obstacleQuery, //
      GoalInterface goalInterface //
  ) {
    super(eta, stateIntegrator, controls, obstacleQuery, goalInterface);
  }

  @Override // from ExpandInterface
  public void expand(final GlcNode node) {
    integratorWatch.start();
    Map<GlcNode, List<StateTime>> connectors = controlsIntegrator.inParallel(node);
    CandidatePairQueueMap candidatePairQueueMap = new CandidatePairQueueMap();
    integratorWatch.stop();
    integratorWatch1.start();
    for (GlcNode next : connectors.keySet()) { // <- order of keys is non-deterministic
      // TODO TimeLoss of 7% in TWD
      // {
      // Scalar oldCost = node.costFromRoot();
      // Scalar oldMerit = node.merit();
      // Scalar oldHeuristic = getGoalInterface().minCostToGoal(node.state());
      // Scalar newCost = next.costFromRoot();
      // Scalar newMerit = next.merit();
      // Scalar newHeuristic = getGoalInterface().minCostToGoal(next.state());
      // if (!Scalars.lessEquals(oldMerit, newMerit)) {
      // System.out.println("oldState: " + node.stateTime().toInfoString());
      // System.out.println("oldCost: " + oldCost);
      // System.out.println("oldHeuri: " + oldHeuristic);
      // System.out.println("oldMerit: " + oldMerit);
      // System.out.println("newState: " + next.stateTime().toInfoString());
      // System.out.println("newCost: " + newCost);
      // System.out.println("newHeuri: " + newHeuristic);
      // System.out.println("newMerit: " + newMerit);
      // throw new RuntimeException();
      // }
      // if (Double.isInfinite(newHeuristic.number().doubleValue()))
      // throw new RuntimeException(" " + newHeuristic);
      // if (Double.isInfinite(newCost.number().doubleValue()))
      // throw new RuntimeException(" " + newCost);
      // }
      // ALL Candidates are saved in temporary CandidateList
      CandidatePair nextCandidate = new CandidatePair(node, next);
      final Tensor domainKey = convertToKey(next.state());
      candidatePairQueueMap.insert(domainKey, nextCandidate);
    }
    integratorWatch1.stop();
    integratorWatch2.start();
    // saving the candidates in the corresponding Buckets
    for (Entry<Tensor, CandidatePairQueue> entry : candidatePairQueueMap.map.entrySet()) {
      if (!getCandidateMap().containsKey(entry.getKey()))
        candidateMap.put(entry.getKey(), new HashSet<CandidatePair>());
      getCandidateMap().get(entry.getKey()).addAll(entry.getValue());
    }
    integratorWatch2.stop();
    processCWatch.start();
    processCandidates(node, connectors, candidatePairQueueMap);
    processCWatch.stop();
  }

  private void processCandidates( //
      GlcNode nextParent, Map<GlcNode, List<StateTime>> connectors, CandidatePairQueueMap candidatePairQueueMap) {
    for (Entry<Tensor, CandidatePairQueue> entry : candidatePairQueueMap.map.entrySet()) {
      final Tensor domainKey = entry.getKey();
      final CandidatePairQueue candidateQueue = entry.getValue();
      if (Objects.nonNull(candidateQueue)) {
        while (!candidateQueue.isEmpty()) {
          // retrieving the Candidates
          final CandidatePair nextCandidatePair = candidateQueue.poll();
          final GlcNode formerLabel = getNode(domainKey);
          final GlcNode next = nextCandidatePair.getCandidate();
          if (Objects.nonNull(formerLabel)) {
            if (Scalars.lessThan(next.merit(), formerLabel.merit())) {
              // collision check only if new node is better
              if (getObstacleQuery().isDisjoint(connectors.get(next))) { // better node not collision
                if (formerLabel.isRoot())
                  throw new RuntimeException();
                // final Collection<GlcNode> subDeleteTree =
                deleteSubtreeOf(formerLabel);
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
                // two lookups in HashSets: one with key Tensor, other with key Object
                candidateMap.get(domainKey).remove(nextCandidatePair);
                // GOAL check
                if (!getGoalInterface().isDisjoint(connectors.get(next)))
                  offerDestination(next, connectors.get(next));
                break;
              }
            } else {
              break;
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
              // two lookups in HashSets: one with key Tensor, other with key Object
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
    System.out.println("*** ROOTSWITCH ***");
    if (newRoot.isRoot()) {
      System.out.println("node is already root");
      return 0;
    }
    long tic = System.nanoTime();
    // needed for change of depthlimit
    int increasedDepthBy = newRoot.depthDifferenceToRoot();
    // -- DEBUGING Values
    // -- Collecting Oldtree
    GlcNode oldRoot = getRoot();
    Collection<GlcNode> oldTreeCollection = Nodes.ofSubtree(oldRoot);
    int oldDomainMapSize = domainMap().size();
    long oldtotalCandidates = getCandidateMap().values().stream().flatMap(Collection::stream).count();
    int oldQueueSize = queue().size();
    // -- BASIC REROOTING AND DELETING
    // removes the new root from the child list of its parent
    // Disconnecting newRoot from Old Tree and collecting DeleteTree
    newRoot.makeRoot();
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
          // check if Delete was properly conducted
          if (deleteTreeCollection.contains(nextParent)) {
            System.err.println("parent of node will be deleted --> not in QUEUE ");
            throw new RuntimeException();
          }
          final List<StateTime> connector = //
              getStateIntegrator().trajectory(nextParent.stateTime(), next.flow());
          if (getObstacleQuery().isDisjoint(connector)) { // no collision
            if (!formerLabel.isRoot())
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
    System.out.println(addedNodesToQueue + " Nodes added to Domain = " + domainMap().size());
    System.out.println("** Rootswitch finished in " + (System.nanoTime() - tic) * 1e-9 + "s **");
    return increasedDepthBy;
  }

  @Override
  public void obstacleUpdate(TrajectoryRegionQuery newObstacle) {
    obstacleUpdate(newObstacle, new InvertedRegion(EmptyRegion.INSTANCE));
  }

  // TODO JAN; obstacle check function
  @Override
  public void obstacleUpdate(TrajectoryRegionQuery newObstacle, Region rechabilityObstacleRegion) {
    if (newObstacle == this.getObstacleQuery())
      return;
    if (newObstacle == null) {
      obstacleUpdate(newObstacle);
      return;
    }
    // TODO detect if no obstacles have moved --> do not check for collision
    long tictotal = System.nanoTime();
    System.out.println("*** OBSTACLE UPDATE ***");
    long tic = System.nanoTime();
    setObstacleQuery(newObstacle);
    GlcNode root = getRoot();
    int oldDomainMapSize = domainMap().size();
    int oldCandidateMapSize = candidateMap.size();
    // TODO JONAS: What to do if root in collision
    // DomainMap, over which it is iterated
    Map<Tensor, GlcNode> iterableDomainMap = new HashMap<Tensor, GlcNode>(domainMap());
    // only iterate through domains which are in the reachability Region, connectivity between nodes is useless,
    // because it does not give us information if something could have changed for ancestors or predecessors
    // only nodes are checked, no integration is performed (will be done on selected Nodes in the reachability set)
    iterableDomainMap.values().removeIf(node -> !(rechabilityObstacleRegion.isMember(node.state())));
    System.out.println("Only checking " + iterableDomainMap.size() + " instead of " + oldDomainMapSize + " domains");
    // sorting as breath first search: lowest depth first
    iterableDomainMap = iterableDomainMap.entrySet()//
        .stream()//
        .sorted(NodeDepthComparator.INSTANCE).collect(Collectors.toMap( //
            Map.Entry::getKey, Map.Entry::getValue, (e1, e2) -> e1, LinkedHashMap::new));
    int deletedNodes = 0;
    int addedNodes = 0;
    // going through domains
    for (Entry<Tensor, GlcNode> entry : iterableDomainMap.entrySet()) {
      GlcNode label = entry.getValue();
      Tensor domainKey = entry.getKey();
      List<StateTime> connector = new ArrayList<>();
      if (getNode(domainKey) == label) { // check if nothing was changed in previous loopiteration, otherwise next domain
        // maybe break into next for loop iteration better?
        PriorityQueue<CandidatePair> candidateQueue = new PriorityQueue<>();
        if (candidateMap.containsKey(domainKey)) { // fills candidatebucket
          Set<CandidatePair> tempCandidateSet = candidateMap.get(domainKey);
          // updating the heuristic of the candidatebucket
          tempCandidateSet.parallelStream().forEach(cp -> //
          cp.getCandidate().setMinCostToGoal(getGoalInterface().minCostToGoal(cp.getCandidate().state())));
          candidateQueue.addAll(tempCandidateSet);
        }
        if (!label.isRoot()) {
          connector = getStateIntegrator().trajectory(label.parent().stateTime(), label.flow());
          if (getObstacleQuery().isDisjoint(connector)) { // label NOT in Collision
            while (!candidateQueue.isEmpty()) { // checking for better candidates/open doors
              CandidatePair nextBest = candidateQueue.element();
              GlcNode nextBestNode = nextBest.getCandidate();
              GlcNode nextBestParent = nextBest.getOrigin();
              if (Scalars.lessThan(nextBestNode.merit(), label.merit())) {
                if (Nodes.areConnected(nextBestParent, root)) {
                  // / check if sth better is there
                  connector = getStateIntegrator().trajectory(nextBestParent.stateTime(), nextBestNode.flow());
                  if (getObstacleQuery().isDisjoint(connector)) {
                    // final Collection<GlcNode> subDeleteTree =
                    Collection<GlcNode> deletedNodesList = deleteSubtreeOf(label);
                    deletedNodes += deletedNodesList.size();
                    CandidatePair formerLabelCandidate = new CandidatePair(label.parent(), label);
                    label.parent().removeEdgeTo(label);
                    if (!formerLabelCandidate.getCandidate().isLeaf()) {
                      System.err.println("The Candidate in the bucket has children");
                      throw new RuntimeException();
                    }
                    getCandidateMap().get(domainKey).add(formerLabelCandidate);
                    // adding next to tree and DomainMap
                    insertNodeInTree(nextBest.getOrigin(), nextBestNode);
                    addedNodes++;
                    // removing the nextCandidate from bucket of this domain as new is label
                    boolean removed = candidateMap.get(domainKey).remove(nextBest);
                    if (!removed)
                      throw new RuntimeException(); // candidate should be in candidatemap
                    break; // leaves whileloop around candidateQueue
                  }
                } else { // CandidateOrigin is not connected to tree anymore --> Remove this Candidate from Map
                  candidateMap.get(domainKey).remove(nextBest);
                }
              } else {
                break;// no better candidate can be found, therefore leave while loop of this CandidateQueue
              }
              final CandidatePair removedCP = candidateQueue.remove();
              if (removedCP != nextBest)
                throw new RuntimeException();
            } // if CandidateQueue is empty finish with this domain
          } else { // DELETE label, as IN collision
            Collection<GlcNode> deletedNodesList = deleteSubtreeOf(label);
            deletedNodes += deletedNodesList.size();
            CandidatePair formerLabelCandidate = new CandidatePair(label.parent(), label);
            label.parent().removeEdgeTo(label);
            if (!formerLabelCandidate.getCandidate().isLeaf()) {
              System.err.println("The Candidate in the bucket has children");
              throw new RuntimeException();
            }
            while (!candidateQueue.isEmpty()) {
              CandidatePair nextBest = candidateQueue.element();
              GlcNode nextBestNode = nextBest.getCandidate();
              GlcNode nextBestParent = nextBest.getOrigin();
              // ******COPY PASTED
              if (Nodes.areConnected(nextBestParent, root)) {
                connector = getStateIntegrator().trajectory(nextBestParent.stateTime(), nextBestNode.flow());
                if (getObstacleQuery().isDisjoint(connector)) { // better Candidate obstacle check
                  getCandidateMap().get(domainKey).add(formerLabelCandidate);
                  // formerLabel disconnecting
                  // adding next to tree and DomainMap
                  insertNodeInTree(nextBest.getOrigin(), nextBestNode);
                  addedNodes++;
                  // removing the nextCandidate from bucket of this domain as new is label
                  boolean removed = candidateMap.get(domainKey).remove(nextBest);
                  if (!removed)
                    throw new RuntimeException(); // candidate should be in candidatemap
                  break; // leaves whileloop around candidateQueue
                }
              } else { // CandidateOrigin is not connected to tree anymore --> Remove this Candidate from Map
                candidateMap.get(domainKey).remove(nextBest);
              }
              final CandidatePair removedCP = candidateQueue.remove();
              if (removedCP != nextBest)
                throw new RuntimeException();
            }
          }
        }
      }
      // GOAL check
      if (!connector.isEmpty() && getNode(domainKey) != null) {
        if (!getGoalInterface().isDisjoint(connector))
          offerDestination(getNode(domainKey), connector);
      }
    }
    long toc = System.nanoTime();
    System.out.println("Obstacle check in DomainMap with now " + domainMap().size() + //
        " domains took :" + (toc - tic) * 1e-9 + "s");
    System.out.println("Nodes added:  " + addedNodes);
    System.out.println("Nodes deleted: " + deletedNodes);
    addedNodes = 0;
    deletedNodes = 0;
    int checkedCandidateDomains = 0;
    tic = toc;
    // go through candidateMap without domain map
    for (Entry<Tensor, Set<CandidatePair>> entry : candidateMap.entrySet()) {
      if (!iterableDomainMap.containsKey(entry.getKey()) && rechabilityObstacleRegion.isMember(entry.getKey())) {// was not checked in run before
        PriorityQueue<CandidatePair> candidateQueue = new PriorityQueue<>();
        checkedCandidateDomains++;
        entry.getValue().parallelStream().forEach(cp -> //
        cp.getCandidate().setMinCostToGoal(getGoalInterface().minCostToGoal(cp.getCandidate().state())));
        candidateQueue.addAll(entry.getValue());
        while (!candidateQueue.isEmpty()) {
          CandidatePair nextBest = candidateQueue.element();
          GlcNode nextBestNode = nextBest.getCandidate();
          Tensor domainKey = convertToKey(nextBestNode.state());
          GlcNode nextBestParent = nextBest.getOrigin();
          if (Nodes.areConnected(nextBestParent, root)) {
            List<StateTime> connector = getStateIntegrator().trajectory(nextBestParent.stateTime(), nextBestNode.flow());
            if (getObstacleQuery().isDisjoint(connector)) { // best Candidate obstacle check
              // adding next to tree and DomainMap
              final CandidatePair removedCP = candidateQueue.remove();
              if (removedCP != nextBest)
                throw new RuntimeException();
              insertNodeInTree(nextBest.getOrigin(), nextBestNode);
              addedNodes++;
              // removing the nextCandidate from bucket of this domain as new is label
              boolean removed = candidateMap.get(domainKey).remove(nextBest);
              if (!removed)
                throw new RuntimeException(); // candidate should be in candidatemap
              // GOAL check
              if (!getGoalInterface().isDisjoint(connector))
                offerDestination(nextBestNode, connector);
              break; // leaves whileloop around candidateQueue
            }
          } else { // CandidateOrigin is not connected to tree anymore --> Remove this Candidate from Map
            candidateMap.get(domainKey).remove(nextBest);
          }
          final CandidatePair removedCP = candidateQueue.remove();
          if (removedCP != nextBest)
            throw new RuntimeException();
        }
      }
    }
    toc = System.nanoTime();
    System.out.println("Obstacle check in Rest of CandidateMap with " + (checkedCandidateDomains)//
        + " domains took :" + (toc - tic) * 1e-9 + "s");
    System.out.println("Nodes added:  " + addedNodes);
    System.out.println("Nodes deleted:" + deletedNodes);
    System.out.println("** ObstacleUpdate finished in " + (System.nanoTime() - tictotal) * 1e-9 + "s **");// Consistency checks
    DebugUtils.nodeAmountCompare(this);
    DebugUtils.connectivityCheck(domainMap().values());
    DebugUtils.heuristicConsistencyCheck(this);
  }

  public Map<Tensor, Set<CandidatePair>> getCandidateMap() {
    return Collections.unmodifiableMap(candidateMap);
  }

  @Override
  public String infoString() {
    StringBuilder stringBuilder = new StringBuilder(super.infoString() + ", ");
    stringBuilder.append("OptimalAnyPlanner");
    if (getGoalInterface() instanceof TrajectoryGoalManager)
      stringBuilder.append(", with a TrajectoryGoalManger");
    return stringBuilder.toString();
  }

  @Override
  /* package */ final void relabelingDomains() {
    GlcNode root = getRoot();
    Map<Tensor, GlcNode> iterableTreeMap = new HashMap<Tensor, GlcNode>(domainMap());
    System.err.println("checking for domainlabel changes due to heuristic change,  Treesize: " + iterableTreeMap.size());
    iterableTreeMap = iterableTreeMap.entrySet()//
        .stream()//
        .sorted(NodeDepthComparator.INSTANCE).collect(Collectors.toMap( //
            Map.Entry::getKey, Map.Entry::getValue, (e1, e2) -> e1, LinkedHashMap::new));
    iterableTreeMap.values().stream().parallel().forEach(glcNode -> glcNode.setMinCostToGoal(getGoalInterface().minCostToGoal(glcNode.state())));
    int deletedNodes = 0;
    int replacedNodes = 0;
    for (Entry<Tensor, GlcNode> entry : iterableTreeMap.entrySet()) {
      // iterating through all Nodes in Tree starting at lowest depth
      GlcNode label = entry.getValue();
      Tensor domainKey = entry.getKey();
      if (getNode(domainKey) == label && !label.isRoot()) { // this node could have been deleted from tree in prev iteration
        // could be relabeled in loopiteration before, therefore no check is needed if changed
        if (getCandidateMap().containsKey(domainKey)) {
          Set<CandidatePair> tempCandidateSet = candidateMap.get(domainKey);
          tempCandidateSet.parallelStream().forEach(cp -> //
          cp.getCandidate().setMinCostToGoal(getGoalInterface().minCostToGoal(cp.getCandidate().state())));
          PriorityQueue<CandidatePair> candidateQueue = new PriorityQueue<>(tempCandidateSet);
          while (!candidateQueue.isEmpty()) {
            final CandidatePair nextCandidatePair = candidateQueue.element();
            final GlcNode possibleCandidateNode = nextCandidatePair.getCandidate();
            if (Scalars.lessThan(possibleCandidateNode.merit(), label.merit())) {
              // collision check only if new node is better
              final GlcNode possibleCandidateOrigin = nextCandidatePair.getOrigin();
              if (Nodes.areConnected(possibleCandidateOrigin, root)) {
                final List<StateTime> connector = //
                    getStateIntegrator().trajectory(possibleCandidateOrigin.stateTime(), possibleCandidateNode.flow());
                if (getObstacleQuery().isDisjoint(connector)) {
                  Collection<GlcNode> deleteTree = deleteSubtreeOf(label);
                  CandidatePair formerLabelCandidate = new CandidatePair(label.parent(), label);
                  label.parent().removeEdgeTo(label);
                  if (!formerLabelCandidate.getCandidate().isLeaf()) {
                    System.err.println("The Candidate in the bucket has children");
                    throw new RuntimeException();
                  }
                  getCandidateMap().get(domainKey).add(formerLabelCandidate);
                  deletedNodes = deletedNodes + deleteTree.size() - 1; // -1 as this node was replaced
                  replacedNodes++;
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
            final CandidatePair removedCP = candidateQueue.remove();
            if (removedCP != nextCandidatePair)
              throw new RuntimeException(); // removed candidate should be nextBest from before
          }
        }
      }
    }
    long candidateMapBeforeSize = candidateMap.size();
    long oldtotalCandidates = candidateMap.values().parallelStream().flatMap(Collection::stream).count();
    candidateMap.entrySet().parallelStream().forEach(candidateSet -> candidateSet.getValue()//
        .removeIf(cp -> !Nodes.rootFrom(cp.getOrigin()).equals(root)));
    // Deleting CandidateBuckets, if they are empty
    candidateMap.values().removeIf(bucket -> bucket.isEmpty());
    long newtotalCandidates = candidateMap.values().parallelStream().flatMap(Collection::stream).count();
    // INFO
    System.out.println("replaced Nodes: " + replacedNodes + " deleted Nodes: " + deletedNodes + ", during relabel");// stopped iterating over tree
    System.out.println(oldtotalCandidates - newtotalCandidates + " of " + oldtotalCandidates + //
        " C. removed from CL. with Origin in deleteTree: " + newtotalCandidates);
    System.out.println("CandidateMap before " + candidateMapBeforeSize + //
        " and after: " + candidateMap.size());
    DebugUtils.nodeAmountCompare(this);
  }

  @Override
  protected final boolean goalCheckTree(final Region possibleGoalNodesRegion) {
    final Collection<GlcNode> treeCollection = Nodes.ofSubtree(getRoot());
    // Smart way: uses 15% -> 30% of the time of normal implementation
    // Smart way: uses 30% -> 40% of the time of parallel implementation
    // , tested with R2GlcAnyCircleDemo
    Collection<GlcNode> possibleGoalNodes = new ArrayList<>();
    possibleGoalNodes = treeCollection.stream().parallel()//
        .filter(node -> possibleGoalNodesRegion.isMember(node.state()))//
        .collect(Collectors.toList());
    // checking only Nodes, which could reach the Goal
    System.out.println("Total Nodes in Tree: " + treeCollection.size() + " possibleGoalNodes: " + possibleGoalNodes.size());
    // --
    return goalCheckTree(possibleGoalNodes);
  }

  private boolean goalCheckTree(Collection<GlcNode> treeCollection) {
    // Parallel: 15%-50% Speedgain, tested with R2GlcConstTimeHeuristicAnyDemo,
    // TODO JONAS/ JAN why does parallel give different result? then non parallel? e.g. R2GlcAnyCircleDemo
    // JONAS still happening at current state
    // No difference with synchronized, same bug, even with exactly the same List
    treeCollection.stream().forEach(node -> {
      if (!node.isRoot()) {
        final List<StateTime> trajectory = getStateIntegrator().trajectory(node.parent().stateTime(), node.flow());
        if (!getGoalInterface().isDisjoint(trajectory))
          offerDestination(node, trajectory);
      }
    });
    return getBest().isPresent();
  }
}
