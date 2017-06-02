// code by jl
package ch.ethz.idsc.owly.glc.core;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
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
import ch.ethz.idsc.owly.math.state.Trajectories;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.ZeroScalar;

/** TODO assumptions in order to use any... */
public class AnyTrajectoryPlanner extends TrajectoryPlanner {
  private final StateIntegrator stateIntegrator;
  private final Collection<Flow> controls;
  private CostFunction costFunction;
  private TrajectoryRegionQuery goalQuery;
  private final TrajectoryRegionQuery obstacleQuery;
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
    super(eta);
    this.stateIntegrator = stateIntegrator;
    this.controls = controls;
    this.costFunction = costFunction;
    this.goalQuery = goalQuery;
    this.obstacleQuery = obstacleQuery;
  }

  @Override // from ExpandInterface
  public void expand(final GlcNode node) {
    // TODO count updates in cell based on costs for benchmarking
    Map<GlcNode, List<StateTime>> connectors = //
        SharedUtils.integrate(node, controls, stateIntegrator, costFunction);
    CandidatePairQueueMap candidates = new CandidatePairQueueMap();
    for (GlcNode next : connectors.keySet()) { // <- order of keys is non-deterministic
      // ALL Candidates are saved in CandidateList
      CandidatePair nextCandidate = new CandidatePair(node, next);
      final Tensor domain_key = convertToKey(next.state());
      candidates.insert(domain_key, nextCandidate);
    }
    for (Entry<Tensor, CandidatePairQueue> entry : candidates.map.entrySet()) {
      if (!candidateMap.containsKey(entry.getKey()))
        candidateMap.put(entry.getKey(), new HashSet<CandidatePair>());
      candidateMap.get(entry.getKey()).addAll(entry.getValue());
    }
    processCandidates(node, connectors, candidates);
  }

  private void processCandidates( //
      GlcNode node, Map<GlcNode, List<StateTime>> connectors, CandidatePairQueueMap candidates) {
    for (Entry<Tensor, CandidatePairQueue> entry : candidates.map.entrySet()) {
      final Tensor domain_key = entry.getKey();
      final CandidatePairQueue candidateQueue = entry.getValue();
      if (candidateQueue != null && best == null) {
        // while (Candidatesleft > 0) {
        while (!candidateQueue.isEmpty()) {
          final CandidatePair nextCandidatePair = candidateQueue.element();
          final GlcNode formerLabel = getNode(domain_key);
          final GlcNode next = nextCandidatePair.getCandidate();
          if (formerLabel != null) {
            if (Scalars.lessThan(next.merit(), formerLabel.merit())) {
              // collision check only if new node is better
              if (obstacleQuery.isDisjoint(connectors.get(next))) {// better node not collision
                // current label back in Candidatelist, is also in Queue
                CandidatePair formerCandidate = new CandidatePair(formerLabel.parent(), formerLabel);
                candidateMap.get(domain_key).add(formerCandidate);
                // removing the nextCandidtae from candidate list from this domain
                candidateMap.get(domain_key).remove(nextCandidatePair);
                // current label disconnecting
                // 15:30
                formerLabel.parent().removeEdgeTo(formerLabel);
                // adding next to tree and DomainMap
                node.insertEdgeTo(next);
                insert(domain_key, next);
                // GOAL check
                if (!goalQuery.isDisjoint(connectors.get(next)))
                  offerDestination(next);
                // remove next from DomainQueue
                candidateQueue.remove();
                break;
              }
            }
          } else {
            if (obstacleQuery.isDisjoint(connectors.get(next))) {
              node.insertEdgeTo(next);
              insert(domain_key, next);
              if (!goalQuery.isDisjoint(connectors.get(next)))
                offerDestination(next);
              candidateQueue.remove();
              break;
            }
          }
          candidateQueue.remove();// remove from Queue as was not better
        }
      }
    }
  }

  /** @param state the new Rootstate
   * @return The value,by which the depth limit needs to be increased as of the RootSwitch */
  public int switchRootToState(Tensor state) {
    GlcNode newRoot = this.getNode(convertToKey(state));
    // TODO not nice, as we jump from state to startnode
    if (newRoot != null)
      return switchRootToNode(newRoot);
    else { // TODO WHY dead Code?
      System.out.println("This domain is not labelled yet");
      return 0;
    }
  }

  public int switchRootToNode(GlcNode newRoot) {
    if (newRoot.isRoot()) {
      System.out.println("node is already root");
      return 0;
    }
    int increasedDepthBy = newRoot.reCalculateDepth();
    // Collecting Oldtree
    GlcNode oldRoot = getNodesfromRootToGoal().get(0);
    Collection<GlcNode> oldTreeCollection = Nodes.ofSubtree(oldRoot);
    // DEBUGING Values
    int oldDomainMapSize = domainMap().size();
    long oldtotalCandidates = candidateMap.values().stream().flatMap(Collection::stream).count();
    int oldQueueSize = queue().size();
    int nodesInMap = 0;
    int keyNotInMap = 0;
    int wrongObjectInMap = 0;
    Set<GlcNode> DebugCollection = new HashSet<>();
    for (GlcNode tempNode : oldTreeCollection) {
      Tensor tempKey = convertToKey(tempNode.state());
      if (domainMap().containsKey(tempKey)) {
        nodesInMap++;
        if (!domainMap().get(tempKey).equals(tempNode)) {
          wrongObjectInMap++;
          DebugCollection.add(tempNode);
        }
      } else
        keyNotInMap++;
    }
    for (GlcNode tempNode : DebugCollection) {
      System.out.println("additional Nodes not in a Domain: state: " + tempNode.state() + "cost" + tempNode.costFromRoot());
      if (!tempNode.isRoot())
        System.out.println("has a parent");
      if (tempNode.isLeaf())
        System.out.println("is leaf");
      if (tempNode.flow() != null)
        System.out.println("has flow");
    }
    System.out.println("Nodes In Map: " + nodesInMap + //
        " /Nodes with no key in map: " + keyNotInMap + " /wrong object at key: " + wrongObjectInMap);
    // removes the new root from the child list of its parent
    // Disconnecting newRoot and collecting DeleteTree
    newRoot.parent().removeEdgeTo(newRoot);
    Collection<GlcNode> deleteTreeCollection = Nodes.ofSubtree(oldRoot);
    // GOAL: goal deleted?
    if (best != null) {
      if (deleteTreeCollection.contains(best))
        best = null;
    }
    System.out.println("Nodes to be deleted: " + deleteTreeCollection.size());
    // QUEUE: Deleting Nodes from Queue
    // TODO paraliszable?
    if (queue().removeAll(deleteTreeCollection))
      System.out.println("Removed " + (oldQueueSize - queue().size()) + " out of " + oldQueueSize + " nodes from Queue = " + queue().size());
    // EDGE: Removing Edges between Nodes in DeleteTree
    // TODO paralizable?
    for (GlcNode tempNode : deleteTreeCollection) {
      if (!tempNode.isRoot())
        tempNode.parent().removeEdgeTo(tempNode);
    }
    // DOMAINMAP: Removing Nodes (DeleteTree) from DomainMap
    domainMap().values().removeAll(deleteTreeCollection);
    // TODO Domain map grows unboundly
    // DEBUGING
    Collection<GlcNode> newTreeCollection = Nodes.ofSubtree(getNodesfromRootToGoal().get(0));
    System.out.println(oldDomainMapSize - domainMap().size() + " out of " + oldDomainMapSize + //
        " Domains removed from DomainMap = " + domainMap().size());
    System.out.println(deleteTreeCollection.size() + " out of " + oldTreeCollection.size() + " Nodes removed from Tree = " + newTreeCollection.size());
    // CANDIDATEMAP: Deleting Candidates, if Origin includes in DeleteTree
    // TODO What is the time gain by parallization?
    candidateMap.entrySet().parallelStream().forEach( //
        CandidateSet -> CandidateSet.getValue().removeIf(cp -> deleteTreeCollection.contains(cp.getOrigin())));
    // DEBUGING
    long newtotalCandidates = candidateMap.values().stream().flatMap(Collection::stream).count();
    System.out.println(oldtotalCandidates - newtotalCandidates + " of " + oldtotalCandidates + //
        " Candidates removed from CandidateList ");
    // RELABELING:
    int addedNodesToQueue = 0;
    for (GlcNode tempLabel : deleteTreeCollection) {
      Tensor tempDomainKey = convertToKey(tempLabel.state());
      if (candidateMap.containsKey(tempDomainKey)) {
        Set<CandidatePair> tempCandidateSet = candidateMap.get(tempDomainKey);
        // TODO modify constructor of CandidatePairQueue
        // CandidatePairQueue candidateQueue = new CandidatePairQueue(tempCandidateSet);
        PriorityQueue<CandidatePair> candidateQueue = new PriorityQueue<>(tempCandidateSet);
        while (!candidateQueue.isEmpty()) {
          final CandidatePair nextCandidate = candidateQueue.element();
          final GlcNode next = nextCandidate.getCandidate();
          final GlcNode nextParent = nextCandidate.getOrigin();
          // Double check, if Candidatedeleting failes
          if (deleteTreeCollection.contains(nextParent)) {
            System.out.println("parent of node will be deleted --> not in QUEUE ");
            break;
          }
          final List<StateTime> trajectory = //
              stateIntegrator.trajectory(nextParent.stateTime(), next.flow());
          if (obstacleQuery.isDisjoint(trajectory)) { // no collision
            nextParent.insertEdgeTo(next);
            // DomainMap at this key shoudl ne empty
            if (!insert(tempDomainKey, next))
              System.out.println("Something was replaced --> BUG");
            addedNodesToQueue++;
            if (!goalQuery.isDisjoint(trajectory))
              offerDestination(next);
            break; // leaves the while loop, but not the for loop
          }
          candidateQueue.remove();
        }
      }
    }
    System.out.println(addedNodesToQueue + " Nodes added to Queue");
    System.out.println("Domains in DomainMap AFTER RELABEL = " + domainMap().size());
    System.out.println("**Rootswitch finished**");
    return increasedDepthBy;
  }

  @Override
  protected GlcNode createRootNode(Tensor x) { // TODO check if time of root node should always be set to 0
    return new GlcNode(null, new StateTime(x, ZeroScalar.get()), ZeroScalar.get(), //
        costFunction.minCostToGoal(x));
  }

  @Override
  public List<StateTime> detailedTrajectoryTo(GlcNode node) {
    return Trajectories.connect(stateIntegrator, Nodes.fromRoot(node));
  }

  @Override
  public TrajectoryRegionQuery getObstacleQuery() {
    return obstacleQuery;
  }

  @Override
  public TrajectoryRegionQuery getGoalQuery() {
    return goalQuery;
  }

  /** @param newCostFunction is the new CostFunction to the new Goal
   * @param newGoal newGoal is the new RegionQuery for the new Goalregion */
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
    // TODO am I deleting the Node? or just kiling the pointer
    best = null;
    // Checking if goal is already in tree
    // TODO check if tree has right size TreeCollection
    {
      long tic = System.nanoTime();
      Collection<GlcNode> TreeCollection = Nodes.ofSubtree(getNodesfromRootToGoal().get(0));
      System.out.println("treesize for goal checking: " + TreeCollection.size());
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
    System.out.println("Updated Merit of Queue with " + list.size() + " nodes in: " //
        + ((toc - tic) * 1e-9) + "s");
    System.out.println("Goal switch finished");
  }
}
