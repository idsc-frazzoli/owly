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
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;

/** TODO assumptions in order to use any... */
public class AnyTrajectoryPlanner extends TrajectoryPlanner {
  private final StateIntegrator stateIntegrator;
  private final Collection<Flow> controls;
  private CostFunction costFunction;
  private TrajectoryRegionQuery goalQuery;
  private final TrajectoryRegionQuery obstacleQuery;
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
      // ALL Candidates are saved in temporaray CandidateList
      CandidatePair nextCandidate = new CandidatePair(node, next);
      final Tensor domain_key = convertToKey(next.state());
      candidates.insert(domain_key, nextCandidate);
    }
    // saving the candidates in the corresponding Buckets
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
          // retrieving the Candidates
          final CandidatePair nextCandidatePair = candidateQueue.element();
          final GlcNode formerLabel = getNode(domain_key);
          final GlcNode next = nextCandidatePair.getCandidate();
          if (formerLabel != null) {
            if (Scalars.lessThan(next.merit(), formerLabel.merit())) {
              // collision check only if new node is better
              if (obstacleQuery.isDisjoint(connectors.get(next))) {// better node not collision
                // current label back in bucket fo this domains,
                CandidatePair formerCandidate = new CandidatePair(formerLabel.parent(), formerLabel);
                candidateMap.get(domain_key).add(formerCandidate);
                // Removing the Candidate from the Queue, if in it
                queue().remove(formerLabel);
                // removing the nextCandidate from bucket of this domain
                candidateMap.get(domain_key).remove(nextCandidatePair);
                // current label disconnecting
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
          candidateQueue.remove();// remove from TemporyQueue as was not better
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
    else {
      System.out.println("This domain is not labelled yet");
      return 0;
    }
  }

  public int switchRootToNode(GlcNode newRoot) {
    if (newRoot.isRoot()) {
      System.out.println("node is already root");
      return 0;
    }
    // needed for change of depthlimit
    int increasedDepthBy = newRoot.reCalculateDepth();
    // Collecting Oldtree
    GlcNode oldRoot = getNodesfromRootToGoal().get(0);
    Collection<GlcNode> oldTreeCollection = Nodes.ofSubtree(oldRoot);
    // DEBUGING Values
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
    System.out.println(nodesWithoutDomain + " Nodes have no Domain");
    if (nodesWithoutDomain != 0)
      throw new RuntimeException();
    // removes the new root from the child list of its parent
    // Disconnecting newRoot from Old Tree and collecting DeleteTree
    newRoot.parent().removeEdgeTo(newRoot);
    Collection<GlcNode> deleteTreeCollection = Nodes.ofSubtree(oldRoot);
    // GOAL: goal deleted?
    if (best != null) {
      if (deleteTreeCollection.contains(best))
        best = null;
    }
    System.out.println("Nodes to be deleted: " + deleteTreeCollection.size());
    // QUEUE: Deleting Nodes from Queue
    if (queue().removeAll(deleteTreeCollection))
      System.out.println("Removed " + (oldQueueSize - queue().size()) + " out of " + oldQueueSize + " nodes from Queue = " + queue().size());
    // DOMAINMAP: Removing Nodes (DeleteTree) from DomainMap
    domainMap().values().removeAll(deleteTreeCollection);
    // TODO Domain map grows unboundly
    // EDGE: Removing Edges between Nodes in DeleteTree
    // for (GlcNode tempNode : deleteTreeCollection) {
    // if (!tempNode.isRoot())
    // tempNode.parent().removeEdgeTo(tempNode);
    // }
    // for Null error of root
    // TODO: edge removal Needed?
    // oldRoot has no parent, therefore is skipped
    deleteTreeCollection.remove(oldRoot);
    // TODO: parralizable?
    deleteTreeCollection.forEach(tempNode -> tempNode.parent().removeEdgeTo(tempNode));
    deleteTreeCollection.add(oldRoot);
    // DEBUGING
    Collection<GlcNode> newTreeCollection = Nodes.ofSubtree(getNodesfromRootToGoal().get(0));
    System.out.println(oldDomainMapSize - domainMap().size() + " out of " + oldDomainMapSize + //
        " Domains removed from DomainMap = " + domainMap().size());
    System.out.println(deleteTreeCollection.size() + " out of " + oldTreeCollection.size()//
        + " Nodes removed from Tree = " + newTreeCollection.size());
    // CANDIDATEMAP: Deleting Candidates, if Origin is included in DeleteTree
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
      // if a bucket exists do Relabeling otherwise not
      if (candidateMap.containsKey(tempDomainKey)) {
        Set<CandidatePair> tempCandidateSet = candidateMap.get(tempDomainKey);
        // TODO modify constructor of CandidatePairQueue so below statement works
        // CandidatePairQueue candidateQueue = new CandidatePairQueue(tempCandidateSet);
        PriorityQueue<CandidatePair> candidateQueue = new PriorityQueue<>(tempCandidateSet);
        while (!candidateQueue.isEmpty()) {
          final CandidatePair nextCandidate = candidateQueue.element();
          final GlcNode next = nextCandidate.getCandidate();
          final GlcNode nextParent = nextCandidate.getOrigin();
          // check if Delete was properly conducted
          if (deleteTreeCollection.contains(nextParent)) {
            System.out.println("parent of node will be deleted --> not in QUEUE ");
            break;
          }
          final List<StateTime> trajectory = //
              stateIntegrator.trajectory(nextParent.stateTime(), next.flow());
          if (obstacleQuery.isDisjoint(trajectory)) { // no collision
            nextParent.insertEdgeTo(next);
            // DomainMap at this key should not be sempty
            // TODO do not put data structure changing statement in if clause!
            if (!insert(tempDomainKey, next))
              System.out.println("Something was replaced --> BUG");
            addedNodesToQueue++;
            if (!goalQuery.isDisjoint(trajectory))
              offerDestination(next);
            break; // leaves the while loop, but not the for loop
          }
          // TODO: remove in collision candidates from CandidateMAP?
          candidateQueue.remove();// remove from TemporyQueue as was not better/or in Collision
        }
      }
    }
    System.out.println(addedNodesToQueue + " Nodes added to Domain = " + domainMap().size());
    System.out.println("**Rootswitch finished**");
    return increasedDepthBy;
  }

  @Override
  protected GlcNode createRootNode(Tensor x) { // TODO check if time of root node should always be set to 0
    return new GlcNode(null, new StateTime(x, RealScalar.ZERO), RealScalar.ZERO, //
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
    // Checking if current best is still in GOAL
    // TODO needed? as tree check will find it anyways, (maybe better best)
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
    queue().addAll(list);
    long toc = System.nanoTime();
    System.out.println("Updated Merit of Queue with " + list.size() + " nodes in: " //
        + ((toc - tic) * 1e-9) + "s");
    System.out.println("**Goalswitch finished**");
  }
}
