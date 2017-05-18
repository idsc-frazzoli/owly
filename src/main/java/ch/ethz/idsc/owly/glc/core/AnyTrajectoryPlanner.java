// code by bapaden and jph and jl
package ch.ethz.idsc.owly.glc.core;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.PriorityQueue;

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
  private final Map<Tensor, Collection<CandidatePair>> candidateMap = //
      new HashMap<>();
  // private final Map<Tensor, DomainQueue> domainCandidateMap = new HashMap<>();

  // private final Queue<Node> queue = new PriorityQueue<>(NodeMeritComparator.instance);
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

  @Override
  public void expand(final GlcNode node) {
    // TODO count updates in cell based on costs for benchmarking
    Map<GlcNode, List<StateTime>> connectors = //
        SharedUtils.integrate(node, controls, stateIntegrator, costFunction);
    // Set<Tensor> domainsNeedingUpdate = new HashSet<>();
    // Map<Tensor, DomainQueue> candidates = new HashMap<>();
    Map<Tensor, PriorityQueue<CandidatePair>> candidates = new HashMap<>();
    for (GlcNode next : connectors.keySet()) { // <- order of keys is non-deterministic
      // final List<StateTime> trajectory = stateIntegrator.trajectory(node.stateTime(), flow);
      // final StateTime last = Trajectories.getLast(trajectory);
      // final GlcNode next = new GlcNode(flow, last, //
      // node.costFromRoot().add(costFunction.costIncrement(node.stateTime(), trajectory, flow)), //
      // costFunction.minCostToGoal(last.x()));
      CandidatePair nextCandidate = new CandidatePair(node, next);
      final Tensor domain_key = convertToKey(next.stateTime().x());
      final GlcNode former = getNode(domain_key);
      if (former != null) {
        // if (Scalars.lessThan(next.costFromRoot(), former.costFromRoot())) // new node is better than previous one// already some node present from previous
        // exploration
        if (candidates.containsKey(domain_key))
          candidates.get(domain_key).add(nextCandidate);
        else {
          candidates.put(domain_key, new PriorityQueue<CandidatePair>());
          candidates.get(domain_key).add(nextCandidate);
        }
      } else {// No node present in domain
        candidates.put(domain_key, new PriorityQueue<CandidatePair>());
        candidates.get(domain_key).add(nextCandidate);
      }
    }
    // System.out.println("Size of candidates " + candidates.entrySet().size());
    for (Entry<Tensor, PriorityQueue<CandidatePair>> entry : candidates.entrySet()) {
      if (!candidateMap.containsKey(entry.getKey()))
        candidateMap.put(entry.getKey(), new LinkedList<>());
      candidateMap.get(entry.getKey()).addAll(entry.getValue());
    }
    // System.out.println("Size of candidatesMap " + candidateMap.entrySet().size());
    processCandidates(node, connectors, candidates);
  }

  private void processCandidates( //
      GlcNode node, Map<GlcNode, List<StateTime>> connectors, Map<Tensor, PriorityQueue<CandidatePair>> candidates) {
    for (Entry<Tensor, PriorityQueue<CandidatePair>> entry : candidates.entrySet()) { // parallel
      final Tensor domain_key = entry.getKey();
      final PriorityQueue<CandidatePair> domainCandidateQueue = entry.getValue();
      if (domainCandidateQueue != null && best == null)
        while (!domainCandidateQueue.isEmpty()) {
          CandidatePair nextCandidatePair = domainCandidateQueue.element();
          final GlcNode former = getNode(domain_key);
          final GlcNode next = nextCandidatePair.getCandidate();
          if (former != null)
            if (!Scalars.lessThan(next.costFromRoot(), former.costFromRoot()))
              break;
          domainCandidateQueue.remove();
          if (obstacleQuery.isDisjoint(connectors.get(next))) { // no collision
            node.insertEdgeTo(next);
            insert(domain_key, next);
            if (!goalQuery.isDisjoint(connectors.get(next)))
              offerDestination(next); // TODO display computation time until goal was found
            break; // leaves the while loop, but not the for loop
          }
        }
    }
  }

  public void switchRootToState(Tensor state) {
    GlcNode newRoot = this.getNode(convertToKey(state));
    // TODO not nice, as we jump from state to startnode
    if (newRoot != null)
      switchRootToNode(newRoot);
    else
      System.out.println("This domain is not labelled yet");
  }

  private void switchRootToNode(GlcNode newRoot) {
    int oldDomainMapSize = domainMap().size();
    long totalCandidates = candidateMap.values().stream().flatMap(Collection::stream).count();
    int oldQueueSize = queue().size();
    System.out.println("changing to root:" + newRoot.state());
    if (newRoot.isRoot()) {
      System.out.println("node is already root");
      return;
    }
    // removes the new root from the child list of its parent
    final GlcNode parent = newRoot.parent();
    parent.removeEdgeTo(newRoot);
    Collection<GlcNode> oldtree = Nodes.ofSubtree(parent);
    if (queue().removeAll(oldtree))
      System.out.println("Removed " + (oldQueueSize - queue().size()) + " nodes from Queue");
    int removedNodes = 0;
    int removedCandidates = 0;
    int addedNodesToQueue = 0;
    for (GlcNode tempNode : oldtree) { // loop for each domain, where sth was deleted
      Tensor tempDomainKey = convertToKey(tempNode.state());
      if (domainMap().remove(tempDomainKey, tempNode)) // removing from DomainMap
        removedNodes++;
      if (candidateMap.containsKey(tempDomainKey)) {
        Collection<CandidatePair> tempCandidateQueue = candidateMap.get(tempDomainKey);
        if (tempCandidateQueue != null) {
          // System.out.println("tempCandidateQueue is NOT null");
          Iterator<CandidatePair> iterator = tempCandidateQueue.iterator();
          while (iterator.hasNext()) {
            CandidatePair cp = iterator.next();
            if (oldtree.contains(cp.getOrigin())) {
              iterator.remove();
              removedCandidates++;
            }
          }
          // if (tempCandidateQueue.removeIf(candidate -> oldtree.contains(candidate.getOrigin())))
          // removedCandidates++; // TODO Counter wrong, check candidate map vorher/nachher
        }
        // iterate through DomainQueue to find alternative: RELABLING
        PriorityQueue<CandidatePair> queue = new PriorityQueue<>(tempCandidateQueue);
        while (!queue.isEmpty()) {
          final CandidatePair nextBestCandidate = queue.poll();
          final GlcNode next = nextBestCandidate.getCandidate();
          final GlcNode nextParent = nextBestCandidate.getOrigin();
          final List<StateTime> trajectory = //
              stateIntegrator.trajectory(nextParent.stateTime(), next.flow());
          if (obstacleQuery.isDisjoint(trajectory)) { // no collision
            nextParent.insertEdgeTo(next);
            insert(tempDomainKey, next);
            addedNodesToQueue++;
            if (!goalQuery.isDisjoint(trajectory))
              offerDestination(next);
            // TODO already finds goal here and then has runtimeexpectopn
            break; // leaves the while loop, but not the for loop
          }
        }
      }
    }
    System.out.println(removedNodes + " out of " + oldDomainMapSize + " Nodes removed from Tree ");
    System.out.println(removedCandidates + " of " + totalCandidates + " Candidates removed from CandidateList ");
    System.out.println(addedNodesToQueue + " Nodes added to Queue");
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

  /** @param newGoal is the new RegionQuery for the new Goalregion */
  public void setGoalQuery(CostFunction newCostFunction, TrajectoryRegionQuery newGoal) {
    this.goalQuery = newGoal;
    costFunction = newCostFunction;
    List<StateTime> bestList = new ArrayList<>();
    bestList.add(best.stateTime());
    if (newGoal.isDisjoint(bestList)) {
      best = null;
      // TODO Do I need to check entire tree if goal is already in it?
      long tic = System.nanoTime();
      // Changing the Merit in Queue for each Node
      List<GlcNode> list = new LinkedList<>(queue());
      queue().clear();
      list.stream().parallel() //
          .forEach(glcNode -> glcNode.setMinCostToGoal(costFunction.minCostToGoal(glcNode.state())));
      queue().addAll(list);
      long toc = System.nanoTime();
      System.out.println("Updated Merit of Queue with " + list.size() + " nodes: " + ((toc - tic) * 1e-9));
    } else {
      System.out.println("Goal was already found in the existing tree");
      offerDestination(best);
    }
  }
}
