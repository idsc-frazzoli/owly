// code by jl
package ch.ethz.idsc.owly.glc.core;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
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
      // System.out.println("Candidatessize is: " + candidates.size());
      // ALL Candidates are saved in CandidateList
    }
    // save candidates in CandidateMap for RootSwitchlater
    for (Entry<Tensor, CandidatePairQueue> entry : candidates.map.entrySet()) {
      if (!candidateMap.containsKey(entry.getKey()))
        candidateMap.put(entry.getKey(), new LinkedList<>());
      candidateMap.get(entry.getKey()).addAll(entry.getValue());
    }
    processCandidates(node, connectors, candidates);
  }

  private void processCandidates( //
      GlcNode node, Map<GlcNode, List<StateTime>> connectors, CandidatePairQueueMap candidates) {
    for (Entry<Tensor, CandidatePairQueue> entry : candidates.map.entrySet()) {
      final Tensor domain_key = entry.getKey();
      final CandidatePairQueue domainCandidateQueue = entry.getValue();
      if (domainCandidateQueue != null && best == null) {
        int Candidatesleft = domainCandidateQueue.size();
        while (Candidatesleft > 0) {
          // while (!domainCandidateQueue.isEmpty()) {
          CandidatePair nextCandidatePair = domainCandidateQueue.element();
          Candidatesleft--;
          final GlcNode former = getNode(domain_key);
          final GlcNode next = nextCandidatePair.getCandidate();
          if (former != null) {
            if (Scalars.lessThan(next.merit(), former.merit())) {
              // collision check only if new node is better
              domainCandidateQueue.remove(); // remove next from DomainQueue
              if (obstacleQuery.isDisjoint(connectors.get(next))) {// better node not collision
                // current label disconnecting,
                candidateMap.get(domain_key).add(new CandidatePair(former.parent(), former));
                former.parent().removeEdgeTo(former);
                // current label back in Candidatelist
                node.insertEdgeTo(next);
                insert(domain_key, next);
                if (!goalQuery.isDisjoint(connectors.get(next)))
                  offerDestination(next);
                break;
              }
            }
          } else {
            domainCandidateQueue.remove();
            if (obstacleQuery.isDisjoint(connectors.get(next))) {
              node.insertEdgeTo(next);
              insert(domain_key, next);
              if (!goalQuery.isDisjoint(connectors.get(next)))
                offerDestination(next);
              break;
            }
          }
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
    long oldtotalCandidates = candidateMap.values().stream().flatMap(Collection::stream).count();
    int oldQueueSize = queue().size();
    System.out.println("changing to root:" + newRoot.state());
    if (newRoot.isRoot()) {
      System.out.println("node is already root");
      return;
    }
    if (best != null) {
      List<StateTime> bestList = new ArrayList<>();
      bestList.add(best.stateTime());
      if (goalQuery.isDisjoint(bestList)) // if best not in goal, delete from best
        best = null;
      else
        System.out.println("**** Goal already found ****");
    }
    // removes the new root from the child list of its parent
    final GlcNode parent = newRoot.parent();
    parent.removeEdgeTo(newRoot);
    Collection<GlcNode> oldtree = Nodes.ofSubtree(parent);
    if (queue().removeAll(oldtree))
      System.out.println("Removed " + (oldQueueSize - queue().size()) + " nodes from Queue");
    int removedNodes = 0;
    int addedNodesToQueue = 0;
    for (GlcNode tempNode : oldtree) { // loop for each domain, where sth was deleted
      Tensor tempDomainKey = convertToKey(tempNode.state());
      if (domainMap().remove(tempDomainKey, tempNode)) // removing from DomainMap
        removedNodes++;
      if (candidateMap.containsKey(tempDomainKey)) {
        Collection<CandidatePair> tempCandidateQueue = candidateMap.get(tempDomainKey);
        if (tempCandidateQueue != null)
          tempCandidateQueue.removeIf(candidate -> oldtree.contains(candidate.getOrigin()));
        // --
        // Iterate through DomainQueue to find alternative: RELABELING
        // --
        // CandidatePairQueue queue = new CandidatePairQueue(tempCandidateQueue);
        // TODO: Why does not work with CandidatePairQueue
        PriorityQueue<CandidatePair> queue = new PriorityQueue<>(tempCandidateQueue);
        while (!queue.isEmpty()) {
          final CandidatePair nextBestCandidate = queue.poll();
          final GlcNode next = nextBestCandidate.getCandidate();
          final GlcNode nextParent = nextBestCandidate.getOrigin();
          final List<StateTime> trajectory = //
              stateIntegrator.trajectory(nextParent.stateTime(), next.flow());
          if (obstacleQuery.isDisjoint(trajectory)) { // no collision
            nextParent.insertEdgeTo(next); // always replace as former was deleted
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
    long newtotalCandidates = candidateMap.values().stream().flatMap(Collection::stream).count();
    System.out.println(removedNodes + " out of " + oldDomainMapSize + " Nodes removed from Tree ");
    System.out.println(oldtotalCandidates - newtotalCandidates + " of " + oldtotalCandidates + " Candidates removed from CandidateList ");
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
    // TODO refactoring as some code is double
    if (best != null) {
      List<StateTime> bestList = new ArrayList<>();
      bestList.add(best.stateTime());
      if (newGoal.isDisjoint(bestList)) {
        best = null;
        // TODO Do I need to check entire tree if goal is already in it? Yes (JL)
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
    } else {
      long tic = System.nanoTime();
      // Changing the Merit in Queue for each Node
      List<GlcNode> list = new LinkedList<>(queue());
      queue().clear();
      list.stream().parallel() //
          .forEach(glcNode -> glcNode.setMinCostToGoal(costFunction.minCostToGoal(glcNode.state())));
      queue().addAll(list);
      long toc = System.nanoTime();
      System.out.println("Updated Merit of Queue with " + list.size() + " nodes: " + ((toc - tic) * 1e-9));
    }
  }
}
