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
    // does buzllshit
    // goes backwards with it (the root)
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
        int Candidatesleft = candidateQueue.size();
        // while (Candidatesleft > 0) {
        while (!candidateQueue.isEmpty()) {
          CandidatePair nextCandidatePair = candidateQueue.element();
          Candidatesleft--;
          final GlcNode formerLabel = getNode(domain_key);
          final GlcNode next = nextCandidatePair.getCandidate();
          if (formerLabel != null) {
            if (Scalars.lessThan(next.merit(), formerLabel.merit())) {
              // collision check only if new node is better
              // remove next from DomainQueue
              if (obstacleQuery.isDisjoint(connectors.get(next))) {// better node not collision
                // current label back in Candidatelist
                // GlcNode formerLabelParent = formerLabel.parent();
                // formerLabel.parent().removeEdgeTo(formerLabel);
                // candidateMap.get(domain_key).add(new CandidatePair(formerLabel.parent(), formerLabel));
                // current label disconnecting
                node.insertEdgeTo(next);
                insert(domain_key, next);
              //removing the newnode from candidate list from this domain
                candidateMap.get(domain_key).remove(next); 
                if (!goalQuery.isDisjoint(connectors.get(next)))
                  offerDestination(next);
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

  public void switchRootToState(Tensor state) {
    GlcNode newRoot = this.getNode(convertToKey(state));
    // TODO not nice, as we jump from state to startnode
    if (newRoot != null)
      switchRootToNode(newRoot);
    else
      System.out.println("This domain is not labelled yet");
  }

  public void switchRootToNode(GlcNode newRoot) {
    if (newRoot.isRoot()) {
      System.out.println("node is already root");
      return;
    }
    int oldDomainMapSize = domainMap().size();
    long oldtotalCandidates = candidateMap.values().stream().flatMap(Collection::stream).count();
    int oldQueueSize = queue().size();
    System.out.println("changing to root:" + newRoot.state());
    // removes the new root from the child list of its parent
    final GlcNode parent = newRoot.parent();
    parent.removeEdgeTo(newRoot);
    Collection<GlcNode> oldtree = Nodes.ofSubtree(parent);
    if (best != null) {
      if (oldtree.contains(best)) // check if goalnode was deleted
        best = null;
    }
    System.out.println("size of oldtree: " + oldtree.size());
    if (queue().removeAll(oldtree))
      System.out.println("Removed " + (oldQueueSize - queue().size()) + " nodes from Queue");
    int removedNodes = 0;
    int addedNodesToQueue = 0;
    for (GlcNode tempLabel : oldtree) { // loop for each domain, where sth was deleted
      Tensor tempDomainKey = convertToKey(tempLabel.state());
      if (domainMap().remove(tempDomainKey, tempLabel)) // removing from DomainMap
        removedNodes++;
      if (candidateMap.containsKey(tempDomainKey)) {
        Set<CandidatePair> tempCandidateSet = candidateMap.get(tempDomainKey);
        if (tempCandidateSet != null) {
          tempCandidateSet.removeIf(candidate -> oldtree.contains(candidate.getOrigin()));
          // --
          // Iterate through DomainQueue to find alternative: RELABELING
          // --
          // CandidatePairQueue queue = new CandidatePairQueue(tempCandidateQueue);
          // TODO: Why does not work with CandidatePairQueue
          PriorityQueue<CandidatePair> candidateQueue = new PriorityQueue<>(tempCandidateSet);
          while (!candidateQueue.isEmpty()) {
            final CandidatePair nextCandidate = candidateQueue.element();
            final GlcNode next = nextCandidate.getCandidate();
            final GlcNode nextParent = nextCandidate.getOrigin();
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
            candidateQueue.remove();
          }
        }
      }
    }
    long newtotalCandidates = candidateMap.values().stream().flatMap(Collection::stream).count();
    System.out.println(removedNodes + " out of " + oldDomainMapSize + " Nodes removed from Tree ");
    System.out.println(oldtotalCandidates - newtotalCandidates + " of " + oldtotalCandidates + " Candidates removed from CandidateList ");
    System.out.println(addedNodesToQueue + " Nodes added to Queue");
    System.out.println("Rootswitch finished");
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
    // TODO refactoring as some code is double
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
    {
      long tic = System.nanoTime();
      Collection<GlcNode> TreeCollection = Nodes.ofSubtree(getNodesfromRootToGoal().get(1));
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
    list.stream().parallel().// Changing the depth of queue
        forEach(glcNode -> glcNode.calculateDepth());
    queue().addAll(list);
    long toc = System.nanoTime();
    System.out.println("Updated Merit & Depth of Queue with " + list.size() + " nodes in: " //
        + ((toc - tic) * 1e-9) + "s");
    System.out.println("Goal switch finished");
  }
}
