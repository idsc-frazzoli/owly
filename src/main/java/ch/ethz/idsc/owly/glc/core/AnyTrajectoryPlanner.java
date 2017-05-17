// code by bapaden and jph and jl
package ch.ethz.idsc.owly.glc.core;

import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
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

public class AnyTrajectoryPlanner extends TrajectoryPlanner {
  private final StateIntegrator stateIntegrator;
  private final Collection<Flow> controls;
  private CostFunction costFunction;
  private TrajectoryRegionQuery goalQuery;
  private final TrajectoryRegionQuery obstacleQuery;
  private final Map<Tensor, DomainQueue> domainCandidateMap = //
      new HashMap<>();
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
    // Map<Tensor, DomainQueue> domainCandidateMap = new HashMap<>();
    Set<Tensor> domainsNeedingUpdate = new HashSet<>();
    Map<GlcNode, List<StateTime>> connectors = new HashMap<>();
    for (final Flow flow : controls) {
      final List<StateTime> trajectory = stateIntegrator.trajectory(node.stateTime(), flow);
      final StateTime last = Trajectories.getLast(trajectory);
      final GlcNode next = new GlcNode(flow, last, //
          node.costFromRoot().add(costFunction.costIncrement(node.stateTime(), trajectory, flow)), //
          costFunction.minCostToGoal(last.x()) //
      );
      // TODO all Candidates need parent
      next.setParent(node);
      connectors.put(next, trajectory);
      // ---
      final Tensor domain_key = convertToKey(next.stateTime().x());
      domainsNeedingUpdate.add(domain_key);
      final GlcNode former = getNode(domain_key);
      if (former != null) { // Node present in Domain
        // TODO save all nodes in domainCandidateMap-> following if to true
        // if (Scalars.lessThan(next.costFromRoot(), former.costFromRoot()))
        // new node is better than previous one
        if (domainCandidateMap.containsKey(domain_key))
          domainCandidateMap.get(domain_key).add(next);
        else
          domainCandidateMap.put(domain_key, new DomainQueue(next));
      } else // No Node present in Domain
        domainCandidateMap.put(domain_key, new DomainQueue(next));
    }
    // ---
    // make a keylist, through which I iterate and pass to processCandidates
    processCandidates(node, domainsNeedingUpdate, connectors);
  }

  private void processCandidates(GlcNode node, //
      Set<Tensor> domainsNeedingUpdate, Map<GlcNode, List<StateTime>> connectors) {
    for (Tensor domain_key : domainsNeedingUpdate) {
      final DomainQueue domainQueue = domainCandidateMap.get(domain_key);
      if (domainQueue != null)
        while (!domainQueue.isEmpty()) {
          final GlcNode next = domainQueue.element();
          final GlcNode former = getNode(domain_key);
          // If next node in domainQueue NOT better then former leave while loop
          if (former != null)
            if (!Scalars.lessThan(next.costFromRoot(), former.costFromRoot()))
              break;
          domainQueue.remove(); // removes the better head of this queue
          if (obstacleQuery.isDisjoint(connectors.get(next))) { // no collision
            node.insertEdgeTo(next);
            insert(domain_key, next);
            domainCandidateMap.get(domain_key).remove(next);
            if (!goalQuery.isDisjoint(connectors.get(next)))
              offerDestination(next);
            break; // leaves the while loop, but not the for loop
          }
        }
    }
  }

  public void switchRootToState(Tensor state) {
    GlcNode newRoot = this.getNode(convertToKey(state));
    if (newRoot != null)
      switchRootToNode(newRoot);
    else
      System.out.println("This domain is not labelled yet");
  }

  private void switchRootToNode(GlcNode newRoot) {
    int oldDomainMapSize = domainMap().size();
    int oldCandidateMapSize = domainCandidateMap.size();
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
      Tensor tempDomain_key = convertToKey(tempNode.state());
      if (domainMap().remove(tempDomain_key, tempNode))
        removedNodes++;
      DomainQueue tempDomainQueue = domainCandidateMap.get(tempDomain_key);
      if (tempDomainQueue != null) {
        tempDomainQueue.removeIf(GlcNode::isRoot);
        removedCandidates++;
      }
      // iterate through DomainQueue to find alternative
      if (tempDomainQueue != null)
        while (!tempDomainQueue.isEmpty()) {
          final GlcNode next = tempDomainQueue.poll();
          final GlcNode nextParent = next.parent();
          final List<StateTime> trajectory = //
              stateIntegrator.trajectory(nextParent.stateTime(), next.flow());
          if (obstacleQuery.isDisjoint(trajectory)) { // no collision
            nextParent.insertEdgeTo(next);
            insert(tempDomain_key, next);
            addedNodesToQueue++;
            if (!goalQuery.isDisjoint(trajectory))
              offerDestination(next);
            break; // leaves the while loop, but not the for loop
          }
        }
    }
    System.out.println(removedNodes + " of " + oldDomainMapSize + " Nodes removed from Tree ");
    System.out.println(removedCandidates + " of " + oldCandidateMapSize + " Candidates removed from CandidateList ");
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
    best = null;
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
