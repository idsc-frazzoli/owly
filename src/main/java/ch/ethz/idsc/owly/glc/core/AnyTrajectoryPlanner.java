// code by bapaden and jph and jl
package ch.ethz.idsc.owly.glc.core;

import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

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
    Map<GlcNode, List<StateTime>> connectors = new HashMap<>();
    for (final Flow flow : controls) {
      final List<StateTime> trajectory = stateIntegrator.trajectory(node.stateTime(), flow);
      final StateTime last = Trajectories.getLast(trajectory);
      final GlcNode next = new GlcNode(flow, last, //
          node.costFromRoot().add(costFunction.costIncrement(node.stateTime(), trajectory, flow)), //
          costFunction.minCostToGoal(last.x()) //
      );
      connectors.put(next, trajectory);
      // ---
      final Tensor domain_key = convertToKey(next.stateTime().x());
      final GlcNode former = getNode(domain_key);
      if (former != null) { // already some node present from previous exploration
        if (Scalars.lessThan(next.costFromRoot(), former.costFromRoot())) // new node is better than previous one
          if (domainCandidateMap.containsKey(domain_key))
            domainCandidateMap.get(domain_key).add(next);
          else
            domainCandidateMap.put(domain_key, new DomainQueue(next));
      } else
        domainCandidateMap.put(domain_key, new DomainQueue(next));
    }
    // ---
    processCandidates(node, domainCandidateMap, connectors);
  }

  private void processCandidates( //
      GlcNode node, Map<Tensor, DomainQueue> candidates, Map<GlcNode, List<StateTime>> connectors) {
    for (Entry<Tensor, DomainQueue> entry : candidates.entrySet()) {
      final Tensor domain_key = entry.getKey();
      final DomainQueue domainQueue = entry.getValue();
      while (!domainQueue.isEmpty()) {
        GlcNode next = domainQueue.poll(); // poll() Retrieves and removes the head of this queue
        if (obstacleQuery.isDisjoint(connectors.get(next))) { // no collision
          node.insertEdgeTo(next);
          insert(domain_key, next);
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
      System.out.println("Removed oldtree from queue");
    int removedNodes = 0;
    for (GlcNode tempNode : oldtree) {
      if (domainMap().remove(convertToKey(tempNode.state()), tempNode))
        removedNodes++;
    }
    System.out.println(removedNodes + " of " + oldDomainMapSize +" Nodes removed from Tree ");
    // TODO JONAS relabel domains and add to queue
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
    List<GlcNode> list = new LinkedList<>(queue());
    queue().clear();
    list.stream().parallel() //
        .forEach(glcNode -> glcNode.setMinCostToGoal(costFunction.minCostToGoal(glcNode.state())));
    queue().addAll(list);
    long toc = System.nanoTime();
    System.out.println("rebuild queue with " + list.size() + " nodes: " + ((toc - tic) * 1e-9));
  }
}
