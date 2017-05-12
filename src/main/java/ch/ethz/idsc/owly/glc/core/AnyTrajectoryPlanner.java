// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

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
  private final CostFunction costFunction;
  private final TrajectoryRegionQuery goalQuery;
  private final TrajectoryRegionQuery obstacleQuery;

  public AnyTrajectoryPlanner( //
      Tensor partitionScale, //
      StateIntegrator stateIntegrator, //
      Collection<Flow> controls, //
      CostFunction costFunction, //
      TrajectoryRegionQuery goalQuery, //
      TrajectoryRegionQuery obstacleQuery //
  ) {
    super(partitionScale);
    this.stateIntegrator = stateIntegrator;
    this.controls = controls;
    this.costFunction = costFunction;
    this.goalQuery = goalQuery;
    this.obstacleQuery = obstacleQuery;
  }

  @Override
  public void expand(final Node node) {
    // TODO count updates in cell based on costs for benchmarking
    Map<Tensor, DomainQueue> candidates = new HashMap<>();
    Map<Node, List<StateTime>> connectors = new HashMap<>();
    for (final Flow flow : controls) {
      final List<StateTime> trajectory = stateIntegrator.trajectory(node.stateTime(), flow);
      final StateTime last = Trajectories.getLast(trajectory);
      final Node next = new Node(flow, last, //
          node.cost().add(costFunction.costIncrement(node.stateTime(), trajectory, flow)), //
          costFunction.minCostToGoal(last.x()) //
      );
      connectors.put(next, trajectory);
      // ---
      final Tensor domain_key = convertToKey(next.stateTime().x());
      final Node former = getNode(domain_key);
      if (former != null) { // already some node present from previous exploration
        if (Scalars.lessThan(next.cost(), former.cost())) // new node is better than previous one
          if (candidates.containsKey(domain_key))
            candidates.get(domain_key).add(next);
          else
            candidates.put(domain_key, new DomainQueue(next));
      } else
        candidates.put(domain_key, new DomainQueue(next));
    }
    // ---
    processCandidates(node, candidates, connectors);
  }

  private void processCandidates( //
      Node node, Map<Tensor, DomainQueue> candidates, Map<Node, List<StateTime>> connectors) {
    for (Entry<Tensor, DomainQueue> entry : candidates.entrySet()) {
      final Tensor domain_key = entry.getKey();
      final DomainQueue domainQueue = entry.getValue();
      while (!domainQueue.isEmpty()) {
        Node next = domainQueue.poll(); // poll() Retrieves and removes the head of this queue
        if (obstacleQuery.isDisjoint(connectors.get(next))) { // no collision
          node.addChild(next);
          insert(domain_key, next);
          if (!goalQuery.isDisjoint(connectors.get(next)))
            offerDestination(next);
          break; // leaves the while loop, but not the for loop
        }
      }
    }
  }

  public void switchRootToState(Tensor state) {
    Node newRoot = domainMap().get(convertToKey(state));
    if (newRoot != null) switchRootToNode(newRoot);
    else System.out.println("This domain is not labelled yet");
      return;
  }

  private void switchRootToNode(Node newRoot) {
    // removes the new root from the child list of its parent
    newRoot.parent().children().remove(newRoot.flow(), newRoot);
    HashSet<Node> oldtree = new HashSet<>();
    addNodeToSet(newRoot, oldtree);
    if (queue().removeAll(oldtree))
      System.out.println("Removed oldtree from queue");
    for (Node tempNode : oldtree) {
      if (domainMap().remove(convertToKey(tempNode.stateTime().x()), tempNode)) {
        System.out.println("Removed nodes:");
        tempNode.printNodeState();
      }
    }
    return;
  }

  protected void addNodeToSet(Node node, HashSet<Node> subtree) {
    subtree.add(node);
    if (node.parent() != null)
      addNodeToSet(node.parent(), subtree);
    for (Entry<Flow, Node> tempChild : node.children().entrySet()) {
      if (tempChild != null)
        addNodeToSet(tempChild.getValue(), subtree);
    }
    return;
  }

  @Override
  protected Node createRootNode(Tensor x) { // TODO check if time of root node should always be set to 0
    return new Node(null, new StateTime(x, ZeroScalar.get()), ZeroScalar.get(), costFunction.minCostToGoal(x));
  }

  @Override
  public List<StateTime> detailedTrajectoryTo(Node node) {
    return Trajectories.connect(stateIntegrator, Nodes.nodesFromRoot(node));
  }

  @Override
  public TrajectoryRegionQuery getObstacleQuery() {
    return obstacleQuery;
  }

  @Override
  public TrajectoryRegionQuery getGoalQuery() {
    return goalQuery;
  }
}
