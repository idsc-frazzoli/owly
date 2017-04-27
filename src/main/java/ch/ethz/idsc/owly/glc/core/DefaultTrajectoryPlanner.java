// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import ch.ethz.idsc.owly.math.Flow;
import ch.ethz.idsc.owly.math.integrator.Integrator;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.ZeroScalar;

public class DefaultTrajectoryPlanner extends TrajectoryPlanner {
  protected final Collection<Flow> controls;
  protected final int trajectorySize;
  protected final CostFunction costFunction;
  protected final Heuristic heuristic;
  protected final TrajectoryRegionQuery goalQuery;
  protected final TrajectoryRegionQuery obstacleQuery;

  public DefaultTrajectoryPlanner( //
      Integrator integrator, //
      Scalar timeStep, //
      Tensor partitionScale, //
      Collection<Flow> controls, //
      int trajectorySize, //
      CostFunction costFunction, //
      Heuristic heuristic, //
      TrajectoryRegionQuery goalQuery, //
      TrajectoryRegionQuery obstacleQuery //
  ) {
    super(integrator, timeStep, partitionScale);
    this.controls = controls;
    this.trajectorySize = trajectorySize;
    this.costFunction = costFunction;
    this.heuristic = heuristic;
    this.goalQuery = goalQuery;
    this.obstacleQuery = obstacleQuery;
  }

  @Override
  protected void expand(final Node current_node) {
    // TODO count updates in cell based on costs for benchmarking
    Map<Tensor, DomainQueue> candidates = new HashMap<>();
    Map<Node, List<StateTime>> traj_from_parent = new HashMap<>();
    for (final Flow flow : controls) {
      final List<StateTime> trajectory = new ArrayList<>();
      {
        StateTime prev = new StateTime(current_node.x, current_node.time);
        for (int c0 = 0; c0 < trajectorySize; ++c0) {
          Tensor x1 = integrator.step(flow, prev.x, timeStep);
          StateTime next = new StateTime(x1, prev.time.add(timeStep));
          trajectory.add(next);
          prev = next;
        }
      }
      final StateTime last = Trajectory.getLast(trajectory);
      final Node new_arc = new Node(flow, last.x, last.time, //
          current_node.cost.add(costFunction.costIncrement(current_node.getStateTime(), trajectory, flow)), // new_arc.cost
          heuristic.costToGo(last.x) // new_arc.merit
      );
      traj_from_parent.put(new_arc, trajectory);
      // ---
      final Tensor domain_key = convertToKey(new_arc.x);
      final Node prev = getNode(domain_key);
      if (prev != null) { // already some node present from previous exploration
        if (Scalars.lessThan(new_arc.cost, prev.cost)) // new node is better than previous one
          if (candidates.containsKey(domain_key))
            candidates.get(domain_key).add(new_arc);
          else
            candidates.put(domain_key, new DomainQueue(new_arc));
      } else
        candidates.put(domain_key, new DomainQueue(new_arc));
    }
    // ---
    processCandidates(current_node, candidates, traj_from_parent);
  }

  private void processCandidates( //
      Node current_node, //
      Map<Tensor, DomainQueue> candidates, //
      Map<Node, List<StateTime>> traj_from_parent) {
    for (Entry<Tensor, DomainQueue> entry : candidates.entrySet()) {
      final Tensor domain_key = entry.getKey();
      final DomainQueue domainQueue = entry.getValue();
      while (!domainQueue.isEmpty()) {
        Node node = domainQueue.poll(); // poll() Retrieves and removes the head of this queue
        if (obstacleQuery.isDisjoint(traj_from_parent.get(node))) { // no collision
          current_node.addChild(node);
          insert(domain_key, node);
          if (!goalQuery.isDisjoint(traj_from_parent.get(node)))
            offerDestination(node);
          break; // leaves the while loop, but not the for loop
        }
      }
    }
  }

  @Override
  protected Node createRootNode(Tensor x) {
    return new Node(null, x, ZeroScalar.get(), ZeroScalar.get(), heuristic.costToGo(x));
  }

  @Override
  public TrajectoryRegionQuery getObstacleQuery() {
    return obstacleQuery;
  }
}
