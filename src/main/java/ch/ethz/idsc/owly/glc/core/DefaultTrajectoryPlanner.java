// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

import ch.ethz.idsc.owly.math.Flow;
import ch.ethz.idsc.owly.math.integrator.Integrator;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;

public class DefaultTrajectoryPlanner extends TrajectoryPlanner {
  protected final Controls controls;
  protected final CostFunction costFunction;
  protected final Heuristic heuristic;
  protected final TrajectoryRegionQuery goalQuery;
  protected final TrajectoryRegionQuery obstacleQuery;

  public DefaultTrajectoryPlanner( //
      Integrator integrator, //
      Scalar timeStep, // 
      Tensor partitionScale, //
      Controls controls, //
      int trajectorySize, //
      CostFunction costFunction, //
      Heuristic heuristic, //
      TrajectoryRegionQuery goalQuery, //
      TrajectoryRegionQuery obstacleQuery //
  ) {
    super(integrator, timeStep, partitionScale);
    this.controls = controls;
    this.costFunction = costFunction;
    this.heuristic = heuristic;
    this.goalQuery = goalQuery;
    this.obstacleQuery = obstacleQuery;
  }

  @Override
  protected void expand(Node current_node) {
    // TODO count updates in cell based on costs for benchmarking
    Map<Tensor, DomainQueue> candidates = new HashMap<>();
    Map<Node, Trajectory> traj_from_parent = new HashMap<>();
    for (Flow flow : controls) {
      final Trajectory trajectory = new Trajectory();
      {
        StateTime prev = new StateTime(current_node.x, current_node.time);
        for (int c0 = 0; c0 < 5; ++c0) { // TODO magic const
          Tensor x1 = integrator.step(flow, prev.x, timeStep);
          StateTime next = new StateTime(x1, prev.time.add(timeStep));
          trajectory.add(next);
          prev = next;
        }
      }
      final StateTime last = trajectory.getBack();
      Node new_arc = new Node(flow, last.x, last.time, //
          current_node.cost.add(costFunction.cost(trajectory, flow)), // new_arc.cost
          heuristic.costToGo(last.x) // new_arc.merit
      );
      traj_from_parent.put(new_arc, trajectory);
      // ---
      final Tensor domain_key = convertToKey(new_arc.x);
      Node prev = getNode(domain_key);
      if (prev != null) {
        if (Scalars.lessThan(new_arc.cost, prev.cost))
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

  void processCandidates(Node current_node, Map<Tensor, DomainQueue> candidates, Map<Node, Trajectory> traj_from_parent) {
    for (Entry<Tensor, DomainQueue> entry : candidates.entrySet()) {
      final Tensor domain_key = entry.getKey();
      final DomainQueue domainQueue = entry.getValue();
      while (!domainQueue.isEmpty()) {
        Node node = domainQueue.poll(); // poll() Retrieves and removes the head of this queue
        if (obstacleQuery.isDisjoint(traj_from_parent.get(node))) {
          current_node.addChild(node, node.time);
          insert(domain_key, node);
          if (!goalQuery.isDisjoint(traj_from_parent.get(node)))
            offerDestination(node);
          domainQueue.clear();
          break;
        }
      }
    }
  }
}
