// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

import ch.ethz.idsc.owly.math.Flow;
import ch.ethz.idsc.owly.math.integrator.Integrator;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;

public class SteepTrajectoryPlanner extends TrajectoryPlanner {
  protected final Controls controls;
  protected final CostFunction costFunction;
  protected final Heuristic heuristic;
  protected final TrajectoryRegionQuery goalQuery;
  protected final TrajectoryRegionQuery obstacleQuery;

  public SteepTrajectoryPlanner( //
      Integrator integrator, //
      DynamicalSystem dynamicalSystem, //
      Controls controls, //
      CostFunction costFunction, //
      Heuristic heuristic, //
      TrajectoryRegionQuery goalQuery, //
      TrajectoryRegionQuery obstacleQuery //
  ) {
    super(integrator, dynamicalSystem);
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
      final Trajectory trajectory = evolve(flow, current_node);
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
          current_node.addChild(node, expand_time);
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
