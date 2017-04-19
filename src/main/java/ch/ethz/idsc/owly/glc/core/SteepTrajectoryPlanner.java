// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

import ch.ethz.idsc.owly.math.Flow;
import ch.ethz.idsc.owly.math.integrator.Integrator;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.sca.Floor;

public class SteepTrajectoryPlanner extends TrajectoryPlanner {
  public SteepTrajectoryPlanner( //
      Integrator integrator, //
      DynamicalSystem dynamicalSystem, //
      Controls controls, //
      CostFunction costFunction, //
      Heuristic heuristic, //
      TrajectoryRegionQuery goalQuery, //
      TrajectoryRegionQuery obstacleQuery //
  ) {
    super(integrator, dynamicalSystem, controls, costFunction, heuristic, goalQuery, obstacleQuery);
  }

  private Tensor convertToKey(Tensor x) {
    return partitionScale.pmul(x).map(Floor.function);
  }

  private void insert(Tensor domain_key, Node node) {
    queue.add(node);
    domain_labels.put(domain_key, node);
  }

  @Override
  boolean expand() {
    if (queue.isEmpty()) {
      System.out.println("queue is empty");
      return false;
    }
    final Node current_node = queue.poll(); // poll() Retrieves and removes the head of this queue
    if (depth_limit < current_node.depth) {
      System.out.println("depth limit reached " + current_node.depth);
      return false;
    }
    Map<Tensor, Node> candidates = new HashMap<>();
    Map<Node, Trajectory> traj_from_parent = new HashMap<>();
    for (Flow flow : controls) {
      final Trajectory trajectory = dynamicalSystem.sim(integrator, flow, current_node.time, current_node.time.add(expand_time), current_node.x);
      final StateTime last = trajectory.getBack();
      Node new_arc = new Node(flow, last.x, last.time, //
          current_node.cost.add(costFunction.cost(trajectory, flow)), // new_arc.cost
          heuristic.costToGo(last.x) // new_arc.merit
      );
      traj_from_parent.put(new_arc, trajectory);
      // ---
      final Tensor domain_key = convertToKey(new_arc.x);
      if (domain_labels.containsKey(domain_key)) {
        Node bucket = domain_labels.get(domain_key);
        if (Scalars.lessThan(new_arc.cost, bucket.cost)) {
          if (candidates.containsKey(domain_key)) {
            Node cmp = candidates.get(domain_key);
            if (Scalars.lessThan(new_arc.cost, cmp.cost))
              candidates.put(domain_key, new_arc);
          } else
            candidates.put(domain_key, new_arc);
        }
      } else
        candidates.put(domain_key, new_arc);
    }
    // ---
    for (Entry<Tensor, Node> entry : candidates.entrySet()) {
      final Tensor domain_key = entry.getKey();
      final Node node = entry.getValue();
      if (obstacleQuery.isDisjoint(traj_from_parent.get(node))) {
        current_node.addChild(node, expand_time);
        insert(domain_key, node);
        if (!goalQuery.isDisjoint(traj_from_parent.get(node)))
          if (best == null || Scalars.lessThan(node.cost, best.cost)) {
            best = node;
            System.out.println("found goal");
            // TODO count updates in cell based on costs
          }
      }
    }
    return best == null;
  }
}
