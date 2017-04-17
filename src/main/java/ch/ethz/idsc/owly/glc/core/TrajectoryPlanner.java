// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.PriorityQueue;
import java.util.Queue;

import ch.ethz.idsc.owly.math.Flow;
import ch.ethz.idsc.owly.math.integrator.Integrator;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.ZeroScalar;
import ch.ethz.idsc.tensor.sca.Floor;

public class TrajectoryPlanner {
  private final Integrator integrator;
  private final DynamicalSystem dynamicalSystem;
  private final Controls controls;
  private final CostFunction costFunction;
  private final Heuristic heuristic;
  private final TrajectoryRegionQuery goalQuery;
  private final TrajectoryRegionQuery obstacleQuery;
  // ---
  private Tensor partitionScale;
  private int depth_limit = 20; // TODO
  private Scalar expand_time = RealScalar.ONE; // TODO
  private Node best;
  private final Queue<Node> queue = new PriorityQueue<>(NodeMeritComparator.instance);
  private Map<Tensor, Domain> domain_labels = new HashMap<>();

  public TrajectoryPlanner( //
      Integrator integrator, //
      DynamicalSystem dynamicalSystem, //
      Controls controls, //
      CostFunction costFunction, //
      Heuristic heuristic, //
      TrajectoryRegionQuery goalQuery, //
      TrajectoryRegionQuery obstacleQuery //
  ) {
    this.integrator = integrator;
    this.dynamicalSystem = dynamicalSystem;
    this.controls = controls;
    this.costFunction = costFunction;
    this.heuristic = heuristic;
    this.goalQuery = goalQuery;
    this.obstacleQuery = obstacleQuery;
    // ---
  }

  public void setResolution(Tensor partitionScale) {
    this.partitionScale = partitionScale;
  }

  private Tensor convertToKey(Tensor x) {
    return partitionScale.pmul(x).map(Floor.function);
  }

  public void insertRoot(Tensor x) {
    final Node node = new Node(null, x, ZeroScalar.get(), ZeroScalar.get(), ZeroScalar.get());
    queue.add(node);
    Domain domain = new Domain();
    domain.setLabel(node);
    domain_labels.put(convertToKey(node.x), domain);
  }

  public Collection<Node> getQueue() {
    return Collections.unmodifiableCollection(queue);
  }

  public Map<Tensor, Domain> getDomains() {
    return Collections.unmodifiableMap(domain_labels);
  }

  public void setDepthLimit(int depth_limit) {
    this.depth_limit = depth_limit;
  }

  private boolean expand() {
    if (queue.isEmpty()) {
      System.out.println("queue is empty");
      return false;
    }
    final Node current_node = queue.poll(); // poll() Retrieves and removes the head of this queue
    if (depth_limit < current_node.depth) {
      System.out.println("depth limit reached " + current_node.depth);
      return false;
    }
    Map<Domain, Node> domains_needing_update = new LinkedHashMap<>();
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
      final Tensor current_domain_key = convertToKey(new_arc.x);
      // TODO can make more efficient by treating contain cases in detail
      // TODO don't add to domain if future fail!
      if (!domain_labels.containsKey(current_domain_key))
        domain_labels.put(current_domain_key, new Domain());
      Domain bucket = domain_labels.get(current_domain_key);
      if (bucket.takesOffer(new_arc)) {
        if (domains_needing_update.containsKey(bucket)) {
          Node cmp = domains_needing_update.get(bucket);
          if (Scalars.lessThan(new_arc.cost, cmp.cost))
            domains_needing_update.put(bucket, new_arc);
        } else
          domains_needing_update.put(bucket, new_arc);
      }
    }
    // ---
    for (Entry<Domain, Node> entry : domains_needing_update.entrySet()) {
      final Domain current_domain = entry.getKey();
      final Node node = entry.getValue();
      if (obstacleQuery.isDisjoint(traj_from_parent.get(node))) {
        current_node.addChild(node, expand_time);
        queue.add(node);
        current_domain.setLabel(node);
        if (!goalQuery.isDisjoint(traj_from_parent.get(node)))
          if (best == null || Scalars.lessThan(node.cost, best.cost)) {
            best = node;
            System.out.println("found goal");
          }
      }
    }
    return best == null;
  }

  public void plan() {
    best = null;
    while (expand()) {
      // ---
    }
  }

  public Trajectory getDetailedTrajectory() {
    return getDetailedTrajectory(Nodes.getNodesFromRoot(best));
  }

  public Trajectory getDetailedTrajectory(List<Node> list) {
    Trajectory trajectory = new Trajectory();
    for (int index = 1; index < list.size(); ++index) {
      Node prev = list.get(index - 1);
      Node next = list.get(index);
      Trajectory part = dynamicalSystem.sim(integrator, next.u, prev.time, prev.time.add(expand_time), prev.x);
      trajectory.addAll(part);
    }
    return trajectory;
  }

  public Trajectory getPathFromRootToGoal() {
    return getPathFromRootToGoal(Nodes.getNodesFromRoot(best));
  }

  public Trajectory getPathFromRootToGoal(List<Node> list) {
    Trajectory trajectory = new Trajectory();
    for (Node node : list)
      trajectory.add(node.getStateTime());
    return trajectory;
  }
}
