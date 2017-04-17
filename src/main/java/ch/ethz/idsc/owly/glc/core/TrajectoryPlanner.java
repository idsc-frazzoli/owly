// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
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
  private Scalar expand_time = RealScalar.ONE; // TODO
  private final Queue<Node> queue = new PriorityQueue<>(NodeMeritComparator.instance);
  private Map<Tensor, Node> domain_labels = new HashMap<>();

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
  }

  public void setResolution(Tensor partitionScale) {
    this.partitionScale = partitionScale;
  }

  private Tensor convertToKey(Tensor x) {
    return partitionScale.pmul(x).map(Floor.function);
  }

  public void insertRoot(Tensor x) {
    insert(convertToKey(x), //
        new Node(null, x, ZeroScalar.get(), ZeroScalar.get(), ZeroScalar.get()));
  }

  public Collection<Node> getQueue() {
    return Collections.unmodifiableCollection(queue);
  }

  public Collection<Node> getNodes() {
    return Collections.unmodifiableCollection(domain_labels.values());
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
          }
      }
    }
    return best == null;
  }

  private void insert(Tensor domain_key, Node node) {
    queue.add(node);
    domain_labels.put(domain_key, node);
  }

  private int depth_limit;
  private Node best;

  public void plan(int depth_limit) {
    this.depth_limit = depth_limit;
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
