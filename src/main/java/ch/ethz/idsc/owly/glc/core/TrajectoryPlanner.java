// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.ZeroScalar;
import ch.ethz.idsc.tensor.red.Mean;
import ch.ethz.idsc.tensor.sca.Floor;

public class TrajectoryPlanner {
  final DynamicalSystem dynamicalSystem;
  final Tensor controls;
  final CostFunction costFunction;
  final Heuristic heuristic;
  final TrajectoryRegionQuery goalQuery;
  final TrajectoryRegionQuery obstacleQuery;
  // ---
  Tensor partitionScale;
  int depth_limit = 20; // TODO
  public final PriorityQueue<Node> queue = new PriorityQueue<>(NodeMeritComparator.instance);
  double time_scale = 10; // TODO
  double expand_time;
  private Map<Tensor, Domain> domain_labels = new HashMap<>();
  private int res = 10; // TODO
  double eps;
  private boolean foundGoal = false; // TODO
  private Node best = null;

  public TrajectoryPlanner( //
      DynamicalSystem dynamicalSystem, //
      Tensor controls, //
      CostFunction costFunction, //
      Heuristic heuristic, //
      TrajectoryRegionQuery goalQuery, //
      TrajectoryRegionQuery obstacleQuery //
  ) {
    this.dynamicalSystem = dynamicalSystem;
    this.controls = controls;
    this.costFunction = costFunction;
    this.heuristic = heuristic;
    this.goalQuery = goalQuery;
    this.obstacleQuery = obstacleQuery;
    // ---
  }

  public void initialize(Tensor partitionScale) {
    int state_dim = partitionScale.length();
    this.partitionScale = partitionScale; // TODO
    if (0 < costFunction.getLipschitz()) {
      eps = (Math.sqrt(state_dim) / Mean.of(partitionScale).Get().number().doubleValue()) * //
          (dynamicalSystem.getLipschitz()) / (costFunction.getLipschitz()) * //
          (res * Math.exp(dynamicalSystem.getLipschitz()) - 1.0);
    } else {
      eps = 0;
    }
    expand_time = time_scale / (double) res;
    System.out.println("eps " + eps);
    System.out.println("expand_time " + expand_time);
  }

  private Tensor convertToKey(Tensor x) {
    return partitionScale.pmul(x).map(Floor.function);
  }

  public void insertRoot(Tensor x) {
    final Node node = new Node(x, ZeroScalar.get(), 0, ZeroScalar.get(), null);
    queue.add(node);
    Tensor current_domain = convertToKey(node.x);
    domain_labels.put(current_domain, new Domain());
  }

  public boolean expand() {
    if (queue.isEmpty()) {
      System.out.println("queue is empty");
      return false;
    }
    // poll() Retrieves and removes the head of this queue
    final Node current_node = queue.poll();
    if (depth_limit < current_node.depth) {
      System.out.println("depth limit reached " + current_node.depth);
      return false;
    }
    // System.out.println("current_node " + current_node);
    boolean live = true;
    Set<Domain> domains_needing_update = new HashSet<>();
    Map<Node, Trajectory> traj_from_parent = new HashMap<>();
    for (Tensor u : controls) {
      final Trajectory trajectory = dynamicalSystem.sim(current_node.time, current_node.time + expand_time, current_node.x, u);
      final StateTime last = trajectory.getBack();
      Node new_arc = new Node( //
          last.tensor, // // new_arc.x
          current_node.cost.add(costFunction.cost(trajectory, u)), // new_arc.cost
          last.time, // new_arc.t
          heuristic.costToGo(last.tensor), // new_arc.merit
          u // new_arc.u_idx
      );
      traj_from_parent.put(new_arc, trajectory);
      // ---
      Tensor current_domain = convertToKey(new_arc.x);
      if (!domain_labels.containsKey(current_domain)) {
        Domain d_new = new Domain();
        domain_labels.put(current_domain, d_new);
      }
      Domain bucket = domain_labels.get(current_domain);
      domains_needing_update.add(bucket);
      if (Scalars.lessThan(new_arc.cost, bucket.getCost().add(RealScalar.of(eps))))
        bucket.candidates.add(new_arc);
    }
    // ---
    // System.out.println("domneedupsize " + domains_needing_update.size());
    for (Domain current_domain : domains_needing_update) {
      // System.out.println(current_domain);
      boolean found_best = false;
      while (!current_domain.candidates.isEmpty()) {
        // peek() Retrieves, but does not remove, the head of this queue
        final Node top = current_domain.candidates.peek();
        if (Scalars.lessThan(top.cost, current_domain.getCost().add(RealScalar.of(eps)))) {
          final Node curr = top;
          int collisionIndex = obstacleQuery.firstMember(traj_from_parent.get(curr));
          if (collisionIndex == TrajectoryRegionQuery.NOMATCH) {
            current_node.addChild(curr, expand_time);
            if (!foundGoal) {
              queue.add(curr);
            }
            if (!found_best) {
              found_best = true;
              current_domain.setLabel(curr);
            }
            int goalIndex = goalQuery.firstMember(traj_from_parent.get(curr));
            if (goalIndex != TrajectoryRegionQuery.NOMATCH && (best == null || Scalars.lessThan(curr.cost , best.cost))) {
              // TODO logic is not so intuitive
              foundGoal = true;
              live = false;
              System.out.println("found goal");
              best = curr;
              // double tail_cost = traj_from_parent.get(curr).getDurationFrom(num-1)*
              // (1.0+(cf->getLipschitzConstant())*sqr(controls[best->u_idx][0]));
            }
          }
        }
        current_domain.candidates.poll();
      }
      if (current_domain.empty()) {
        domain_labels.remove(current_domain);
      }
    }
    return live;
  }

  public void plan() {
    while (expand()) {
      // System.out.println("expand");
      // ---
    }
  }

  public Trajectory getDetailed() {
    return getDetailed(Nodes.getNodesFromRoot(best));
  }

  public Trajectory getDetailed(List<Node> list) {
    Trajectory trajectory = new Trajectory();
    for (int index = 1; index < list.size(); ++index) {
      Node prev = list.get(index - 1);
      Node next = list.get(index);
      Trajectory part = dynamicalSystem.sim(prev.time, prev.time + expand_time, prev.x, next.u);
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
