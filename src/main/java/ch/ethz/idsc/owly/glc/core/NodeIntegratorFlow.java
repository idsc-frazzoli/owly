// code by jph
package ch.ethz.idsc.owly.glc.core;

import java.io.Serializable;
import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import ch.ethz.idsc.owly.data.Lists;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;

/* private */ class FlowTrajectory {
  final Flow flow;
  final List<StateTime> trajectory;
  final StateTime last;

  FlowTrajectory(Flow flow, List<StateTime> trajectory) {
    this.flow = flow;
    this.trajectory = trajectory;
    last = Lists.getLast(trajectory);
  }

  List<StateTime> trajectory() {
    return trajectory;
  }

  GlcNode createGlcNode(GlcNode node, CostFunction costFunction) {
    return GlcNode.of(flow, last, //
        node.costFromRoot().add(costFunction.costIncrement(node, trajectory, flow)), //
        costFunction.minCostToGoal(last.state()));
  }
}

/** utility class used in {@link StandardTrajectoryPlanner} to
 * compute the trajectories from a given node for all controls.
 * 
 * since the integration is independent for all controls,
 * the implementation happens in parallel. */
/* package */ class NodeIntegratorFlow implements Serializable {
  private final StateIntegrator stateIntegrator;
  private final Collection<Flow> controls;
  private final CostFunction costFunction;

  /** @param stateIntegrator
   * @param controls */
  public NodeIntegratorFlow(StateIntegrator stateIntegrator, Collection<Flow> controls, CostFunction costFunction) {
    this.stateIntegrator = stateIntegrator;
    this.controls = controls;
    this.costFunction = costFunction;
  }

  /** parallel trajectory integration is used by {@link StandardTrajectoryPlanner}
   * 
   * @param node from which to expand
   * @param costFunction
   * @return */
  public Map<GlcNode, List<StateTime>> parallel(GlcNode node) {
    // parallel results in speedup of ~25% (rice2demo)
    return controls.stream().parallel() //
        .map(flow -> new FlowTrajectory(flow, stateIntegrator.trajectory(node.stateTime(), flow))) //
        .collect(Collectors.toMap( //
            flowTrajectory -> flowTrajectory.createGlcNode(node, costFunction), //
            FlowTrajectory::trajectory));
  }

  /** @param node from which to expand
   * @param costFunction
   * @return */
  public Map<GlcNode, List<StateTime>> serial(GlcNode node) {
    return controls.stream() //
        .map(flow -> new FlowTrajectory(flow, stateIntegrator.trajectory(node.stateTime(), flow))) //
        .collect(Collectors.toMap( //
            flowTrajectory -> flowTrajectory.createGlcNode(node, costFunction), //
            FlowTrajectory::trajectory));
  }
}
