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

/** private utility class to facilitate the construction of the map */
/* private */ class FlowTrajectory {
  private final Flow flow;
  private final List<StateTime> trajectory;

  FlowTrajectory(Flow flow, List<StateTime> trajectory) {
    this.flow = flow;
    this.trajectory = trajectory;
  }

  List<StateTime> trajectory() {
    return trajectory;
  }

  GlcNode createGlcNode(GlcNode node, CostFunction costFunction) {
    StateTime last = Lists.getLast(trajectory);
    return GlcNode.of(flow, last, //
        node.costFromRoot().add(costFunction.costIncrement(node, trajectory, flow)), //
        costFunction.minCostToGoal(last.state()));
  }
}

/** utility class used in {@link TrajectoryPlanner}s to
 * compute the trajectories from a given node for all controls.
 * 
 * since the integration is independent for all controls,
 * the implementation can be carried out in parallel. */
/* package */ class ControlsIntegrator implements Serializable {
  private final StateIntegrator stateIntegrator;
  private final Collection<Flow> controls;
  private final CostFunction costFunction;

  /** @param stateIntegrator
   * @param controls
   * @param costFunction */
  public ControlsIntegrator(StateIntegrator stateIntegrator, Collection<Flow> controls, CostFunction costFunction) {
    this.stateIntegrator = stateIntegrator;
    this.controls = controls;
    this.costFunction = costFunction;
  }

  /** parallel trajectory integration is used by {@link StandardTrajectoryPlanner}
   * 
   * @param node from which to expand
   * @param costFunction
   * @return */
  public Map<GlcNode, List<StateTime>> inParallel(GlcNode node) {
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
  public Map<GlcNode, List<StateTime>> inSequence(GlcNode node) {
    return controls.stream() //
        .map(flow -> new FlowTrajectory(flow, stateIntegrator.trajectory(node.stateTime(), flow))) //
        .collect(Collectors.toMap( //
            flowTrajectory -> flowTrajectory.createGlcNode(node, costFunction), //
            FlowTrajectory::trajectory));
  }
}
