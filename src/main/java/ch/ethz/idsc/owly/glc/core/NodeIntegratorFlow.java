// code by jph
package ch.ethz.idsc.owly.glc.core;

import java.io.Serializable;
import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import ch.ethz.idsc.owly.data.Lists;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;

/** utility class used in {@link StandardTrajectoryPlanner} to
 * compute the trajectories from a given node for all controls.
 * 
 * since the integration is independent for all controls,
 * the implementation happens in parallel. */
/* package */ class NodeIntegratorFlow implements Serializable {
  private final StateIntegrator stateIntegrator;
  private final Collection<Flow> controls;

  /** @param stateIntegrator
   * @param controls */
  public NodeIntegratorFlow(StateIntegrator stateIntegrator, Collection<Flow> controls) {
    this.stateIntegrator = stateIntegrator;
    this.controls = controls;
  }

  /** @param node from which to expand
   * @param costFunction
   * @return */
  public Map<GlcNode, List<StateTime>> parallel(GlcNode node, CostFunction costFunction) {
    Map<GlcNode, List<StateTime>> map = new ConcurrentHashMap<>(); // <- for use of parallel()
    // parallel results in speedup of ~25% (rice2demo)
    // TODO howto stream.collect to map
    controls.stream().parallel().forEach(flow -> {
      final List<StateTime> trajectory = stateIntegrator.trajectory(node.stateTime(), flow);
      final StateTime last = Lists.getLast(trajectory);
      final GlcNode next = GlcNode.of(flow, last, //
          node.costFromRoot().add(costFunction.costIncrement(node, trajectory, flow)), //
          costFunction.minCostToGoal(last.state()) //
      );
      map.put(next, trajectory);
    });
    return map;
  }
}
