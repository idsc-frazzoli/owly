// code by jph
package ch.ethz.idsc.owly.glc.core;

import java.io.Serializable;
import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.CostFunction;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.Trajectories;

/* package */ class NodeIntegratorFlow implements Serializable {
  private final StateIntegrator stateIntegrator;
  private final Collection<Flow> controls;

  public NodeIntegratorFlow(StateIntegrator stateIntegrator, Collection<Flow> controls) {
    this.stateIntegrator = stateIntegrator;
    this.controls = controls;
  }

  /** @param node
   * @param costFunction
   * @return */
  public Map<GlcNode, List<StateTime>> parallel(GlcNode node, CostFunction costFunction) {
    Map<GlcNode, List<StateTime>> map = new ConcurrentHashMap<>(); // <- for use of parallel()
    // parallel results in speedup of ~25% (rice2demo)
    controls.stream().parallel().forEach(flow -> {
      final List<StateTime> trajectory = stateIntegrator.trajectory(node.stateTime(), flow);
      final StateTime last = Trajectories.getLast(trajectory);
      final GlcNode next = GlcNode.of(flow, last, //
          node.costFromRoot().add(costFunction.costIncrement(node.stateTime(), trajectory, flow)), //
          costFunction.minCostToGoal(last.x()) //
      );
      map.put(next, trajectory);
    });
    return map;
  }
}
