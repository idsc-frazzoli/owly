// code by jph
package ch.ethz.idsc.owly.glc.core;

import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.CostFunction;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.Trajectories;

/* package */ enum SharedUtils {
  ;
  // integrate flow for each control
  public static Map<GlcNode, List<StateTime>> integrate( //
      GlcNode node, Collection<Flow> controls, StateIntegrator stateIntegrator, CostFunction costFunction) {
    Map<GlcNode, List<StateTime>> map = new ConcurrentHashMap<>(); // <- for use of parallel()
    controls.stream().parallel().forEach(flow -> { // parallel results in speedup of ~25% (rice2demo)
      final List<StateTime> trajectory = stateIntegrator.trajectory(node.stateTime(), flow);
      final StateTime last = Trajectories.getLast(trajectory);
      final GlcNode next = GlcNodes.of(flow, last, //
          node.costFromRoot().add(costFunction.costIncrement(node.stateTime(), trajectory, flow)), //
          costFunction.minCostToGoal(last.x()) //
      );
      map.put(next, trajectory);
    });
    return map;
  }
}
