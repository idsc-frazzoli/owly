// code by jph
package ch.ethz.idsc.owly.glc.core;

import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.stream.Stream;

import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.CostFunction;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.Trajectories;

/* package */ enum SharedUtils {
  ;
  // integrate flow for each control
  public static Map<GlcNode, List<StateTime>> integrate( //
      GlcNode node, Collection<Flow> controls, StateIntegrator stateIntegrator, CostFunction costFunction, boolean parallel) {
    Map<GlcNode, List<StateTime>> map = new ConcurrentHashMap<>(); // <- for use of parallel()
    Stream<Flow> stream = controls.stream();
    if (parallel) // parallel results in speedup of ~25% (rice2demo)
      stream = stream.parallel();
    // TODO integrate parallel OR deterministic in linkedHashMap
    stream.forEach(flow -> {
      final List<StateTime> trajectory = stateIntegrator.trajectory(node.stateTime(), flow);
      final StateTime last = Trajectories.getLast(trajectory);
      final GlcNode next = GlcNode.of(flow, last, //
          node.costFromRoot().add(costFunction.costIncrement(node.stateTime(), trajectory, flow)), //
          costFunction.minCostToGoal(last.state()) //
      );
      map.put(next, trajectory);
    });
    return map;
  }
}
