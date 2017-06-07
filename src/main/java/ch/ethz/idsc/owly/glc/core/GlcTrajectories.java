// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import java.util.ArrayList;
import java.util.List;

import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;

/** utility functions that operate on List<GlcNode> */
enum GlcTrajectories {
  ;
  /** @param stateIntegrator
   * @param list
   * @return */
  static List<TrajectorySample> connect(StateIntegrator stateIntegrator, List<GlcNode> list) {
    List<TrajectorySample> trajectory = new ArrayList<>();
    if (!list.isEmpty()) {
      GlcNode node = list.get(0);
      trajectory.add(new TrajectorySample(node.stateTime(), node.flow())); // add first node
      for (int index = 1; index < list.size(); ++index) {
        GlcNode prevNode = list.get(index - 1);
        GlcNode nextNode = list.get(index);
        if (prevNode != nextNode.parent())
          throw new RuntimeException();
        List<StateTime> part = stateIntegrator.trajectory(prevNode.stateTime(), nextNode.flow());
        part.forEach(stateTime -> trajectory.add(new TrajectorySample(stateTime, nextNode.flow())));
      }
    }
    return trajectory;
  }
}
