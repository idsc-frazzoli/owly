// code by bapaden and jph
package ch.ethz.idsc.owl.glc.adapter;

import java.util.ArrayList;
import java.util.List;

import ch.ethz.idsc.owl.data.tree.Nodes;
import ch.ethz.idsc.owl.glc.core.GlcNode;
import ch.ethz.idsc.owl.math.state.StateIntegrator;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.TrajectorySample;

/** utility functions that operate on List<GlcNode> */
public enum GlcTrajectories {
  ;
  /** @param stateIntegrator
   * @param node
   * @return densely sampled trajectory from root to given node
   * that is the result of integrating the flows between the nodes */
  public static List<TrajectorySample> detailedTrajectoryTo(StateIntegrator stateIntegrator, GlcNode node) {
    return connect(stateIntegrator, Nodes.listFromRoot(node));
  }

  /** @param stateIntegrator
   * @param list
   * @return */
  public static List<TrajectorySample> connect(StateIntegrator stateIntegrator, List<GlcNode> list) {
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
