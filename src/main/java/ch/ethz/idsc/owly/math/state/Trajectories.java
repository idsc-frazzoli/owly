// code by bapaden and jph
package ch.ethz.idsc.owly.math.state;

import java.util.ArrayList;
import java.util.List;

import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;

/** utility functions that operate on List<StateTime> */
public enum Trajectories {
  ;
  // ---
  /** @param from
   * @param trajectory
   * @return time increment between given from State and end of trajectory */
  public static Scalar timeIncrement(StateTime from, List<StateTime> trajectory) {
    Scalar dt = getLast(trajectory).time().subtract(from.time());
    if (Scalars.lessEquals(dt, RealScalar.ZERO))
      throw new RuntimeException();
    return dt;
  }

  public static StateTime getLast(List<StateTime> list) {
    return list.get(list.size() - 1);
  }

  /** @param stateIntegrator
   * @param list
   * @return */
  public static List<StateTime> connect(StateIntegrator stateIntegrator, List<GlcNode> list) {
    List<StateTime> trajectory = new ArrayList<>();
    if (!list.isEmpty()) {
      trajectory.add(list.get(0).stateTime()); // add first node
      for (int index = 1; index < list.size(); ++index) {
        GlcNode prevNode = list.get(index - 1);
        GlcNode nextNode = list.get(index);
        if (prevNode != nextNode.parent())
          throw new RuntimeException();
        List<StateTime> part = stateIntegrator.trajectory(prevNode.stateTime(), nextNode.flow());
        trajectory.addAll(part);
      }
    }
    return trajectory;
  }

  public static void print(List<StateTime> list) {
    System.out.println("Trajectory (" + list.size() + ")");
    for (StateTime stateTime : list)
      System.out.println(stateTime.info());
  }
}
