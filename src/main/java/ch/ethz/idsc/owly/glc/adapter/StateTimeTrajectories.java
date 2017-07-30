// code by jph
package ch.ethz.idsc.owly.glc.adapter;

import java.util.List;

import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;

/** utility functions that operate on List<StateTime> */
public enum StateTimeTrajectories {
  ;
  // ---
  /** @param from
   * @param trajectory
   * @return time increment between given from State and end of trajectory */
  public static Scalar timeIncrement(StateTime from, List<StateTime> trajectory) {
    Scalar dt = StateTimeTrajectories.getLast(trajectory).time().subtract(from.time());
    if (Scalars.lessEquals(dt, RealScalar.ZERO))
      throw new RuntimeException();
    return dt;
  }

  public static StateTime getLast(List<StateTime> list) {
    return list.get(list.size() - 1);
  }

  public static void print(List<StateTime> list) {
    System.out.println("Trajectory (" + list.size() + ")");
    for (StateTime stateTime : list)
      System.out.println(stateTime.toInfoString());
  }
}
