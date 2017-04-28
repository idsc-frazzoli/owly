// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import java.util.List;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.ZeroScalar;

/** utility functions that operate on List<StateTime> */
public enum Trajectories {
  ;
  // ---
  /** @param from
   * @param trajectory
   * @return time increment between given from State and end of trajectory */
  public static Scalar timeIncrement(StateTime from, List<StateTime> trajectory) {
    Scalar dt = getLast(trajectory).time.subtract(from.time);
    if (Scalars.lessEquals(dt, ZeroScalar.get()))
      throw new RuntimeException();
    return dt;
  }

  public static StateTime getLast(List<StateTime> list) {
    return list.get(list.size() - 1);
  }

  public static void print(List<StateTime> list) {
    System.out.println("Trajectory (" + list.size() + ")");
    for (StateTime stateTime : list)
      System.out.println(stateTime);
  }
}
