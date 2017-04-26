// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import java.util.List;

import ch.ethz.idsc.tensor.Scalar;

public class Trajectory {
  public static Scalar getDuration(List<StateTime> list) {
    return getBack(list).time.subtract(list.get(0).time);
  }

  public static StateTime getBack(List<StateTime> list) {
    return list.get(list.size() - 1);
  }

  public static void print(List<StateTime> list) {
    System.out.println("Trajectory (" + list.size() + ")");
    for (StateTime stateTime : list)
      System.out.println(stateTime);
  }
  // public Trajectory copy() {
  // Trajectory trajectory = new Trajectory();
  // trajectory.addAll(this);
  // return trajectory;
  // }
}
