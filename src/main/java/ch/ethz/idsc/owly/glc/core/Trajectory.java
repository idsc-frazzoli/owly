// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import java.util.List;

public class Trajectory {
  public static StateTime getLast(List<StateTime> list) {
    return list.get(list.size() - 1);
  }

  public static void print(List<StateTime> list) {
    System.out.println("Trajectory (" + list.size() + ")");
    for (StateTime stateTime : list)
      System.out.println(stateTime);
  }
}
