// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import java.util.ArrayList;

public class Trajectory extends ArrayList<StateTime> {
  public double getDuration() {
    return getBack().time - get(0).time;
  }

  public double getDuration(int num) {
    return get(num).time - get(0).time;
  }

  public StateTime getBack() {
    return get(size() - 1);
  }

  public void print() {
    System.out.println("Trajectory (" + size() + ")");
    for (StateTime stateTime : this)
      System.out.println(stateTime);
  }
}
