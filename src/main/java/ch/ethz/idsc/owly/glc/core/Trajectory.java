// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import java.util.ArrayList;

import ch.ethz.idsc.tensor.Scalar;

public class Trajectory extends ArrayList<StateTime> {
  public Scalar getDuration() {
    return getBack().time.subtract(get(0).time);
  }

  public Scalar getDuration(int num) {
    return get(num).time.subtract(get(0).time);
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
