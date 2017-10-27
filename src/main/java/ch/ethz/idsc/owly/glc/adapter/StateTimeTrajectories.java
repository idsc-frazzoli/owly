// code by jph
package ch.ethz.idsc.owly.glc.adapter;

import java.util.List;

import ch.ethz.idsc.owly.data.GlobalAssert;
import ch.ethz.idsc.owly.data.Lists;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.sca.Sign;

/** utility functions that operate on List<StateTime> */
public enum StateTimeTrajectories {
  ;
  // ---
  /** @param stateTime
   * @param trajectory
   * @return time increment between given from State and end of trajectory
   * @throws Exception if time of last node is smaller than of given stateTime */
  public static Scalar timeIncrement(StateTime stateTime, List<StateTime> trajectory) {
    Scalar dt = Lists.getLast(trajectory).time().subtract(stateTime.time());
    GlobalAssert.that(Sign.isPositiveOrZero(dt));
    return dt;
  }

  /** see description of function {@link #timeIncrement(StateTime, List)}
   * 
   * @param glcNode
   * @param trajectory
   * @return */
  public static Scalar timeIncrement(GlcNode glcNode, List<StateTime> trajectory) {
    return timeIncrement(glcNode.stateTime(), trajectory);
  }

  public static void print(List<StateTime> list) {
    System.out.println("Trajectory (" + list.size() + ")");
    for (StateTime stateTime : list)
      System.out.println(stateTime.toInfoString());
  }
}
