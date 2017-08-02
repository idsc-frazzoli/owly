// code by jph
package ch.ethz.idsc.owly.glc.adapter;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import ch.ethz.idsc.owly.data.GlobalAssert;
import ch.ethz.idsc.owly.glc.core.TrajectorySample;

public enum Trajectories {
  ;
  /** @param head
   * @param tail
   * @return trajectory { head[0:end], tail[1:end]}
   * @throws Exception if head[end] != tail[0] */
  public static List<TrajectorySample> glue(List<TrajectorySample> head, List<TrajectorySample> tail) {
    List<TrajectorySample> trajectory = new ArrayList<>();
    trajectory.addAll(head);
    TrajectorySample tsh = head.get(head.size() - 1);
    TrajectorySample tst = tail.get(0);
    GlobalAssert.that(tsh.stateTime().equals(tst.stateTime()));
    GlobalAssert.that(!tst.getFlow().isPresent());
    // System.out.println("last of head: " + tsh.toInfoString());
    // System.out.println(" 1st of tail: " + tst.toInfoString());
    trajectory.addAll(tail.subList(1, tail.size()));
    return Collections.unmodifiableList(trajectory);
  }
}
