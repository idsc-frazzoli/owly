// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

public interface TrajectoryRegionQuery {
  public static final int NOMATCH = -1;

  /** @param trajectory
   * @return index of first {@link StateTime} element of trajectory that is in region
   * or -1 if no such element exists */
  public int firstMember(Trajectory trajectory);

  public default boolean isDisjoint(Trajectory trajectory) {
    return NOMATCH == firstMember(trajectory);
  }
}
