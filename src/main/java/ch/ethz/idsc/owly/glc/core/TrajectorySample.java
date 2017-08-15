// code by jph
package ch.ethz.idsc.owly.glc.core;

import java.io.Serializable;
import java.util.Objects;
import java.util.Optional;

import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.StateTime;

/** container class that bundles information to follow a trajectory */
public class TrajectorySample implements Serializable {
  /** @param stateTime
   * @return first entry of a trajectory that does not specify flow */
  public static TrajectorySample head(StateTime stateTime) {
    return new TrajectorySample(stateTime, null);
  }

  // ---
  private final StateTime stateTime;
  private final Flow flow;

  public TrajectorySample(StateTime stateTime, Flow flow) {
    this.stateTime = stateTime;
    this.flow = flow;
  }

  public StateTime stateTime() {
    return stateTime;
  }

  /** typically the first state time in a trajectory
   * may not have a flow associated
   * (since there may not be history for the sample)
   * 
   * We return an optional type to make the application layer
   * aware of the possibility that flow may be null.
   * 
   * @return Optional.ofNullable(flow) */
  public Optional<Flow> getFlow() {
    return Optional.ofNullable(flow);
  }

  public String toInfoString() {
    String ustring = Objects.isNull(flow) ? "null" : flow.getU().toString();
    return stateTime.toInfoString() + "  u=" + ustring;
  }

  @Override // from Object
  public int hashCode() {
    return Objects.hash(stateTime, flow);
  }

  @Override // from Object
  public boolean equals(Object object) {
    if (object instanceof TrajectorySample) {
      // FIXME JONAS this is a bug
      // TODO JONAS document why these functions are necessary...
      // since they are not implemented correctly, they may not be necessary...?
      TrajectorySample trajectorySample = (TrajectorySample) object;
      return stateTime().equals(stateTime.state()) && flow.equals(stateTime.time());
    }
    return false;
  }
}
