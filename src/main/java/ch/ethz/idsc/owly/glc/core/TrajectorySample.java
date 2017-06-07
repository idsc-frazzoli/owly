// code by jph
package ch.ethz.idsc.owly.glc.core;

import java.io.Serializable;

import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.Tensor;

/** container class that bundles information to follow a trajectory */
public class TrajectorySample implements Serializable {
  private final StateTime stateTime;
  private final Flow flow;

  public TrajectorySample(StateTime stateTime, Flow flow) {
    this.stateTime = stateTime;
    this.flow = flow;
  }

  public StateTime stateTime() {
    return stateTime;
  }

  /** @return true, if function getU() provides control information */
  public boolean hasU() {
    return flow != null;
  }

  /** @return unmodifiable control identifier */
  public Tensor getU() {
    return flow.getU();
  }
}
