// code by jph
package ch.ethz.idsc.owly.math.state;

import java.io.Serializable;

// EXPERIMENTAL API not finalized
public interface StateTimeRegionCallback extends Serializable {
  void notify_isMember(StateTime stateTime);
}
