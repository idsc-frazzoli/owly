// code by jph
package ch.ethz.idsc.owly.math.state;

import java.io.Serializable;

public interface StateTimeRegionCallback extends Serializable {
  void notify_isMember(StateTime stateTime);
}
