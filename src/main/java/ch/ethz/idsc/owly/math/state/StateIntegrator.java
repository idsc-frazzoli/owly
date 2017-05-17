// code by jph
package ch.ethz.idsc.owly.math.state;

import java.io.Serializable;
import java.util.List;

import ch.ethz.idsc.owly.math.flow.Flow;

public interface StateIntegrator extends Serializable {
  /** collects {@link StateTime}s along trajectory from starting point along flow
   * until a stop criterion is met
   * 
   * @param stateTime starting point
   * @param flow
   * @return */
  List<StateTime> trajectory(StateTime stateTime, Flow flow);
}
