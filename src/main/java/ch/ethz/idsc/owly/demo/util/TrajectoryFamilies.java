// code by jph
package ch.ethz.idsc.owly.demo.util;

import java.util.List;
import java.util.NavigableMap;
import java.util.TreeMap;
import java.util.stream.Collectors;

import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.se2.RigidFamily;
import ch.ethz.idsc.owly.math.se2.TranslationFamily;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

public enum TrajectoryFamilies {
  ;
  /** @param stateIntegrator
   * @param initial
   * @param flow
   * @return */
  public static RigidFamily create(StateIntegrator stateIntegrator, StateTime initial, Flow flow) {
    List<StateTime> trajectory = stateIntegrator.trajectory(initial, flow);
    NavigableMap<Scalar, Tensor> navigableMap = trajectory.stream().collect( //
        Collectors.toMap(StateTime::time, stateTime -> stateTime.state().extract(0, 2), (t1, t2) -> t1, TreeMap::new));
    navigableMap.put(initial.time(), initial.state().extract(0, 2));
    return new TranslationFamily(time -> navigableMap.floorEntry(time).getValue());
  }
}
