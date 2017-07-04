// code by jph
package ch.ethz.idsc.owly.demo.twd;

import java.util.Collection;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.Scalar;

/** two wheel drive */
public enum TwdControls {
  ;
  public static Collection<Flow> createControls1(Scalar angle_max, int num) {
    @SuppressWarnings("unused")
    StateSpaceModel stateSpaceModel = new TwdStateSpaceModel();
    // TODO JONAS implement, zB |wl|<=1
    throw new RuntimeException();
  }

  public static Collection<Flow> createControls2(Scalar angle_max, int num) {
    @SuppressWarnings("unused")
    StateSpaceModel stateSpaceModel = new TwdStateSpaceModel();
    // TODO JONAS implement, zB |wl|+|wr|<=1
    throw new RuntimeException();
  }
}
