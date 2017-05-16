// code by jph
package ch.ethz.idsc.owly.demo.glc.twd;

import java.util.Collection;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.Scalar;

/** two wheel drive */
public class TwdControls {
  public static Collection<Flow> createControls(Scalar angle_max, int num) {
    StateSpaceModel stateSpaceModel = new TwdStateSpaceModel();
    // FIXME
    throw new RuntimeException();
  }
}
