// code by jph
package ch.ethz.idsc.owly.demo.se2;

import ch.ethz.idsc.owly.demo.util.FlowsInterface;
import ch.ethz.idsc.owly.math.StateSpaceModels;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.N;

public abstract class CarFlows implements FlowsInterface {
  /** @param speed, positive for forward, and negative for backward, unit [m/s]
   * @param rate of steering, unit [rad/m]
   * @return flow with u == {speed[m*s^-1], 0.0, (rate*speed)[rad*s^-1]} */
  public static Flow singleton(Scalar speed, Tensor rate) {
    return StateSpaceModels.createFlow(Se2StateSpaceModel.INSTANCE, //
        N.DOUBLE.of(Tensors.of(speed, RealScalar.ZERO, rate.multiply(speed))));
  }
}
