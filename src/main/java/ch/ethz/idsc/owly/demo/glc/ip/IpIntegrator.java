// code by jph
package ch.ethz.idsc.owly.demo.glc.ip;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** inverted pendulum */
class IpStateSpaceModel implements StateSpaceModel {
  @Override
  public Tensor createFlow(Tensor x, Tensor u) {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public Scalar getLipschitz() {
    // TODO Auto-generated method stub
    return null;
  }
}
