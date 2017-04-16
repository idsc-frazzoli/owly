// code by bapaden and jph
package ch.ethz.idsc.owly.glc.adapter;

import ch.ethz.idsc.owly.glc.core.DynamicalSystem;
import ch.ethz.idsc.owly.util.StateSpaceModel;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

public abstract class EulerIntegrator extends DynamicalSystem {
  @Override
  public final Tensor step(StateSpaceModel ssm, Tensor x, Tensor u, Scalar dt) {
    return x.add(ssm.flow(x, u).multiply(dt));
  }
}
