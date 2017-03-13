// code by bapaden and jph
package ch.ethz.idsc.owly.glc.adapter;

import ch.ethz.idsc.owly.glc.core.DynamicalSystem;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.Tensor;

public abstract class EulerIntegrator extends DynamicalSystem {
  @Override
  public final Tensor step(Tensor x, Tensor u, double dt) {
    return x.add(flow(x, u).multiply(DoubleScalar.of(dt)));
  }
}
