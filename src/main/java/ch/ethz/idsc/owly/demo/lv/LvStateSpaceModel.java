// code by jph
package ch.ethz.idsc.owly.demo.lv;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

/** Lotka-Volterra */
public class LvStateSpaceModel implements StateSpaceModel {
  // TODO make depend on parameters
  @Override
  public Tensor f(Tensor x, Tensor u) {
    // TODO use control to decimate predators
    // Mathematica
    // x0' = x0*(x1-1)
    // x1' = y1*(2-x0)
    return Tensors.of( //
        x.Get(0).multiply(x.Get(1).subtract(RealScalar.ONE)), //
        x.Get(1).multiply(RealScalar.of(2).subtract(x.Get(0))) //
    );
  }

  @Override
  public Scalar getLipschitz() {
    return RealScalar.ONE; // TODO
  }
}
