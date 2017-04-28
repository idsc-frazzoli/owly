// code by jph
package ch.ethz.idsc.owly.math.flow;

import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

/** fourth-order Runge-Kutta formula
 * 
 * Numerical Recipes 3rd Edition (17.1.3) */
public final class RungeKutta4Integrator implements Integrator {
  private static final Scalar HALF = RationalScalar.of(1, 2);
  private static final Scalar THIRD = RationalScalar.of(1, 3);
  private static final Scalar SIXTH = RationalScalar.of(1, 6);
  private static final Tensor WEIGHTS = Tensors.of(SIXTH, THIRD, THIRD, SIXTH);

  @Override
  public Tensor step(Flow flow, Tensor x, Scalar h) {
    Tensor k1 = flow.at(x).multiply(h);
    Tensor k2 = flow.at(x.add(k1.multiply(HALF))).multiply(h);
    Tensor k3 = flow.at(x.add(k2.multiply(HALF))).multiply(h);
    Tensor k4 = flow.at(x.add(k3)).multiply(h);
    return x.add(WEIGHTS.dot(Tensors.of(k1, k2, k3, k4)));
  }
}
