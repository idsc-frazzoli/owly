// code by jph
package ch.ethz.idsc.owly.math.flow;

import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** fifth-order Runge-Kutta formula based on RK4
 * 
 * Numerical Recipes 3rd Edition (17.2.3) */
public class RungeKutta45Integrator extends RungeKutta4Integrator {
  private static final Scalar W1 = RationalScalar.of(-1, 15);
  private static final Scalar W2 = RationalScalar.of(16, 15);

  @Override
  public Tensor step(Flow flow, Tensor x, Scalar h) {
    Tensor y1 = increment(flow, x, h);
    Scalar h2 = h.multiply(HALF);
    Tensor xm = super.step(flow, x, h2);
    Tensor y2 = super.step(flow, xm, h2).subtract(x);
    Tensor ya = y1.multiply(W1).add(y2.multiply(W2));
    return x.add(ya);
  }
}
