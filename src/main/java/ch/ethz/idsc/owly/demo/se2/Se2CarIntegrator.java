// code by jph
package ch.ethz.idsc.owly.demo.se2;

import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.flow.Integrator;
import ch.ethz.idsc.owly.math.flow.RungeKutta45Integrator;
import ch.ethz.idsc.owly.math.flow.RungeKutta4Integrator;
import ch.ethz.idsc.owly.math.se2.Se2Utils;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** exact integration of flow using matrix exponential and logarithm.
 * states are encoded in the default coordinates of the se2 lie-algebra.
 * 
 * Important: u is assumed to be of the form u == {vx, 0, rate}
 * 
 * Se2Integrator is approximately
 * 3x faster than {@link RungeKutta4Integrator}
 * 11x faster than {@link RungeKutta45Integrator} */
public enum Se2CarIntegrator implements Integrator {
  INSTANCE;
  // ---
  /** Parameter description:
   * g in SE2
   * h in R */
  @Override
  public Tensor step(Flow flow, Tensor g, Scalar h) {
    // u is assumed to be of the form u == {vx, 0, rate}
    return Se2Utils.integrate_vy0(g, flow.getU().multiply(h));
  }
}
