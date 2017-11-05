// code by jph
package ch.ethz.idsc.owly.math.se2;

import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.flow.Integrator;
import ch.ethz.idsc.owly.math.flow.RungeKutta45Integrator;
import ch.ethz.idsc.owly.math.flow.RungeKutta4Integrator;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

/** exact integration of flow using matrix exponential and logarithm.
 * states are encoded in the default coordinates of the se2 lie-algebra.
 * 
 * Se2Integrator is approximately
 * 3x faster than {@link RungeKutta4Integrator}
 * 11x faster than {@link RungeKutta45Integrator} */
public enum Se2Integrator implements Integrator {
  INSTANCE;
  // ---
  /** Parameter description:
   * x in SE2
   * u in se2
   * h in R */
  @Override
  public Tensor step(Flow flow, Tensor x, Scalar h) {
    Tensor u = flow.getU(); // {angle, move}
    Scalar speed = u.Get(1).multiply(h);
    Tensor ux = Tensors.of( //
        speed, // h * move
        RealScalar.ZERO, //
        u.Get(0).multiply(speed) // h * move * angle
    );
    return Se2Utils.combine_vy0(x, ux); // ux is linear in h
  }
}
