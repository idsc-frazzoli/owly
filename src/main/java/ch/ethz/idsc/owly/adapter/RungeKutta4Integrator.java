// code by jph
package ch.ethz.idsc.owly.adapter;

import ch.ethz.idsc.owly.util.Integrator;
import ch.ethz.idsc.owly.util.StateSpaceModel;
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
  public Tensor step(StateSpaceModel stateSpaceModel, Tensor x, Tensor u, Scalar dt) {
    Tensor k1 = stateSpaceModel.flow(x, u).multiply(dt);
    Tensor k2 = stateSpaceModel.flow(x.add(k1.multiply(HALF)), u).multiply(dt);
    Tensor k3 = stateSpaceModel.flow(x.add(k2.multiply(HALF)), u).multiply(dt);
    Tensor k4 = stateSpaceModel.flow(x.add(k3), u).multiply(dt);
    return x.add(WEIGHTS.dot(Tensors.of(k1, k2, k3, k4)));
  }
}
