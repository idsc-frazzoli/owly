// code by jph
package ch.ethz.idsc.owly.demo.aav;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** Agile Aerial Vehicle
 * 
 * bapaden phd thesis: (5.5.9) */
class AavStateSpaceModel implements StateSpaceModel {
  @Override
  public Tensor f(Tensor x, Tensor u) {
    // TODO implement
    throw new RuntimeException();
  }

  /** | f(x_1, u) - f(x_2, u) | <= L | x_1 - x_2 | */
  @Override
  public Scalar getLipschitz() {
    throw new RuntimeException();
  }
}
