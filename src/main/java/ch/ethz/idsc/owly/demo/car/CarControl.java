// code by edo
// code adapted by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.TensorRuntimeException;

public class CarControl {
  public final Scalar delta;
  public final Scalar brake;
  public final Scalar handbrake;
  public final Scalar throttle;

  public CarControl(Tensor u) {
    if (u.length() != 4)
      throw TensorRuntimeException.of(u);
    // ---
    delta = u.Get(0);
    brake = u.Get(1);
    handbrake = u.Get(2);
    throttle = u.Get(3);
  }
}
