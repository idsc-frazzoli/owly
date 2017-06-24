// code by edo
// code adapted by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.TensorRuntimeException;
import ch.ethz.idsc.tensor.sca.Clip;

public class CarControl {
  public final Scalar delta; // in interval [-1, 1]
  public final Scalar brake;
  public final Scalar handbrake;
  public final Scalar throttle; // in interval [0, 1]

  public CarControl(Tensor u) {
    if (u.length() != 4)
      throw TensorRuntimeException.of(u);
    // ---
    delta = u.Get(0); // TODO used in code as absolute
    brake = u.Get(1);
    handbrake = u.Get(2);
    throttle = u.Get(3); // TODO used in code as relative to maxTm ...
    // ---
    if (!Clip.ABS_ONE.of(delta).equals(delta))
      throw TensorRuntimeException.of(delta);
    if (!Clip.UNIT.of(throttle).equals(throttle))
      throw TensorRuntimeException.of(throttle);
  }
}
