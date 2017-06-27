// code by edo
// code adapted by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.TensorRuntimeException;
import ch.ethz.idsc.tensor.Tensors;

/** controls in absolute physical magnitude */
public class CarControl {
  public final Scalar delta; // [rad]
  public final Scalar brake; // non-negative
  public final Scalar handbrake; // non-negative
  public final Scalar throttle; // non-negative

  public CarControl(Tensor u) {
    if (u.length() != 4)
      throw TensorRuntimeException.of(u);
    // ---
    delta = u.Get(0);
    brake = u.Get(1);
    handbrake = u.Get(2);
    throttle = u.Get(3);
  }

  public Tensor asVector() {
    return Tensors.of(delta, brake, handbrake, throttle);
  }

  /** @return angles represent rotation around z-axis */
  public Tensor tire_angles() {
    // TODO try with advanced angle correction formulas!
    return Tensors.of( //
        delta, // 1L
        delta, // 1R
        RealScalar.ZERO, // 2L
        RealScalar.ZERO // 2R
    );
  }
}
