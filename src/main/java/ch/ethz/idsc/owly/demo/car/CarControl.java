// code by edo
// code adapted by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.owly.math.car.SteeringWheelAngle;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.TensorRuntimeException;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.red.Mean;

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
    return Tensors.of( //
        delta, // 1L
        delta, // 1R
        RealScalar.ZERO, // 2L
        RealScalar.ZERO // 2R
    );
  }

  /** @return angles represent rotation around z-axis */
  public Tensor tire_angles(CarModel params) {
    Tensor rear_center = Mean.of(params.levers().extract(2, 4));
    Tensor p1L = params.levers().get(0).subtract(rear_center);
    Tensor p1R = params.levers().get(1).subtract(rear_center);
    Scalar a1L = SteeringWheelAngle.of(p1L.Get(1).divide(p1L.Get(0)), delta);
    Scalar a1R = SteeringWheelAngle.of(p1R.Get(1).divide(p1R.Get(0)), delta);
    return Tensors.of( //
        a1L, // 1L
        a1R, // 1R
        RealScalar.ZERO, // 2L
        RealScalar.ZERO // 2R
    );
  }
}
