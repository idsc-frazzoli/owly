// code by edo
// code adapted by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

public interface VehicleModel {
  /** @return mass [kg] */
  Scalar mass();

  /** @return vectors from COG to wheel centers in local coordinates (=invariant over time)
   * for instance if the car has 4 wheels, then
   * levers() = {
   * {+1.1,+1,-h}, // 1L
   * {+1.1,-1,-h}, // 1R
   * {-1.2,+1,-h}, // 2L
   * {-1.2,-1,-h} // 2R
   * }
   * and h = heightCog() */
  @Deprecated
  Tensor levers(); // TODO remove

  /** @return number of tires */
  int tires();

  /** @param index
   * @return description of tire of given index */
  TireInterface tire(int index);

  /** @param delta steering angle
   * @return angles of wheels (measured from longitude forward direction)
   * for instance if the car has 4 wheels and traditional steering then
   * angles(delta) = {~delta, ~delta, 0, 0} */
  Tensor angles(Scalar delta);

  // TODO consolidate visualization info into one concept
  /** @return distance from COG to front end [m] */
  @Deprecated
  Scalar frontL(); // only used for visualization

  /** @return distance from COG to rear end [m] */
  @Deprecated
  Scalar rearL(); // only used for visualization

  /** @return width of the vehicle [m] */
  @Deprecated
  Scalar width(); // only used for visualization

  /** @return rear/total drive ratio; 0 is FWD, 1 is RWD */
  Scalar gammaM();

  /** @return inverse of yawing moment of inertia [kgm2] */
  Scalar Iz_invert();

  /** @return inverse of wheel moment of inertia [kgm2] */
  Scalar Iw_invert();

  /** @param tensor with relative control parameters in range [-1,1], or [0,1]
   * @return control with absolute physical values */
  CarControl createControl(Tensor tensor);

  Scalar coulombFriction(Scalar speed);

  /** @return Nm per Mpa conversion constant [Nm/Mpa] for Front brakes */
  Scalar press2torF();

  /** @return Nm per Mpa conversion constant [Nm/Mpa] for Rear brakes */
  Scalar press2torR();

  Scalar muRoll();
}
