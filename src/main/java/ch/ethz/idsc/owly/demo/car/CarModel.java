// code by edo
// code adapted by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.owly.math.Pacejka3;
import ch.ethz.idsc.tensor.Scalar;

// TODO function names remain as-is until system works
public interface CarModel {
  static final double Dz1 = 0.05; // TODO not final code design
  // ---

  /** @return mass [kg] */
  Scalar mass();

  Pacejka3 pacejka1();

  Pacejka3 pacejka2();

  /** @return tire radius */
  Scalar radius();

  /** @return height of COG above ground */
  Scalar heightCog();

  /** @return tire-road friction coefficient */
  Scalar mu();

  /** @return lateral distance of wheels from COG */
  Scalar lw();

  /** @return front axle distance from COG [m] */
  Scalar lF();

  /** @return rear axle distance from COG [m] */
  Scalar lR();

  /** @return distance from COG to front end [m] */
  Scalar frontL();

  /** @return distance from COG to rear end [m] */
  Scalar rearL();

  /** @return width of the vehicle [m] */
  Scalar width();

  /** @return maximal motor torque [Nm], with gears included */
  Scalar maxTm();

  /** @return rear/total drive ratio; 0 is FWD, 1 is RWD */
  Scalar gammaM();

  /** @return yawing moment of inertia [kgm2] */
  Scalar Iz_invert();

  /** @return wheel moment of inertia [kgm2] */
  Scalar Iw_invert();

  /***************************************************/
  Scalar gForce();

  Scalar radiusTimes(Scalar omega);

  Scalar noSlipRate(Scalar speed);
}
