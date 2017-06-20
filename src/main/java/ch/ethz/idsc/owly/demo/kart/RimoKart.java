// code by marcello
// code adapted by jph
package ch.ethz.idsc.owly.demo.kart;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;

// EXPERIMENTAL API not final
// all values in SI units
class RimoKart {
  // radius of tire
  public static final Scalar r_FL = RealScalar.of((255e-3) / 2);
  public static final Scalar r_FR = RealScalar.of((255e-3) / 2); // assumed to be identical to r_FL
  public static final Scalar r_RL = RealScalar.of((278e-3) / 2);
  public static final Scalar r_RR = RealScalar.of((278e-3) / 2); // assumed to be identical to r_RL
  public static final Scalar m = RealScalar.of(170); // kg
  public static final Scalar length = RealScalar.of(2);
  public static final Scalar l_F = RealScalar.of(2 / 3 * 2); // length
  // TODO why not negative?
  public static final Scalar l_R = RealScalar.of(1 / 3 * 2); // RealScalar.of(
  public static final Scalar width = RealScalar.of(1.4);
  public static final Scalar w_R = RealScalar.of(1.4 / 2);
  public static final Scalar w_L = RealScalar.of(1.4 / 2);
  public static final Scalar I_z = RealScalar.of(25);
  public static final Scalar I_w = RealScalar.of(20);
}
