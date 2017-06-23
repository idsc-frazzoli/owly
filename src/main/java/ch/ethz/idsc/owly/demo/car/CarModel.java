// code by edo
// code adapted by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.owly.math.Pacejka3;
import ch.ethz.idsc.tensor.Scalar;

public interface CarModel {
  static final double Dz1 = 0.05; // TODO not final code design
  // ---

  Scalar mass();

  Scalar gForce();

  Pacejka3 pacejka1();

  Pacejka3 pacejka2();

  Scalar radius();

  Scalar heightCog();

  Scalar mu();

  Scalar lw();

  Scalar lF();

  Scalar lR();

  /***************************************************/
  Scalar radiusTimes(Scalar omega);

  Scalar noSlipRate(Scalar speed);
}
