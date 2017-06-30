// code by edo
// code adapted by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Sign;

public class BrakeTorques {
  // TODO store torques as vector
  public final Scalar Tb1L; // 1
  public final Scalar Tb1R; // 2
  public final Scalar Tb2L; // 3
  public final Scalar Tb2R; // 4

  public BrakeTorques(CarModel params, CarState cs, CarControl cc, TireForces tireForces) {
    Scalar _Tb1L = RealScalar.ZERO;
    Scalar _Tb1R = RealScalar.ZERO;
    Scalar _Tb2L = RealScalar.ZERO;
    Scalar _Tb2R = RealScalar.ZERO;
    // ---
    Scalar masterPress = cc.brake;
    final Scalar pressF = masterPress;
    final Scalar pressR;
    // ---
    if (Scalars.lessEquals(masterPress, RealScalar.of(1.5))) {
      pressR = masterPress;
    } else {
      pressR = RealScalar.of(0.3).multiply(masterPress).add(RealScalar.of(1.05));
    }
    // ---
    if (Scalars.lessThan(RealScalar.ZERO, masterPress)) {
      if (Scalars.nonZero(cs.omega.Get(0))) {
        _Tb1L = pressF.multiply(params.press2torF()).multiply(Sign.of(cs.omega.Get(0))).negate();
      } else {
        _Tb1L = tireForces.fwheel.Get(0, 0).multiply(params.radius());
      }
      //
      if (Scalars.nonZero(cs.omega.Get(1))) {
        _Tb1R = pressF.multiply(params.press2torF()).multiply(Sign.of(cs.omega.Get(1))).negate();
      } else {
        _Tb1R = tireForces.fwheel.Get(1, 0).multiply(params.radius());
      }
      //
      if (Scalars.nonZero(cs.omega.Get(2))) {
        _Tb2L = pressR.multiply(params.press2torR()).multiply(Sign.of(cs.omega.Get(2))).negate();
      } else {
        _Tb2L = tireForces.fwheel.Get(2, 0).multiply(params.radius());
      }
      //
      if (Scalars.nonZero(cs.omega.Get(3))) {
        _Tb2R = pressR.multiply(params.press2torR()).multiply(Sign.of(cs.omega.Get(3))).negate();
      } else {
        _Tb2R = tireForces.fwheel.Get(3, 0).multiply(params.radius());
      }
    }
    // ---
    if (Scalars.lessThan(RealScalar.ZERO, cc.handbrake)) {
      if (Scalars.nonZero(cs.omega.Get(2))) {
        _Tb2L = _Tb2L.subtract(cc.handbrake.multiply(Sign.of(cs.omega.Get(2))));
      } else {
        _Tb2L = _Tb2L.subtract(tireForces.fwheel.Get(2, 0).multiply(params.radius()));
      }
      //
      if (Scalars.nonZero(cs.omega.Get(3))) {
        _Tb2R = _Tb2R.subtract(cc.handbrake.multiply(Sign.of(cs.omega.Get(3))));
      } else {
        _Tb2R = _Tb2R.subtract(tireForces.fwheel.Get(3, 0).multiply(params.radius()));
      }
    }
    // ---
    Tb1L = _Tb1L;
    Tb1R = _Tb1R;
    Tb2L = _Tb2L;
    Tb2R = _Tb2R;
  }

  public Tensor asVector() {
    return Tensors.of(Tb1L, Tb1R, Tb2L, Tb2R);
  }
}
