// code by edo
// code adapted by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.owly.math.Deadzone;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Array;
import ch.ethz.idsc.tensor.alg.Transpose;
import ch.ethz.idsc.tensor.lie.Cross;
import ch.ethz.idsc.tensor.mat.RotationMatrix;
import ch.ethz.idsc.tensor.red.Total;
import ch.ethz.idsc.tensor.sca.Chop;
import ch.ethz.idsc.tensor.sca.Round;

/** the matlab code applies a rate limiter to u
 * if this is beneficial for stability, the limiter should
 * be a layer outside of the state space model */
public class CarStateSpaceModel implements StateSpaceModel {
  private final CarModel params;

  public CarStateSpaceModel(CarModel carModel) {
    this.params = carModel;
  }

  private long tic = 0;

  @Override
  public Tensor f(Tensor x, Tensor u) {
    // TODO u may need to satisfy certain conditions with respect to previous u
    CarState cs = new CarState(x);
    CarControl cc = new CarControl(u);
    TireForces tire = new TireForces(params, cs, cc);
    BrakeTorques brakeTorques = new BrakeTorques(params, cs, cc, tire);
    MotorTorques torques = new MotorTorques(params, cc.throttle);
    // ---
    final Scalar dux;
    final Scalar rollFric = params.rollFric(); // TODO at the moment == 0!
    Deadzone deadzone = Deadzone.of(rollFric.negate(), rollFric);
    final Tensor total = Total.of(tire.Forces);
    {
      Scalar dF_z = total.Get(2).subtract(params.gForce());
      if (Scalars.nonZero((Scalar) dF_z.map(Chop.below(1e-5))))
        System.out.println("dF_z=" + dF_z);
    }
    {
      Scalar prel = total.Get(0).add(params.mass().multiply(cs.Uy).multiply(cs.r));
      dux = deadzone.apply(prel).subtract(params.coulombFriction(cs.Ux)).divide(params.mass());
    }
    // ---
    final Scalar duy;
    {
      Scalar prel = total.Get(1).subtract(params.mass().multiply(cs.Ux).multiply(cs.r));
      duy = deadzone.apply(prel).subtract(RealScalar.ZERO.multiply(params.coulombFriction(cs.Uy))).divide(params.mass());
    }
    // ---
    Scalar dr;
    {
      Tensor torque = Array.zeros(3);
      for (int index = 0; index < params.levers().length(); ++index)
        torque = torque.add(Cross.of(params.levers().get(index), tire.Forces.get(index)));
      {
        // TODO assert that components 0, and 1 are == 0! at the moment they are not
        long toc = System.currentTimeMillis();
        if (tic + 987 <= toc) {
          tic = toc;
          System.out.println("---");
          System.out.println("Tq=" + torque.map(Round._2));
          Tensor Fz = Transpose.of(tire.Forces).get(2);
          Fz = Tensors.of(Fz.Get(0).add(Fz.Get(3)), Fz.Get(1).add(Fz.Get(2)));
          System.out.println("Fz=" + Fz.map(Round._2));
        }
      }
      dr = torque.Get(2).multiply(params.Iz_invert());
    }
    Tensor dp = RotationMatrix.of(cs.Ksi).dot(cs.u_2d());
    // ---
    Scalar dw1L = torques.Tm1L.add(brakeTorques.Tb1L).subtract(params.radiusTimes(tire.fwheel.Get(0, 0))).multiply(params.Iw_invert());
    Scalar dw1R = torques.Tm1R.add(brakeTorques.Tb1R).subtract(params.radiusTimes(tire.fwheel.Get(1, 0))).multiply(params.Iw_invert());
    Scalar dw2L = torques.Tm2L.add(brakeTorques.Tb2L).subtract(params.radiusTimes(tire.fwheel.Get(2, 0))).multiply(params.Iw_invert());
    Scalar dw2R = torques.Tm2R.add(brakeTorques.Tb2R).subtract(params.radiusTimes(tire.fwheel.Get(3, 0))).multiply(params.Iw_invert());
    // ---
    return Tensors.of( //
        dux, duy, //
        dr, //
        cs.r, //
        dp.Get(0), dp.Get(1), //
        dw1L, dw1R, dw2L, dw2R);
  }

  @Override
  public Scalar getLipschitz() {
    return RealScalar.ONE; // null
  }
}
