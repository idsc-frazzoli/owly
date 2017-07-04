// code by edo
// code adapted by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.owly.math.Deadzone;
import ch.ethz.idsc.owly.math.PhysicalConstants;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.lie.RotationMatrix;
import ch.ethz.idsc.tensor.red.Total;
import ch.ethz.idsc.tensor.sca.Chop;
import ch.ethz.idsc.tensor.sca.Round;

/** the matlab code applies a rate limiter to u
 * if this is beneficial for stability, the limiter should
 * be a layer outside of the state space model */
public class CarStateSpaceModel implements StateSpaceModel {
  private final VehicleModel vehicleModel;
  private final TrackInterface trackInterface;

  /** @param vehicleModel
   * @param mu friction coefficient of tire on road */
  public CarStateSpaceModel(VehicleModel vehicleModel, TrackInterface trackInterface) {
    this.vehicleModel = vehicleModel;
    this.trackInterface = trackInterface;
  }

  private long tic = 0;

  @Override
  public Tensor f(Tensor x, Tensor u) {
    // u may need to satisfy certain conditions with respect to previous u
    CarState carState = new CarState(x);
    CarControl carControl = vehicleModel.createControl(u);
    TireForces tire = new TireForces( //
        vehicleModel, carState, carControl, trackInterface.mu(carState.asVector()));
    BrakeTorques brakeTorques = new BrakeTorques(vehicleModel, carState, carControl, tire);
    // ---
    final Scalar dux;
    final Scalar gForce = vehicleModel.mass().multiply(PhysicalConstants.G_EARTH);
    // TODO friction from drag
    final Scalar rollFric = gForce.multiply(vehicleModel.muRoll()); // TODO at the moment == 0!
    Deadzone deadzone = Deadzone.of(rollFric.negate(), rollFric);
    final Tensor total = Total.of(tire.Forces);
    {
      // Scalar
      Scalar dF_z = total.Get(2).subtract(gForce);
      if (Scalars.nonZero((Scalar) dF_z.map(Chop.below(1e-5))))
        System.out.println("dF_z=" + dF_z);
    }
    {
      Scalar prel = total.Get(0).add(vehicleModel.mass().multiply(carState.Uy).multiply(carState.r));
      dux = deadzone.apply(prel).subtract(vehicleModel.coulombFriction(carState.Ux)).divide(vehicleModel.mass());
    }
    // ---
    final Scalar duy;
    {
      Scalar prel = total.Get(1).subtract(vehicleModel.mass().multiply(carState.Ux).multiply(carState.r));
      duy = deadzone.apply(prel).subtract(RealScalar.ZERO.multiply(vehicleModel.coulombFriction(carState.Uy))).divide(vehicleModel.mass());
    }
    // ---
    Scalar dr;
    {
      Tensor torque = tire.torque();
      if (!tire.isTorqueConsistent() || !tire.isFzConsistent()) {
        long toc = System.currentTimeMillis();
        if (tic + 987 <= toc) {
          tic = toc;
          System.out.println("---");
          System.out.println("Tq=" + torque.map(Round._2));
          Scalar f03 = tire.Forces.Get(0, 2).add(tire.Forces.Get(3, 2));
          Scalar f12 = tire.Forces.Get(1, 2).add(tire.Forces.Get(2, 2));
          System.out.println("Fz=" + Tensors.of(f03, f12).map(Round._2));
        }
      }
      dr = torque.Get(2).multiply(vehicleModel.Iz_invert());
    }
    Tensor dp = RotationMatrix.of(carState.Ksi).dot(carState.u_2d());
    // ---
    Scalar dw1L = carControl.throttleV.Get(0).add(brakeTorques.Tb1L).subtract(vehicleModel.wheel(0).radius().multiply(tire.fwheel.Get(0, 0)))
        .multiply(vehicleModel.wheel(0).Iw_invert());
    Scalar dw1R = carControl.throttleV.Get(1).add(brakeTorques.Tb1R).subtract(vehicleModel.wheel(1).radius().multiply(tire.fwheel.Get(1, 0)))
        .multiply(vehicleModel.wheel(1).Iw_invert());
    Scalar dw2L = carControl.throttleV.Get(2).add(brakeTorques.Tb2L).subtract(vehicleModel.wheel(2).radius().multiply(tire.fwheel.Get(2, 0)))
        .multiply(vehicleModel.wheel(2).Iw_invert());
    Scalar dw2R = carControl.throttleV.Get(3).add(brakeTorques.Tb2R).subtract(vehicleModel.wheel(3).radius().multiply(tire.fwheel.Get(3, 0)))
        .multiply(vehicleModel.wheel(3).Iw_invert());
    // ---
    Tensor fxu = Tensors.of( //
        dux, duy, //
        dr, //
        carState.r, //
        dp.Get(0), dp.Get(1), //
        dw1L, dw1R, dw2L, dw2R);
    // the observation is that dwXY oscillate a lot!
    // this is consistent with the MATLAB code
    // if (Scalars.lessThan(RealScalar.of(1e4), Norm.Infinity.of(fxu)))
    // System.out.println(fxu);
    return fxu;
  }

  @Override
  public Scalar getLipschitz() {
    return RealScalar.ONE; // null
  }
}
