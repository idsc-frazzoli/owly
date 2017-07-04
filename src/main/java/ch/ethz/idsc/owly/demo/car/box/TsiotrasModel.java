// code by jph
package ch.ethz.idsc.owly.demo.car.box;

import ch.ethz.idsc.owly.demo.car.CarControl;
import ch.ethz.idsc.owly.demo.car.CarSteering;
import ch.ethz.idsc.owly.demo.car.DefaultCarModel;
import ch.ethz.idsc.owly.demo.car.TireInterface;
import ch.ethz.idsc.owly.math.car.Pacejka3;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.TensorRuntimeException;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Clip;

/** specifications of vehicle taken from:
 * Time-Optimal Vehicle Posture Control to Mitigate Unavoidable
 * Collisions Using Conventional Control Inputs
 * Chakraborty, Tsiotras, Diaz */
public class TsiotrasModel extends DefaultCarModel {
  public static TsiotrasModel standard() {
    return new TsiotrasModel(CarSteering.FRONT, RealScalar.ZERO);
  }

  // ---
  private final Tensor levers;
  private final CarSteering carSteering;
  private final Scalar gammaM;

  /** @param carSteering
   * @param gammaM rear/total drive ratio; 0 is FWD, 1 is RWD, 0.5 is AWD */
  public TsiotrasModel(CarSteering carSteering, Scalar gammaM) {
    this.carSteering = carSteering;
    this.gammaM = gammaM;
    final Pacejka3 PACEJKA = new Pacejka3(7, 1.4);
    final Scalar RADIUS = DoubleScalar.of(0.29); // wheel radius [m]
    final Scalar HEIGHT_COG = DoubleScalar.of(0.58); // height of COG [m]
    final Scalar LW = DoubleScalar.of(0.8375); // TODO unspecified lateral distance of wheels from COG [m]
    final Scalar LF = DoubleScalar.of(1.1); // front axle distance from COG [m]
    final Scalar LR = DoubleScalar.of(1.3); // rear axle distance from COG [m]
    Scalar h_negate = HEIGHT_COG.negate();
    levers = Tensors.of( //
        Tensors.of(LF, LW, h_negate), // 1L
        Tensors.of(LF, LW.negate(), h_negate), // 1R
        Tensors.of(LR.negate(), LW, h_negate), // 2L
        Tensors.of(LR.negate(), LW.negate(), h_negate) // 2R
    ).unmodifiable();
    // TODO define tires!
    // DoubleScalar.of(1 / 1.8); // wheel moment of inertia [kgm2]
  }

  // ---
  @Override
  public Scalar mass() {
    return DoubleScalar.of(1245); // mass [kg]
  }

  @Override
  public TireInterface tire(int index) {
    throw new RuntimeException();
  }

  @Override
  public Scalar Iz_invert() {
    return DoubleScalar.of(1 / 1200.0); // yawing moment of inertia [kgm2]
  }

  @Override
  public Scalar b() {
    return DoubleScalar.of(5); // dynamic friction coefficient N/(m/s)
  }

  @Override
  public Scalar fric() {
    return DoubleScalar.of(47); // coulomb friction
  }

  @Override
  public CarSteering steering() {
    return carSteering;
  }

  private static final Scalar maxDelta = DoubleScalar.of(45 * Math.PI / 180); // maximal steering angle [rad]
  // maximal motor torque [Nm], with gears included
  private static final Scalar maxPress = DoubleScalar.of(13.0); // TODO should result in 3000 Nm maximal master cylinder pressure [MPa]
  private static final Scalar maxThb = DoubleScalar.of(1000.0); // max handbrake torque [Nm]
  private static final Scalar maxThrottle = DoubleScalar.of(2000.0);

  @Override
  public CarControl createControl(Tensor u) {
    if (!Clip.ABSOLUTE_ONE.of(u.Get(0)).equals(u.Get(0)))
      throw TensorRuntimeException.of(u.Get(0));
    if (!Clip.UNIT.of(u.Get(3)).equals(u.Get(3)))
      throw TensorRuntimeException.of(u.Get(3));
    // ---
    Scalar delta = u.Get(0).multiply(maxDelta).multiply(carSteering.factor);
    Scalar brake = u.Get(1).multiply(maxPress);
    Scalar handbrake = u.Get(2).multiply(maxThb);
    Scalar throttle = u.Get(3).multiply(maxThrottle);
    return null; // new CarControl(Tensors.of(delta, brake, handbrake, throttle));
  }

  @Override
  public Scalar press2torF() {
    return DoubleScalar.of(250); // Nm per Mpa conversion constant [Nm/Mpa] for Front and Rear brakes
  }

  @Override
  public Scalar press2torR() {
    return DoubleScalar.of(150);
  }

  @Override
  public Scalar muRoll() {
    // TODO check if == 0 ok
    // for ==2 the car will not make a turn but slide in nose direction...
    return DoubleScalar.of(0); // rolling friction coefficient
  }
}
