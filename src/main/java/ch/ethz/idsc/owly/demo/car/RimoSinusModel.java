// code by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.owly.math.car.Pacejka3;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.TensorRuntimeException;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Clip;

/** specifications of vehicle taken from:
 * TODO */
public class RimoSinusModel extends DefaultCarModel {
  private static final Pacejka3 PACEJKA = new Pacejka3(7, 1.4); // TODO
  private static final Scalar RADIUS1 = DoubleScalar.of(0.255 * 0.5); // wheel radius [m]
  private static final Scalar RADIUS2 = DoubleScalar.of(0.278 * 0.5); // wheel radius [m]
  // TODO front wheel, back wheel
  private static final Scalar HEIGHT_COG = DoubleScalar.of(0.20); // height of COG [m]
  private static final Scalar LW = DoubleScalar.of(0.8375); // TODO unspecified lateral distance of wheels from COG [m]
  private static final Scalar LF = DoubleScalar.of(0.7); // TODO front axle distance from COG [m]
  private static final Scalar LR = DoubleScalar.of(0.7); // rear axle distance from COG [m]

  public static RimoSinusModel standard() {
    return new RimoSinusModel(CarSteering.FRONT, RealScalar.ZERO);
  }

  // ---
  private final Tensor levers;
  private final CarSteering carSteering;
  private final Scalar gammaM;

  /** @param carSteering
   * @param gammaM rear/total drive ratio; 0 is FWD, 1 is RWD, 0.5 is AWD */
  public RimoSinusModel(CarSteering carSteering, Scalar gammaM) {
    this.carSteering = carSteering;
    this.gammaM = gammaM;
    Scalar h_negate = HEIGHT_COG.negate();
    levers = Tensors.of( //
        Tensors.of(LF, LW, h_negate), // 1L
        Tensors.of(LF, LW.negate(), h_negate), // 1R
        Tensors.of(LR.negate(), LW, h_negate), // 2L
        Tensors.of(LR.negate(), LW.negate(), h_negate) // 2R
    ).unmodifiable();
  }

  // ---
  @Override
  public Scalar mass() {
    return DoubleScalar.of(170); // mass [kg]
  }

  // @Override
  // public Pacejka3 pacejka(int index) {
  // return PACEJKA;
  // }
  @Override
  public Scalar radius() {
    return RADIUS1;
  }

  @Override
  public Tensor levers() {
    return levers;
  }

  @Override
  public Scalar frontL() {
    return DoubleScalar.of(1.915); // distance from COG to front end [m]
  }

  @Override
  public Scalar rearL() {
    return DoubleScalar.of(2.835); // distance from COG to rear end [m]
  }

  @Override
  public Scalar width() {
    return DoubleScalar.of(1.916); // width of the vehicle [m]
  }

  @Override
  public Scalar gammaM() {
    return gammaM; // rear/total drive ratio; 0 is FWD, 1 is RWD
  }

  @Override
  public Scalar Iz_invert() {
    return DoubleScalar.of(1 / 1200.0); // yawing moment of inertia [kgm2]
  }

  @Override
  public Scalar Iw_invert() {
    return DoubleScalar.of(1 / 1.8); // wheel moment of inertia [kgm2]
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
    return new CarControl(Tensors.of(delta, brake, handbrake, throttle));
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

  @Override
  public Scalar rollFric() {
    return gForce().multiply(muRoll());
  }
}
