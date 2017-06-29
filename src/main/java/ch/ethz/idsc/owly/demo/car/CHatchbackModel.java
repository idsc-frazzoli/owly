// code by edo
// code adapted by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.owly.math.car.Pacejka3;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.TensorRuntimeException;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Clip;

public class CHatchbackModel extends DefaultCarModel {
  private static final Pacejka3 PACEJKA1 = new Pacejka3(13.8509, 1.3670, 0.9622);
  private static final Pacejka3 PACEJKA2 = new Pacejka3(14.1663, 1.3652, 0.9744);
  private static final Scalar RADIUS = RealScalar.of(0.325); // wheel radius [m]
  private static final Scalar HEIGHT_COG = RealScalar.of(0.54); // height of COG [m]
  private static final Scalar LW = RealScalar.of(0.8375); // lateral distance of wheels from COG [m]
  private static final Scalar LF = RealScalar.of(1.015); // front axle distance from COG [m]
  private static final Scalar LR = RealScalar.of(1.895); // rear axle distance from COG [m]
  private final Tensor levers;
  public CarSteering carSteering = CarSteering.FRONT;

  public CHatchbackModel() {
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
    return RealScalar.of(1412); // mass [kg]
  }

  @Override
  public Pacejka3 pacejka1() {
    return PACEJKA1;
  }

  @Override
  public Pacejka3 pacejka2() {
    return PACEJKA2;
  }

  @Override
  public Scalar radius() {
    return RADIUS;
  }

  @Override
  public Scalar heightCog() {
    return HEIGHT_COG;
  }

  @Override
  public Scalar mu() {
    return RealScalar.of(0.85); // tire road friction coefficient
  }

  @Override
  public Tensor levers() {
    return levers;
  }

  @Override
  public Scalar frontL() {
    return RealScalar.of(1.915); // distance from COG to front end [m]
  }

  @Override
  public Scalar rearL() {
    return RealScalar.of(2.835); // distance from COG to rear end [m]
  }

  @Override
  public Scalar width() {
    return RealScalar.of(1.916); // width of the vehicle [m]
  }

  @Override
  public Scalar gammaM() {
    return RealScalar.of(0.5); // rear/total drive ratio; 0 is FWD, 1 is RWD
  }

  @Override
  public Scalar Iz_invert() {
    return RealScalar.of(1 / (1536.7 + 427.7084)); // yawing moment of inertia [kgm2]
  }

  @Override
  public Scalar Iw_invert() {
    return RealScalar.of(1 / 0.9); // wheel moment of inertia [kgm2]
  }

  @Override
  public Scalar b() {
    return RealScalar.of(5); // dynamic friction coefficient N/(m/s)
  }

  @Override
  public Scalar fric() {
    return RealScalar.of(47); // coulomb friction
  }

  @Override
  public CarSteering steering() {
    return carSteering;
  }

  // maximal steering angle [deg]
  // TODO check online what is appropriate
  private static final Scalar maxDelta = RealScalar.of(45 * Math.PI / 180);
  // maximal motor torque [Nm], with gears included
  private static final Scalar maxPress = RealScalar.of(13); // maximal master cylinder presure [MPa]
  private static final Scalar maxThb = RealScalar.of(2000); // max handbrake torque [Nm]
  private static final Scalar maxThrottle = RealScalar.of(2000.);

  @Override
  public CarControl createControl(Tensor u) {
    if (!Clip.ABSOLUTE_ONE.of(u.Get(0)).equals(u.Get(0)))
      throw TensorRuntimeException.of(u.Get(0));
    if (!Clip.UNIT.of(u.Get(3)).equals(u.Get(3)))
      throw TensorRuntimeException.of(u.Get(3));
    // ---
    Scalar maxSteer = //
        carSteering.equals(CarSteering.BOTH) ? maxDelta.multiply(RealScalar.of(.5)) : maxDelta;
    Scalar delta = u.Get(0).multiply(maxSteer);
    Scalar brake = u.Get(1).multiply(maxPress);
    Scalar handbrake = u.Get(2).multiply(maxThb);
    Scalar throttle = u.Get(3).multiply(maxThrottle);
    return new CarControl(Tensors.of(delta, brake, handbrake, throttle));
  }

  @Override
  public Scalar press2torF() {
    return RealScalar.of(250); // Nm per Mpa conversion constant [Nm/Mpa] for Front and Rear brakes
  }

  @Override
  public Scalar press2torR() {
    return RealScalar.of(150);
  }

  @Override
  public Scalar muRoll() {
    // TODO check if == 0 ok
    // for ==2 the car will not make a turn but slide in nose direction...
    return RealScalar.of(0); // rolling friction coefficient
  }

  @Override
  public Scalar rollFric() {
    return gForce().multiply(muRoll());
  }

  // public static final Scalar eps = RealScalar.of(1e-4); // tolerance below which is speed considered 0
  // public static final Scalar Dz1 = RealScalar.of(0.05); // dead zone tOLERANCE
  // public static final Scalar Dz2 = RealScalar.of(3.1415 / 180);
  // public static final Scalar T = RealScalar.of(0.1);
  // public static final Scalar maxDeltaRate = RealScalar.of(50 * Math.PI / 180); // rad/s
  private static final Scalar maxBrakeRate = RealScalar.of(5); // 1/s
  private static final Scalar maxHandbrakeRate = RealScalar.of(5); // 1/s
  private static final Scalar maxThrottleRate = RealScalar.of(5); // 1/s
}
