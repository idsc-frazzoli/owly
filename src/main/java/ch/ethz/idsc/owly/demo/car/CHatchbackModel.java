// code by edo
// code adapted by jph
package ch.ethz.idsc.owly.demo.car;

import java.util.ArrayList;
import java.util.List;

import ch.ethz.idsc.owly.math.car.Pacejka3;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.TensorRuntimeException;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Clip;

public class CHatchbackModel extends DefaultCarModel {
  public static CHatchbackModel standard() {
    return new CHatchbackModel(CarSteering.FRONT, RealScalar.ZERO);
  }

  // ---
  private final Tensor levers;
  private final CarSteering carSteering;
  private final Scalar gammaM;
  private final List<TireInterface> list = new ArrayList<>();

  /** @param carSteering
   * @param gammaM rear/total drive ratio; 0 is FWD, 1 is RWD, 0.5 is AWD */
  public CHatchbackModel(CarSteering carSteering, Scalar gammaM) {
    this.carSteering = carSteering;
    this.gammaM = gammaM;
    final Scalar LW = DoubleScalar.of(0.8375); // lateral distance of wheels from COG [m]
    final Scalar LF = DoubleScalar.of(1.015); // front axle distance from COG [m]
    final Scalar LR = DoubleScalar.of(1.895); // rear axle distance from COG [m]
    Scalar h_negate = DoubleScalar.of(-0.54); // height of COG [m]
    levers = Tensors.of( //
        Tensors.of(LF, LW, h_negate), // 1L
        Tensors.of(LF, LW.negate(), h_negate), // 1R
        Tensors.of(LR.negate(), LW, h_negate), // 2L
        Tensors.of(LR.negate(), LW.negate(), h_negate) // 2R
    ).unmodifiable();
    Scalar RADIUS = DoubleScalar.of(0.325); // wheel radius [m]
    final Pacejka3 PACEJKA1 = new Pacejka3(13.8509, 1.3670, 0.9622);
    final Pacejka3 PACEJKA2 = new Pacejka3(14.1663, 1.3652, 0.9744);
    list.add(new DefaultTire(levers.get(0), RADIUS, DoubleScalar.of(1536.7 + 427.7084), PACEJKA1));
    list.add(new DefaultTire(levers.get(1), RADIUS, DoubleScalar.of(1536.7 + 427.7084), PACEJKA1));
    list.add(new DefaultTire(levers.get(2), RADIUS, DoubleScalar.of(1536.7 + 427.7084), PACEJKA2));
    list.add(new DefaultTire(levers.get(3), RADIUS, DoubleScalar.of(1536.7 + 427.7084), PACEJKA2));
  }

  public TireInterface tire(int index) {
    return list.get(index);
  }

  // ---
  @Override
  public Scalar mass() {
    return DoubleScalar.of(1412); // mass [kg]
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
    return DoubleScalar.of(1 / (1536.7 + 427.7084)); // yawing moment of inertia [kgm2]
  }

  @Override
  public Scalar Iw_invert() {
    return DoubleScalar.of(1 / 0.9); // TODO check wheel moment of inertia [kgm2]
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

  // maximal steering angle [deg]
  // TODO check online what is appropriate
  private static final Scalar maxDelta = DoubleScalar.of(45 * Math.PI / 180);
  // maximal motor torque [Nm], with gears included
  private static final Scalar maxPress = DoubleScalar.of(13); // maximal master cylinder presure [MPa]
  private static final Scalar maxThb = DoubleScalar.of(2000); // max handbrake torque [Nm]
  private static final Scalar maxThrottle = DoubleScalar.of(2000.);

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
  // public static final Scalar Dz1 = RealScalar.of(0.05); // dead zone tOLERANCE
  // public static final Scalar Dz2 = RealScalar.of(3.1415 / 180);
  // public static final Scalar T = RealScalar.of(0.1);
}
