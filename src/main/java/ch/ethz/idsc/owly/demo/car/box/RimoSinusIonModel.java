// code by jph
package ch.ethz.idsc.owly.demo.car.box;

import java.util.ArrayList;
import java.util.List;

import ch.ethz.idsc.owly.demo.car.CarControl;
import ch.ethz.idsc.owly.demo.car.CarSteering;
import ch.ethz.idsc.owly.demo.car.DefaultCarModel;
import ch.ethz.idsc.owly.demo.car.DefaultTire;
import ch.ethz.idsc.owly.demo.car.TireInterface;
import ch.ethz.idsc.owly.math.car.Pacejka3;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.TensorRuntimeException;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Clip;

/** specifications of vehicle taken from:
 * http://www.rimo-germany.com/technische-daten-sinus-ion.html
 * 
 * 
 * L/B/H: 2020 / 1390 / 600-1200\
 * 186 kg
 * Motoren: 2 x 2.8 kW Motoren
 * 
 * Drehmoment: 95 Nm pro Rad
 * 
 * specifications of tires taken from:
 * http://www.prespo.de/shop/reifen/satz-dunlop-sl1-dimension-450710.html
 * 
 * Tires: DUNLOP SL1
 * Tires front:
 * Laufflächenbreite: 86mm
 * Gesamtbreite: 133mm
 * Außendurchmesser: 255mm
 * Felgengrösse (inch): 4.5 (125-130mm)
 * 
 * Tires rear:
 * Laufflächenbreite: 136mm
 * Gesamtbreite: 210mm
 * Außendurchmesser: 280mm
 * Felgengrösse (inch): 8.0 (210mm) */
public class RimoSinusIonModel extends DefaultCarModel {
  public static RimoSinusIonModel standard() {
    return new RimoSinusIonModel(CarSteering.FRONT);
  }

  // ---
  private final CarSteering carSteering;
  private final List<TireInterface> list = new ArrayList<>();

  /** @param carSteering
   * @param gammaM rear/total drive ratio; 0 is FWD, 1 is RWD, 0.5 is AWD */
  public RimoSinusIonModel(CarSteering carSteering) {
    this.carSteering = carSteering;
    final Pacejka3 PACEJKA = new Pacejka3(7, 1.4); //
    final Scalar RADIUS1 = DoubleScalar.of(0.255 * 0.5); // wheel radius [m]
    final Scalar RADIUS2 = DoubleScalar.of(0.280 * 0.5); // wheel radius [m]
    final Scalar IW = DoubleScalar.of(1); // wheel inertia [kgm2]
    // TODO dimensions are not accurate
    final Scalar LZ = DoubleScalar.of(-0.25); // height of COG [m]
    final Scalar LF = DoubleScalar.of(+0.645); // front axle distance from COG [m]
    final Scalar LR = DoubleScalar.of(-0.4); // rear axle distance from COG [m]
    final Scalar TF = DoubleScalar.of(1.055 / 2); // half front track
    final Scalar TR = DoubleScalar.of(1.200 / 2); // half rear track
    list.add(new DefaultTire(RADIUS1, IW, PACEJKA, Tensors.of(LF, TF, LZ)));
    list.add(new DefaultTire(RADIUS1, IW, PACEJKA, Tensors.of(LF, TF.negate(), LZ)));
    list.add(new DefaultTire(RADIUS2, IW, PACEJKA, Tensors.of(LR, TR, LZ)));
    list.add(new DefaultTire(RADIUS2, IW, PACEJKA, Tensors.of(LR, TR.negate(), LZ)));
  }

  // ---
  @Override
  public Scalar mass() {
    return DoubleScalar.of(186); // mass [kg]
  }

  @Override
  public Scalar gammaM() {
    return DoubleScalar.of(1); // rear/total drive ratio; 0 is FWD, 1 is RWD
  }

  @Override
  public Scalar Iz_invert() {
    return DoubleScalar.of(1 / 20.0); // yawing moment of inertia [kgm2]
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
  private static final Scalar maxPress = DoubleScalar.of(13.0); // TODO
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
}
