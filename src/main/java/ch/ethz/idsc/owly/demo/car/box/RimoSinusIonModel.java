// code by jph
package ch.ethz.idsc.owly.demo.car.box;

import java.util.ArrayList;
import java.util.List;

import ch.ethz.idsc.owly.demo.car.CarControl;
import ch.ethz.idsc.owly.demo.car.CarSteering;
import ch.ethz.idsc.owly.demo.car.DefaultCarModel;
import ch.ethz.idsc.owly.demo.car.DefaultWheel;
import ch.ethz.idsc.owly.demo.car.MotorTorques;
import ch.ethz.idsc.owly.demo.car.VehicleModel;
import ch.ethz.idsc.owly.demo.car.WheelInterface;
import ch.ethz.idsc.owly.math.car.Pacejka3;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.TensorRuntimeException;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Clip;

/** specifications of vehicle taken from:
 * http://www.rimo-germany.com/technische-daten-sinus-ion.html
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
// TODO the gokart allows to put throttle to the rear wheels separately
public class RimoSinusIonModel extends DefaultCarModel {
  private static final VehicleModel STANDARD = new RimoSinusIonModel();

  public static VehicleModel standard() {
    return STANDARD;
  }

  // ---
  private final List<WheelInterface> list = new ArrayList<>();
  private final Tensor hull;

  private RimoSinusIonModel() {
    final Pacejka3 PACEJKA = new Pacejka3(7, 1.4); //
    final Scalar RADIUS1 = DoubleScalar.of(0.255 * 0.5); // wheel radius [m]
    final Scalar RADIUS2 = DoubleScalar.of(0.280 * 0.5); // wheel radius [m]
    final Scalar IW = DoubleScalar.of(1); // wheel inertia [kgm2]
    final Scalar LZ = DoubleScalar.of(-0.25); // height of COG [m]
    // data-sheet:
    // final Scalar LF = DoubleScalar.of(+0.645); // front axle distance from COG [m]
    // final Scalar LR = DoubleScalar.of(-0.4); // rear axle distance from COG [m]
    // measured:
    final Scalar LF = DoubleScalar.of(+0.72); // front axle distance from COG [m]
    final Scalar LR = DoubleScalar.of(-0.47); // rear axle distance from COG [m]
    // data-sheet:
    // final Scalar TF = DoubleScalar.of(1.055 / 2); // half front track
    // final Scalar TR = DoubleScalar.of(1.200 / 2); // half rear track
    // measured:
    final Scalar TF = DoubleScalar.of(0.935 / 2); // half front track
    final Scalar TR = DoubleScalar.of(1.070 / 2); // half rear track
    final Scalar TWF = RealScalar.of(0.09); // tire width front on ground
    // tire width front total: 13 cm (same as tire rear width on ground)
    final Scalar TWR = RealScalar.of(0.13); // tire width read
    // tire width rear total: 19.5 cm
    list.add(new DefaultWheel(RADIUS1, TWF, IW, PACEJKA, Tensors.of(LF, TF, LZ)));
    list.add(new DefaultWheel(RADIUS1, TWF, IW, PACEJKA, Tensors.of(LF, TF.negate(), LZ)));
    list.add(new DefaultWheel(RADIUS2, TWR, IW, PACEJKA, Tensors.of(LR, TR, LZ)));
    list.add(new DefaultWheel(RADIUS2, TWR, IW, PACEJKA, Tensors.of(LR, TR.negate(), LZ)));
    // cog + front axle to boundary contact 35 [cm] + to front tip 22.5 [cm]
    final Scalar HFX = LF.add(DoubleScalar.of(0.350 + 0.225));
    final Scalar HRX = HFX.subtract(DoubleScalar.of(2.060)); // measured
    final Scalar HFY = DoubleScalar.of(1.45 / 2); // measured
    final Scalar reduce = RealScalar.of(.6);
    hull = Tensors.empty();
    hull.append(Tensors.of(HFX, TF.multiply(reduce), LZ));
    hull.append(Tensors.of(LF, HFY, LZ));
    hull.append(Tensors.of(LR, HFY, LZ));
    hull.append(Tensors.of(HRX, TR, LZ));
    hull.append(Tensors.of(HRX, TR.negate(), LZ));
    hull.append(Tensors.of(LR, HFY.negate(), LZ));
    hull.append(Tensors.of(LF, HFY.negate(), LZ));
    hull.append(Tensors.of(HFX, TF.multiply(reduce).negate(), LZ));
  }

  // ---
  @Override
  public Scalar mass() {
    return DoubleScalar.of(186); // mass [kg]
  }

  @Override
  public WheelInterface wheel(int index) {
    return list.get(index);
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
    return CarSteering.FRONT;
  }

  // at the extreme the tires are inclined as 18 [deg] to 35 [deg]
  private static final Scalar maxDelta = DoubleScalar.of(25 * Math.PI / 180); // maximal steering angle [rad]
  // maximal motor torque [Nm], with gears included
  private static final Scalar maxPress = DoubleScalar.of(4.0); // TODO no info yet
  private static final Scalar maxThb = DoubleScalar.of(1000.0); // max handbrake torque [Nm]
  private static final Scalar maxThrottle = DoubleScalar.of(100.0); // TODO no info yet

  @Override
  public CarControl createControl(Tensor u) {
    if (!Clip.ABSOLUTE_ONE.of(u.Get(0)).equals(u.Get(0)))
      throw TensorRuntimeException.of(u.Get(0));
    if (!Clip.UNIT.of(u.Get(3)).equals(u.Get(3)))
      throw TensorRuntimeException.of(u.Get(3));
    // ---
    Scalar delta = u.Get(0).multiply(maxDelta);
    Scalar brake = u.Get(1).multiply(maxPress);
    Scalar handbrake = u.Get(2).multiply(maxThb);
    Tensor throttleV = MotorTorques.electonicGokart( //
        u.Get(3).multiply(maxThrottle), u.Get(4).multiply(maxThrottle));
    return new CarControl(delta, brake, handbrake, throttleV);
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
  public Tensor footprint() {
    return hull;
  }
}
