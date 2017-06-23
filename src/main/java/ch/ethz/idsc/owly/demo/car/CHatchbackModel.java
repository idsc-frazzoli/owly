// code by edo
// code adapted by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.owly.math.Pacejka3;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.sca.Sign;

public class CHatchbackModel extends DefaultCarModel {
  @Override
  public Scalar mass() {
    return RealScalar.of(1412); // mass [kg]
  }

  public static final Scalar Iz = RealScalar.of(1536.7 + 427.7084); // yawing moment of inertia [kgm2]
  public static final Scalar Iw = RealScalar.of(0.9); // wheel moment of inertia [kgm2]
  // dimensions
  public static final Scalar frontL = RealScalar.of(1.915); // distance from COG to front end [m]
  public static final Scalar rearL = RealScalar.of(2.835); // distance from COG to rear end [m]
  public static final Scalar width = RealScalar.of(1.916); // width of the vehicle [m]
  public static final Scalar lF = RealScalar.of(1.015); // front axle distance from COG [m]
  public static final Scalar lR = RealScalar.of(1.895); // rear axle distanc from COG [m]
  public static final Scalar lw = RealScalar.of(1.675 / 2); // lateral distance of wheels from COG [m]
  public static final Scalar h = RealScalar.of(0.54); // height of COG [m]
  public static final Scalar R = RealScalar.of(0.325); // wheel radius [m]
  // pacejka model parameters
  public final Pacejka3 pacejka1 = new Pacejka3(13.8509, 1.3670, 0.9622);
  public final Pacejka3 pacejka2 = new Pacejka3(14.1663, 1.3652, 0.9744);
  public static final Scalar maxDeltaDEGREE = RealScalar.of(30); // maximal steering angle [deg]
  public static final Scalar press2torF = RealScalar.of(250); // Nm per Mpa conversion constant [Nm/Mpa] for Front and Rear brakes
  public static final Scalar press2torR = RealScalar.of(150);
  public static final Scalar maxThb = RealScalar.of(2000); // max handbrake torque [Nm]
  public static final Scalar maxPress = RealScalar.of(13); // maximal master cylinder presure [MPa]
  public static final Scalar maxTm = RealScalar.of(1000); // maximal motor torque [Nm], with gears included
  public static final Scalar gammaM = RealScalar.of(0); // rear/total drive ratio; 0 is FWD, 1 is RWD
  public static final Scalar g = RealScalar.of(9.81);
  public static final Scalar mu = RealScalar.of(0.85); // tire road friction coefficient
  public static final Scalar muRoll = RealScalar.of(0); // rolling friction coefficient
  public static final Scalar b = RealScalar.of(5); // dynamic friction coefficient N/(m/s)
  public static final Scalar fric = RealScalar.of(47); // coulomb friction
  public static final Scalar eps = RealScalar.of(1e-4); // tolerance below which is speed considered 0
  public static final Scalar Dz1 = RealScalar.of(0.05); // dead zone tOLERANCE
  public static final Scalar Dz2 = RealScalar.of(3.1415 / 180);
  // public static final Scalar T = RealScalar.of(0.1);
  public static final Scalar maxDeltaRate = RealScalar.of(50 * Math.PI / 180); // rad/s
  public static final Scalar maxBrakeRate = RealScalar.of(5); // 1/s
  public static final Scalar maxHandbrakeRate = RealScalar.of(5); // 1/s
  public static final Scalar maxThrottleRate = RealScalar.of(5); // 1/s

  public Scalar rollFric() {
    return gForce().multiply(muRoll);
  }

  public Scalar coulombFriction(Scalar in) {
    return Sign.of(in).multiply(b.multiply(in.abs()).add(fric));
  }
}
