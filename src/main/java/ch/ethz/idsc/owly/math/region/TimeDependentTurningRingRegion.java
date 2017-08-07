// code by jph
package ch.ethz.idsc.owly.math.region;

import ch.ethz.idsc.owly.math.Se2Utils;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.StateTimeRegion;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.ArcCos;
import ch.ethz.idsc.tensor.sca.Mod;

/** evaluate does not correspond to Euclidean distance */
public class TimeDependentTurningRingRegion implements StateTimeRegion {
  private static Mod MOD = Mod.function(RealScalar.of(2 * Math.PI), RealScalar.of(-Math.PI));
  private final Tensor center;
  private final Scalar initialGapAngle;
  private final Scalar gapSizeAngle;
  private final Scalar lowerRingRadius;
  private final Scalar upperRingRadius;
  private final Scalar turningSpeed = Se2Utils.DEGREE(30); // 30 °/s

  public TimeDependentTurningRingRegion(Tensor center, Scalar initialGapAngle, Scalar gapLength, Scalar ringThickness, Scalar ringRadius) {
    this.center = center;
    this.initialGapAngle = initialGapAngle;
    this.gapSizeAngle = gapLength;
    this.lowerRingRadius = ringRadius.subtract(ringThickness.divide(RealScalar.of(2)));
    this.upperRingRadius = ringRadius.add(ringThickness.divide(RealScalar.of(2)));
  }

  @Override
  public boolean isMember(StateTime stateTime) {
    System.out.println("");
    System.err.println("count");
    Scalar time = stateTime.time();
    Tensor state = stateTime.state().extract(0, 2);
    // flatten(-1) right?
    if (!stateTime.state().flatten(0).filter(s -> s.equals(time)).findAny().isPresent())
      throw new RuntimeException(); // None of the States represents the Time
    if (state.length() != 2)
      throw new RuntimeException();
    Scalar radius = Norm._2.of(state.subtract(center));
    if (Scalars.lessEquals(lowerRingRadius, radius) && Scalars.lessEquals(radius, upperRingRadius)) { // in Obstacle radial
      Scalar upperGapAngle = initialGapAngle.add(gapSizeAngle.divide(RealScalar.of(2)));
      Scalar lowerGapAngle = initialGapAngle.subtract(gapSizeAngle.divide(RealScalar.of(2)));
      Tensor vec1 = state.subtract(center);
      Tensor vec2 = Tensors.vector(1, 0);
      Tensor angle = ArcCos.of((vec1.dot(vec2)).divide(Norm._2.of(vec1)).divide(Norm._2.of(vec2))).subtract(turningSpeed.multiply(time));
      System.out.println("Angle" + angle);
      System.out.println("Upper_" + MOD.of(upperGapAngle));
      System.out.println("lower_" + MOD.of(lowerGapAngle));
      if (!angle.isScalar())
        throw new RuntimeException();
      // TODO JONAS: does not work yet, as arccos gives me value between Pi and -Pi
      if (Scalars.lessEquals((Scalar) MOD.of(angle), upperGapAngle)//
          && Scalars.lessEquals(lowerGapAngle, (Scalar) MOD.of(angle))) {
        System.out.println("In GAP");
        return false; // in Obstacle by angle
      } else {
        return true;
      }
    }
    return false;
  }
}
