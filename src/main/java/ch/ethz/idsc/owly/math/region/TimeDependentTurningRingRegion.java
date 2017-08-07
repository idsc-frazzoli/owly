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

  /** Constructs a Ring, with a gap in it, which turns at 30°/s CCW
   * 
   * @param center
   * @param initialGapAngle: inital positon where Gap should be
   * @param gapSizeAngle: size of Gap in rad
   * @param ringThickness: thickness of the obstacleRing
   * @param ringRadius: Radius of the Ring (to the middle) */
  public TimeDependentTurningRingRegion(Tensor center, Scalar initialGapAngle, Scalar gapSizeAngle, Scalar ringThickness, Scalar ringRadius) {
    this.center = center;
    this.initialGapAngle = initialGapAngle;
    this.gapSizeAngle = gapSizeAngle;
    this.lowerRingRadius = ringRadius.subtract(ringThickness.divide(RealScalar.of(2)));
    this.upperRingRadius = ringRadius.add(ringThickness.divide(RealScalar.of(2)));
  }

  @Override
  public boolean isMember(StateTime stateTime) {
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
      Tensor angleTensor = ArcCos.of((vec1.dot(vec2)).divide(Norm._2.of(vec1)).divide(Norm._2.of(vec2)));
      if (!angleTensor.isScalar())
        throw new RuntimeException();
      Scalar angle = (Scalar) angleTensor;
      if (Scalars.lessThan(vec1.Get(1), RealScalar.ZERO)) { // if state is in lower half : negative Angle
        angle = angle.negate();
      }
      angle = angle.subtract(turningSpeed.multiply(time));
      if (Scalars.lessEquals(MOD.of(angle), upperGapAngle) && Scalars.lessEquals(lowerGapAngle, MOD.of(angle)))
        return false; // checks if in GAP
      return true; // Otherwise in Ring
    }
    return false;
  }
}
