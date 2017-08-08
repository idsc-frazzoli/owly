// code by jph
package ch.ethz.idsc.owly.math.region;

import ch.ethz.idsc.owly.math.Se2Utils;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class TimeDependentTurningRingRegionTest extends TestCase {
  public void testSimple() {
    Tensor center = Tensors.vector(0, 0);
    Scalar initialGapAngle = Se2Utils.DEGREE(0);
    Scalar gapLength = Se2Utils.DEGREE(40);
    Scalar ringThickness = RealScalar.of(0.4);
    Scalar ringRadius = RealScalar.ONE;
    TimeDependentTurningRingRegion test = new TimeDependentTurningRingRegion(center, initialGapAngle, gapLength, ringThickness, ringRadius);
    assertFalse(test.isMember(new StateTime(Tensors.vector(0.5, 0, 0), RealScalar.of(0)))); // inside
    assertFalse(test.isMember(new StateTime(Tensors.vector(0, 0.5, 0), RealScalar.of(0)))); // inside
    assertFalse(test.isMember(new StateTime(Tensors.vector(0, -1.5, 0), RealScalar.of(0)))); // outside
    assertFalse(test.isMember(new StateTime(Tensors.vector(0, 1.5, 0), RealScalar.of(0)))); // outside
    
    assertFalse(test.isMember(new StateTime(Tensors.vector(1, 0, 0), RealScalar.of(0)))); // in gap
    assertTrue(test.isMember(new StateTime(Tensors.vector(1, 0, 3), RealScalar.of(3)))); // 3s = 90° later
    // --
    assertTrue(test.isMember(new StateTime(Tensors.vector(0, 1, 0), RealScalar.of(0)))); // North
    assertFalse(test.isMember(new StateTime(Tensors.vector(0, 1, 3), RealScalar.of(3)))); // 3s = 90° later in gap at North
    assertTrue(test.isMember(new StateTime(Tensors.vector(1, 0, 3), RealScalar.of(3))));
    // --
    assertTrue(test.isMember(new StateTime(Tensors.vector(-1, 0, 0), RealScalar.of(0)))); // West
    assertFalse(test.isMember(new StateTime(Tensors.vector(-1, 0, 6), RealScalar.of(6)))); // 6s = 180° later in gap
    assertTrue(test.isMember(new StateTime(Tensors.vector(1, 0, 6), RealScalar.of(6))));
    // --
    assertTrue(test.isMember(new StateTime(Tensors.vector(0, -1, 0), RealScalar.of(0)))); // South
    assertFalse(test.isMember(new StateTime(Tensors.vector(0, -1, 9), RealScalar.of(9))));
    assertTrue(test.isMember(new StateTime(Tensors.vector(1, 0, 9), RealScalar.of(9))));// 9s=270° later in gap
    // --
  }
}
