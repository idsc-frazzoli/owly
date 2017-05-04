// code by jph
package ch.ethz.idsc.owly.math.region;

import java.util.Arrays;

import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class FreeBoundedIntervalRegionTest extends TestCase {
  public void testSimple() {
    FreeBoundedIntervalRegion bir = new FreeBoundedIntervalRegion(0, RealScalar.of(10), RealScalar.of(20));
    assertEquals(bir.evaluate(Tensors.vector(+5)), RealScalar.of(-5));
    assertEquals(bir.evaluate(Tensors.vector(10)), RealScalar.of(+0));
    assertEquals(bir.evaluate(Tensors.vector(15)), RealScalar.of(+5));
    assertEquals(bir.evaluate(Tensors.vector(20)), RealScalar.of(+0));
    assertEquals(bir.evaluate(Tensors.vector(25)), RealScalar.of(-5));
  }

  public void testTrajectory() {
    FreeBoundedIntervalRegion bir = new FreeBoundedIntervalRegion(0, RealScalar.of(10), RealScalar.of(20));
    TrajectoryRegionQuery trq = new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(bir));
    assertTrue(trq.isDisjoint(Arrays.asList(new StateTime(Tensors.vector(15), null))));
    assertFalse(trq.isDisjoint(Arrays.asList(new StateTime(Tensors.vector(5), null))));
  }
}
