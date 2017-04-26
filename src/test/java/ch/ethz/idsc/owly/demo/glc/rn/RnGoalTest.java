// code by jph
package ch.ethz.idsc.owly.demo.glc.rn;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.ZeroScalar;
import junit.framework.TestCase;

public class RnGoalTest extends TestCase {
  public void testSimple() {
    RnGoal rnGoal = new RnGoal(Tensors.vector(5, 0), RealScalar.of(2));
    assertEquals(rnGoal.costToGo(Tensors.vector(2, 0)), RealScalar.ONE);
    assertEquals(rnGoal.costToGo(Tensors.vector(3, 0)), ZeroScalar.get());
    assertEquals(rnGoal.costToGo(Tensors.vector(4, 0)), ZeroScalar.get());
  }
}
