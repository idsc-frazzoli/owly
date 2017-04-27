// code by jph
package ch.ethz.idsc.owly.demo.glc.rn;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.ZeroScalar;
import junit.framework.TestCase;

public class RnGoalManagerTest extends TestCase {
  public void testSimple() {
    RnGoalManager rnGoal = new RnGoalManager(Tensors.vector(5, 0), RealScalar.of(2));
    assertEquals(rnGoal.costToGoal(Tensors.vector(2, 0)), RealScalar.ONE);
    assertEquals(rnGoal.costToGoal(Tensors.vector(3, 0)), ZeroScalar.get());
    assertEquals(rnGoal.costToGoal(Tensors.vector(4, 0)), ZeroScalar.get());
  }
}
