// code by jph
package ch.ethz.idsc.owly.demo.twd;

import ch.ethz.idsc.owly.demo.twd.glc.TwdMinTimeGoalManager;
import ch.ethz.idsc.owly.glc.adapter.HeuristicQ;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class TwdMinTimeGoalManagerTest extends TestCase {
  public void testSimple() {
    TwdMinTimeGoalManager manager = //
        new TwdMinTimeGoalManager(Tensors.vector(10, 0, Math.PI), RealScalar.ONE, RealScalar.ONE);
    assertTrue(HeuristicQ.of(manager.getGoalInterface()));
    Scalar cost = manager.minCostToGoal(Tensors.vector(0, 0, 0));
    assertTrue(Scalars.lessEquals(RealScalar.of(9), cost));
    assertTrue(manager.isMember(Tensors.vector(10, 0, Math.PI + 0.9)));
    assertFalse(manager.isMember(Tensors.vector(10, 0, Math.PI + 1.1)));
    assertTrue(manager.isMember(Tensors.vector(10, 0, Math.PI + 2 * Math.PI + 0.9)));
    assertFalse(manager.isMember(Tensors.vector(10, 0, Math.PI + 2 * Math.PI + 1.1)));
  }

  public void testAllAngles() {
    TwdMinTimeGoalManager manager = //
        new TwdMinTimeGoalManager(Tensors.vector(0, 0, Math.PI), RealScalar.ONE, RealScalar.of(Math.PI));
    for (int index = -100; index < 100; ++index)
      assertTrue(manager.isMember(Tensors.vector(0, 0, index)));
  }
}
