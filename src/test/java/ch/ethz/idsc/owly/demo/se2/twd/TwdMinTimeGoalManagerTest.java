// code by jph
package ch.ethz.idsc.owly.demo.se2.twd;

import java.util.Collection;

import ch.ethz.idsc.owly.demo.se2.Se2MinTimeGoalManager;
import ch.ethz.idsc.owly.glc.adapter.HeuristicQ;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class TwdMinTimeGoalManagerTest extends TestCase {
  public void testSimple() {
    TwdConfig twdConfig = new TwdConfig(RealScalar.of(1), RealScalar.of(1));
    Collection<Flow> controls = twdConfig.createControls(8);
    Se2MinTimeGoalManager manager = new Se2MinTimeGoalManager( //
        Tensors.vector(10, 0, Math.PI), Tensors.vector(1, 1, 1), controls);
    // TwdMinTimeGoalManager manager = //
    // new TwdMinTimeGoalManager(Tensors.vector(10, 0, Math.PI), RealScalar.ONE, RealScalar.ONE);
    assertTrue(HeuristicQ.of(manager.getGoalInterface()));
    Scalar cost = manager.minCostToGoal(Tensors.vector(0, 0, 0));
    assertTrue(Scalars.lessEquals(RealScalar.of(9), cost));
    assertTrue(manager.isMember(Tensors.vector(10, 0, Math.PI + 0.9)));
    assertFalse(manager.isMember(Tensors.vector(10, 0, Math.PI + 1.1)));
    assertTrue(manager.isMember(Tensors.vector(10, 0, Math.PI + 2 * Math.PI + 0.9)));
    assertFalse(manager.isMember(Tensors.vector(10, 0, Math.PI + 2 * Math.PI + 1.1)));
  }

  public void testAllAngles() {
    // TwdMinTimeGoalManager manager = //
    // new TwdMinTimeGoalManager(Tensors.vector(0, 0, Math.PI), RealScalar.ONE, RealScalar.of(Math.PI));
    TwdConfig twdConfig = new TwdConfig(RealScalar.of(1), RealScalar.of(1));
    Collection<Flow> controls = twdConfig.createControls(8);
    Se2MinTimeGoalManager manager = new Se2MinTimeGoalManager( //
        Tensors.vector(0, 0, Math.PI), Tensors.vector(1, 1, Math.PI), controls);
    for (int index = -100; index < 100; ++index)
      assertTrue(manager.isMember(Tensors.vector(0, 0, index)));
  }
}
