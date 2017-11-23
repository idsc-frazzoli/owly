// code by jph
package ch.ethz.idsc.owly.demo.rn;

import java.util.Collection;

import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.qty.Quantity;
import ch.ethz.idsc.tensor.sca.Chop;
import junit.framework.TestCase;

public class RnMinTimeGoalManagerTest extends TestCase {
  public void testSimple() {
    R2Flows r2Config = new R2Flows(Quantity.of(2, "m*s^-1"));
    Collection<Flow> controls = r2Config.getFlows(10);
    Tensor center = Tensors.fromString("{3[m],6[m]}");
    Scalar radius = Quantity.of(1, "m");
    GoalInterface goalInterface = RnMinTimeGoalManager.create(center, radius, controls);
    // Scalar cost = ;
    assertEquals(goalInterface.minCostToGoal(Tensors.fromString("{3[m],6[m]}")), Quantity.of(0, "s"));
    assertEquals(goalInterface.minCostToGoal(Tensors.fromString("{2[m],6[m]}")), Quantity.of(0, "s"));
    assertTrue(Chop._14.close(goalInterface.minCostToGoal( //
        Tensors.fromString("{0[m],6[m]}")), Quantity.of(1, "s")));
  }
}
