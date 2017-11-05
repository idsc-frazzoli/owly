// code by jph
package ch.ethz.idsc.owly.demo.se2;

import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;

import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.GlcNodes;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class Se2MinTimeMinShiftGoalManagerTest extends TestCase {
  public void testSimple() {
    Flow flowF = Se2Controls.singleton(RealScalar.of(+1), RealScalar.of(1));
    Flow flowR = Se2Controls.singleton(RealScalar.of(-1), RealScalar.of(1));
    Collection<Flow> controls = Arrays.asList(flowF, flowR);
    GoalInterface goalInterface = //
        Se2MinTimeMinShiftGoalManager.create(Tensors.vector(10, 0, 0), Tensors.vector(1, 1, 1), controls);
    GlcNode root = GlcNodes.createRoot(new StateTime(Tensors.vector(0, 0, 0), RealScalar.ZERO), goalInterface);
    StateTime cstime = new StateTime(Tensors.vector(1, 0, 0), RealScalar.ONE);
    Scalar cost1 = goalInterface.costIncrement(root, Collections.singletonList(cstime), flowR);
    GlcNode child = GlcNode.of(flowR, cstime, cost1, RealScalar.ZERO);
    root.insertEdgeTo(child);
    {
      Scalar cost = goalInterface.costIncrement(child, Collections.singletonList(cstime), flowF);
      assertEquals(cost, Se2MinTimeMinShiftGoalManager.SHIFT_PENALTY);
    }
    {
      Scalar cost = goalInterface.costIncrement(child, Collections.singletonList(cstime), flowR);
      assertEquals(cost, RealScalar.ZERO);
    }
  }
}
