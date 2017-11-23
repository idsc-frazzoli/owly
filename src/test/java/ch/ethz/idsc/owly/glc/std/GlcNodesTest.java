// code by jph
package ch.ethz.idsc.owly.glc.std;

import java.util.List;

import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owly.glc.adapter.GlcNodes;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class GlcNodesTest extends TestCase {
  public void testRoot() {
    GlcNode root = GlcNode.of(null, new StateTime(Tensors.empty(), RealScalar.ZERO), //
        RealScalar.ZERO, RealScalar.ZERO);
    List<StateTime> list = GlcNodes.getPathFromRootTo(root);
    assertEquals(list.size(), 1);
  }

  public void testSimple() {
    try {
      GlcNodes.getPathFromRootTo(null);
      assertTrue(false);
    } catch (Exception exception) {
      // ---
    }
  }
}
