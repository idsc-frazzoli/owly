// code by jph
package ch.ethz.idsc.owly.demo.util;

import java.util.Collection;

import ch.ethz.idsc.owly.demo.rn.R2Controls;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.alg.Dimensions;
import ch.ethz.idsc.tensor.opt.ConvexHull;
import junit.framework.TestCase;

public class R2ControlsTest extends TestCase {
  public void testSimple() {
    int n = 100;
    Collection<Flow> flows = R2Controls.createRadial(n);
    assertEquals(flows.size(), n);
    Tensor tflow = Tensor.of(flows.stream().map(Flow::getU));
    Tensor hul = ConvexHull.of(tflow);
    assertEquals(Dimensions.of(tflow), Dimensions.of(hul));
  }
}
