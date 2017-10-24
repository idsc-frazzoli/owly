// code by jph
package ch.ethz.idsc.owly.demo.rn;

import java.util.Collection;

import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.alg.Dimensions;
import ch.ethz.idsc.tensor.opt.ConvexHull;
import ch.ethz.idsc.tensor.sca.Chop;
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

  public void testMaxSpeed() {
    int n = 10;
    Collection<Flow> controls = R2Controls.createRadial(n);
    Scalar maxSpeed = R2Controls.maxSpeed(controls);
    assertTrue(Chop._14.close(maxSpeed, RealScalar.ONE));
  }

  public void testFail() {
    try {
      R2Controls.createRadial(2);
      assertTrue(false);
    } catch (Exception exception) {
      // ---
    }
  }
}
