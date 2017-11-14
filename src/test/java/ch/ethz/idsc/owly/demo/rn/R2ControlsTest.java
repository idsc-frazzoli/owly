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
    R2Config r2Config = new R2Config(RealScalar.ONE);
    Collection<Flow> flows = r2Config.createRadial(n);
    assertEquals(flows.size(), n);
    Tensor tflow = Tensor.of(flows.stream().map(Flow::getU));
    Tensor hul = ConvexHull.of(tflow);
    assertEquals(Dimensions.of(tflow), Dimensions.of(hul));
  }

  public void testMaxSpeed() {
    int n = 10;
    R2Config r2Config = new R2Config(RealScalar.ONE);
    Collection<Flow> controls = r2Config.createRadial(n);
    Scalar maxSpeed = RnControls.maxSpeed(controls);
    assertTrue(Chop._14.close(maxSpeed, RealScalar.ONE));
  }

  public void testFail() {
    R2Config r2Config = new R2Config(RealScalar.ONE);
    try {
      r2Config.createRadial(2);
      assertTrue(false);
    } catch (Exception exception) {
      // ---
    }
  }
}
