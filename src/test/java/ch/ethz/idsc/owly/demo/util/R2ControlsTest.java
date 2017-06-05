// code by jph
package ch.ethz.idsc.owly.demo.util;

import java.util.Collection;

import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.Tensor;
import junit.framework.TestCase;

public class R2ControlsTest extends TestCase {
  public void testSimple() {
    Collection<Flow> flows = R2Controls.createRadial(3);
    assertEquals(flows.size(), 3);
    Tensor tflow = Tensor.of(flows.stream().map(Flow::getU));
    tflow.copy();
    // TODO test flows in general
  }
}
