// code by jph
package ch.ethz.idsc.owly.demo.delta;

import java.util.Collection;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.io.ResourceData;
import ch.ethz.idsc.tensor.sca.Chop;
import ch.ethz.idsc.tensor.sca.Sign;
import junit.framework.TestCase;

public class DeltaControlsTest extends TestCase {
  public void testSimple() {
    ImageGradient imageGradient = //
        new ImageGradient(ResourceData.of("/io/delta_uxy.png"), Tensors.vector(10, 10), RealScalar.of(.1));
    Scalar maxNormGradient = imageGradient.maxNormGradient();
    // System.out.println(maxNormGradient);
    assertTrue(Sign.isPositive(maxNormGradient));
    StateSpaceModel stateSpaceModel = new DeltaStateSpaceModel(imageGradient, null);
    Scalar amp = RealScalar.of(2);
    Collection<Flow> controls = DeltaControls.createControls(stateSpaceModel, amp, 20);
    Scalar max = DeltaControls.maxSpeed(controls);
    assertTrue(Chop._12.close(max, amp));
  }
}
