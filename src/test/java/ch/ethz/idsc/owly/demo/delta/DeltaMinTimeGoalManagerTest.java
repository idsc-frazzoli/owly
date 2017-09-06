// code by jph
package ch.ethz.idsc.owly.demo.delta;

import java.util.Collection;

import ch.ethz.idsc.owly.glc.adapter.HeuristicQ;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.io.ResourceData;
import junit.framework.TestCase;

public class DeltaMinTimeGoalManagerTest extends TestCase {
  public void testSimple() {
    ImageGradient imageGradient = //
        new ImageGradient(ResourceData.of("/io/delta_uxy.png"), Tensors.vector(10, 10), RealScalar.of(.1));
    Scalar maxNormGradient = imageGradient.maxNormGradient();
    assertTrue(Scalars.lessThan(RealScalar.ZERO, maxNormGradient));
    StateSpaceModel stateSpaceModel = new DeltaStateSpaceModel(imageGradient, null);
    Scalar amp = RealScalar.of(2);
    Collection<Flow> controls = DeltaControls.createControls(stateSpaceModel, amp, 20);
    DeltaMinTimeGoalManager dmtgm = //
        new DeltaMinTimeGoalManager(Tensors.vector(1, 1), RealScalar.ONE, controls, maxNormGradient);
    assertTrue(HeuristicQ.of(dmtgm));
  }
}
