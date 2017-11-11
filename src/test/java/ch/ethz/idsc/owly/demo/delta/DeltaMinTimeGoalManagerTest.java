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
import ch.ethz.idsc.tensor.sca.Chop;
import junit.framework.TestCase;

public class DeltaMinTimeGoalManagerTest extends TestCase {
  public void testSimple() {
    ImageGradient imageGradient = //
        new ImageGradient(ResourceData.of("/io/delta_uxy.png"), Tensors.vector(10, 10), RealScalar.of(.1));
    Scalar maxNormGradient = imageGradient.maxNormGradient();
    assertTrue(Scalars.lessThan(RealScalar.ZERO, maxNormGradient));
    Scalar amp = RealScalar.of(2);
    StateSpaceModel stateSpaceModel = new DeltaStateSpaceModel(imageGradient, amp);
    Collection<Flow> controls = DeltaControls.createControls(stateSpaceModel, amp, 20);
    assertTrue(Chop._10.close(DeltaControls.maxSpeed(controls), amp));
    // System.out.println(stateSpaceModel.getLipschitz());
    Scalar maxMove = DeltaControls.maxSpeed(controls).add(imageGradient.maxNormGradient());
    // System.out.println(maxMove);
    assertTrue(Chop._10.close(maxMove, stateSpaceModel.getLipschitz()));
    DeltaMinTimeGoalManager dmtgm = new DeltaMinTimeGoalManager(Tensors.vector(1, 1), RealScalar.ONE, maxMove);
    assertTrue(HeuristicQ.of(dmtgm));
  }
}
