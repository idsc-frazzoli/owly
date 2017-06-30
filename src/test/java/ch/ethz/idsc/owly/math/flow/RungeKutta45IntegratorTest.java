// code by jph
package ch.ethz.idsc.owly.math.flow;

import ch.ethz.idsc.owly.demo.psu.PsuStateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModels;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class RungeKutta45IntegratorTest extends TestCase {
  public void testNonTrivial() {
    Tensor u = Tensors.of(RationalScalar.of(1, 2));
    Tensor x = Tensors.vector(1, 2);
    Scalar h = RationalScalar.of(1, 3);
    Flow flow = StateSpaceModels.createFlow(PsuStateSpaceModel.INSTANCE, u);
    Tensor res = RungeKutta45Integrator.INSTANCE.step(flow, x, h);
    Tensor eul = EulerIntegrator.INSTANCE.step(flow, x, h);
    assertFalse(res.equals(eul));
  }

  public void testReference() {
    Tensor u = Tensors.of(RationalScalar.of(1, 2));
    Tensor x = Tensors.vector(1, 2);
    Scalar h = RationalScalar.of(1, 3);
    Flow flow = StateSpaceModels.createFlow(PsuStateSpaceModel.INSTANCE, u);
    Tensor res = RungeKutta45Integrator.INSTANCE.step(flow, x, h);
    Tensor ref = RungeKutta45Reference.INSTANCE.step(flow, x, h);
    assertEquals(res, ref);
  }
}
