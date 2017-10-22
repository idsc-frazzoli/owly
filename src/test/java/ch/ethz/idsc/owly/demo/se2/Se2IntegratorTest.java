// code by jph
package ch.ethz.idsc.owly.demo.se2;

import ch.ethz.idsc.owly.math.StateSpaceModels;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.flow.RungeKutta45Integrator;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Chop;
import junit.framework.TestCase;

public class Se2IntegratorTest extends TestCase {
  public void testStraight() {
    Tensor x = Tensors.vector(-1, -2, 1);
    Tensor u = Tensors.vector(0, 1);
    Scalar h = RealScalar.of(2);
    Flow flow = StateSpaceModels.createFlow(Se2StateSpaceModel.INSTANCE, u);
    Se2StateSpaceModel.INSTANCE.f(x, u);
    Tensor expl = Se2Integrator.INSTANCE.step(flow, x, h);
    Tensor impl = RungeKutta45Integrator.INSTANCE.step(flow, x, h);
    assertTrue(Chop._10.close(impl, expl));
  }

  public void testRotate1() {
    Tensor x = Tensors.vector(-1, -2, 1);
    Tensor u = Tensors.vector(1, 1);
    Scalar h = RealScalar.of(.25);
    Flow flow = StateSpaceModels.createFlow(Se2StateSpaceModel.INSTANCE, u);
    Se2StateSpaceModel.INSTANCE.f(x, u);
    Tensor expl = Se2Integrator.INSTANCE.step(flow, x, h);
    Tensor impl = RungeKutta45Integrator.INSTANCE.step(flow, x, h);
    assertTrue(Chop._10.close(impl, expl));
  }

  public void testRotate2() {
    Tensor x = Tensors.vector(-1, -2, 1);
    Tensor u = Tensors.vector(2, 1);
    Scalar h = RealScalar.of(.25);
    Flow flow = StateSpaceModels.createFlow(Se2StateSpaceModel.INSTANCE, u);
    Se2StateSpaceModel.INSTANCE.f(x, u);
    Tensor expl = Se2Integrator.INSTANCE.step(flow, x, h);
    Tensor impl = RungeKutta45Integrator.INSTANCE.step(flow, x, h);
    assertTrue(Chop._07.close(impl, expl));
  }
}
