// code by jph
package ch.ethz.idsc.owly.math.se2;

import ch.ethz.idsc.owly.demo.se2.Se2StateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModels;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.flow.RungeKutta45Integrator;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Array;
import ch.ethz.idsc.tensor.lie.MatrixExp;
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
    Tensor u = Tensors.vector(2, .5);
    Scalar h = RealScalar.of(.25);
    Flow flow = StateSpaceModels.createFlow(Se2StateSpaceModel.INSTANCE, u);
    Se2StateSpaceModel.INSTANCE.f(x, u);
    Tensor expl = Se2Integrator.INSTANCE.step(flow, x, h);
    Tensor imp1 = RungeKutta45Integrator.INSTANCE.step(flow, x, h);
    assertTrue(Chop._07.close(imp1, expl));
  }

  public void testRotateHN() {
    Tensor x = Tensors.vector(-1, -2, 1);
    Tensor u = Tensors.vector(1.2, .7);
    Scalar h = RealScalar.of(-.25);
    Flow flow = StateSpaceModels.createFlow(Se2StateSpaceModel.INSTANCE, u);
    Se2StateSpaceModel.INSTANCE.f(x, u);
    Tensor expl = Se2Integrator.INSTANCE.step(flow, x, h);
    Tensor impl = RungeKutta45Integrator.INSTANCE.step(flow, x, h);
    assertTrue(Chop._07.close(impl, expl));
  }

  public void testRotateUN() {
    Tensor x = Tensors.vector(-1, -2, 1);
    Tensor u = Tensors.vector(2, -.8);
    Scalar h = RealScalar.of(.25);
    Flow flow = StateSpaceModels.createFlow(Se2StateSpaceModel.INSTANCE, u);
    Se2StateSpaceModel.INSTANCE.f(x, u);
    Tensor expl = Se2Integrator.INSTANCE.step(flow, x, h);
    Tensor impl = RungeKutta45Integrator.INSTANCE.step(flow, x, h);
    assertTrue(Chop._07.close(impl, expl));
  }

  private static Tensor exp_of(Scalar x, Scalar y, Scalar theta) {
    Tensor matrix = Array.zeros(3, 3);
    matrix.set(theta, 1, 0);
    matrix.set(theta.negate(), 0, 1);
    matrix.set(x, 0, 2);
    matrix.set(y, 1, 2);
    return MatrixExp.of(matrix);
  }

  private static Tensor exp_of(Number x, Number y, Number theta) {
    return exp_of(RealScalar.of(x), RealScalar.of(y), RealScalar.of(theta));
  }

  public void testExpSubstitute() {
    Tensor mat = exp_of(1, 2, .3);
    Tensor vec = Se2Utils.combine(Array.zeros(3), Tensors.vector(1, 2, .3));
    Tensor v0 = Se2Utils.combine0(Tensors.vector(1, 2, .3));
    assertEquals(vec, v0);
    Tensor alt = Se2Utils.toSE2Matrix(vec);
    assertTrue(Chop._13.close(mat, alt));
  }
}
