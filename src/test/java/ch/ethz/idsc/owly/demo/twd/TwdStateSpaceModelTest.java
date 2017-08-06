// code by jph
package ch.ethz.idsc.owly.demo.twd;

import java.util.Collection;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Array;
import ch.ethz.idsc.tensor.red.Norm;
import junit.framework.TestCase;

public class TwdStateSpaceModelTest extends TestCase {
  public void testSimple() {
    Scalar wheelRadius = RationalScalar.of(5, 100); // 5[cm]
    Scalar wheelDistance = RationalScalar.of(40, 10); // 40[cm]
    StateSpaceModel ssm = new TwdStateSpaceModel(wheelRadius, wheelDistance);
    { // full speed ahead
      Tensor xdot = ssm.f(Array.zeros(3), Tensors.vector(1, 1).divide(wheelRadius)); // 1[rad]
      assertEquals(xdot, Tensors.vector(1, 0, 0));
    }
    { // only ccw rotation
      Tensor xdot = ssm.f(Array.zeros(3), Tensors.vector(-1, 1).divide(wheelRadius)); // 1[rad]
      assertEquals(xdot, Tensors.vector(0, 0, 0.5));
    }
    { // foward motion with ccw rotation
      Tensor xdot = ssm.f(Array.zeros(3), Tensors.vector(0, 1).divide(wheelRadius)); // 1[rad]
      assertEquals(xdot, Tensors.vector(0.5, 0, 0.25));
    }
  }

  public void testDefault() {
    TwdStateSpaceModel ssm = TwdStateSpaceModel.createDefault();
    Collection<Flow> collection = TwdControls.createControls(ssm, 10);
    for (Flow flow : collection) {
      Tensor fxu = ssm.f(Array.zeros(3), flow.getU());
      Scalar ninf = Norm.INFINITY.of(fxu);
      Scalar none = Norm._1.of(fxu);
      assertTrue(Scalars.lessEquals(ninf, RealScalar.ONE));
      assertTrue(none.equals(RealScalar.ONE));
      // System.out.println(ninf + "\t" + none + "\t" + flow.getU() + "\t" + fxu);
    }
  }
}
