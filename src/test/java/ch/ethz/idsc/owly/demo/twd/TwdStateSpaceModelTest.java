// code by jph
package ch.ethz.idsc.owly.demo.twd;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Array;
import junit.framework.TestCase;

public class TwdStateSpaceModelTest extends TestCase {
  public void testSimple() {
    Scalar wheelRadius = RationalScalar.of(5, 100); // 5[cm]
    Scalar wheelDistance = RationalScalar.of(40, 10); // 40[cm]
    StateSpaceModel ssm = new TwdStateSpaceModel(wheelRadius, wheelDistance);
    { // full speed ahead
      Tensor xdot = ssm.f(Array.zeros(3), Tensors.vector(1, 1).multiply(wheelRadius.invert())); // 1[rad]
      assertEquals(xdot, Tensors.vector(1, 0, 0));
    }
    { // only ccw rotation
      Tensor xdot = ssm.f(Array.zeros(3), Tensors.vector(-1, 1).multiply(wheelRadius.invert())); // 1[rad]
      assertEquals(xdot, Tensors.vector(0, 0, 0.5));
    }
    { // foward motion with ccw rotation
      Tensor xdot = ssm.f(Array.zeros(3), Tensors.vector(0, 1).multiply(wheelRadius.invert())); // 1[rad]
      assertEquals(xdot, Tensors.vector(0.5, 0, 0.25));
    }
  }
}
