// code by jph
package ch.ethz.idsc.owly.demo.se2;

import ch.ethz.idsc.owly.math.TensorUnaryOperator;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Floor;
import junit.framework.TestCase;

public class Se2WrapTest extends TestCase {
  static Tensor convertToKey(Tensor eta, TensorUnaryOperator represent, Tensor x) {
    return eta.pmul(represent.apply(x)).map(Floor.FUNCTION);
  }

  public void testSimple() {
    Se2Wrap se2Wrap = new Se2Wrap(Tensors.vector(1, 1, 2));
    Tensor eta = Tensors.vector(3, 3, 50 / Math.PI);
    {
      Tensor rep = convertToKey(eta, se2Wrap::represent, Tensors.vector(0, 0, 0));
      assertEquals(rep.Get(2), RealScalar.ZERO);
    }
    {
      Tensor rep = convertToKey(eta, se2Wrap::represent, Tensors.vector(0, 0, Math.PI - 0.0001));
      assertEquals(rep.Get(2), RealScalar.of(49));
    }
    {
      Tensor rep = convertToKey(eta, se2Wrap::represent, Tensors.vector(0, 0, Math.PI));
      assertEquals(rep.Get(2), RealScalar.of(50));
    }
    {
      Tensor rep = convertToKey(eta, se2Wrap::represent, Tensors.vector(0, 0, 2 * Math.PI - 0.0001));
      assertEquals(rep.Get(2), RealScalar.of(99));
    }
    {
      Tensor rep = convertToKey(eta, se2Wrap::represent, Tensors.vector(0, 0, -0.0001));
      assertEquals(rep.Get(2), RealScalar.of(99));
    }
  }
}
