// code by edo
package ch.ethz.idsc.owly.demo.drift;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Chop;
import ch.ethz.idsc.tensor.sca.Round;
import junit.framework.TestCase;

public class DriftStateSpaceModelTest extends TestCase {
  // TODO the number currently match the MATLAB implementation => build asserts (later)
  public void testSimple() {
    DriftParameters driftParameters = new DriftParameters();
    DriftStateSpaceModel driftStateSpaceModel = new DriftStateSpaceModel(driftParameters);
    {
      Tensor dx = driftStateSpaceModel.f(Tensors.vector(0, 0, 1), Tensors.vector(0, 0));
      assertEquals(dx, Tensors.vector(0, 0, 0));
    }
    {
      Tensor dx = driftStateSpaceModel.f(Tensors.vector(3, 2, 1), Tensors.vector(0, 0));
      System.out.println(dx.map(Round._4));
      Tensor aprox = Tensors.vector(-3.3825, -4.3477, 6);
      // assertTrue(Chop._05.close(dx, aprox));
    }
    {
      Tensor dx = driftStateSpaceModel.f(Tensors.vector(1, -1, 2), Tensors.vector(0, 0));
      // assertEquals(dx, Tensors.vector(0, 0, 0));
      System.out.println(dx.map(Round._4));
    }
    {
      Tensor dx = driftStateSpaceModel.f(Tensors.vector(-2, 1, -1), Tensors.vector(1, 1000));
      // assertEquals(dx, Tensors.vector(0, 0, 0));
      System.out.println(dx.map(Round._4));
    }
  }

  public void testFzF() {
    DriftParameters driftParameters = new DriftParameters();
    DriftStateSpaceModel driftStateSpaceModel = new DriftStateSpaceModel(driftParameters);
    Scalar FzF = driftParameters.Fz_F();
    Scalar FzR = driftParameters.Fz_R();
    assertTrue(Chop._05.close(FzF, RealScalar.of(9020.278144329897)));
    assertTrue(Chop._05.close(FzR, RealScalar.of(4831.441855670103)));
  }
}
