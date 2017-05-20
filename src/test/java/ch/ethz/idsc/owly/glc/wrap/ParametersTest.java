package ch.ethz.idsc.owly.glc.wrap;

import ch.ethz.idsc.owly.demo.glc.se2.Se2StateSpaceModel;
import ch.ethz.idsc.owly.demo.glc.se2glc.Se2Parameters;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.N;
import junit.framework.TestCase;

public class ParametersTest extends TestCase {
  public void testDepthtoZero() {
    Scalar timeScale = RealScalar.of(10);
    Scalar depthScale = RealScalar.of(5);
    Tensor partitionScale = Tensors.vector(3, 3, 15);
    Scalar dtMax = RationalScalar.of(1, 6);
    int maxIter = 2000;
    StateSpaceModel stateSpaceModel = new Se2StateSpaceModel();
    Scalar resolution = RationalScalar.of(2, 1); // resolution is bound by Integer.MAX_VALUE
    Scalar oldValue = RealScalar.of(1000);
    Scalar newValue = oldValue;
    long iter = 0;
    while (++iter < 1000) {
      resolution = resolution.multiply(RealScalar.of(2));
      Parameters test = new Se2Parameters(//
          (RationalScalar) resolution, timeScale, depthScale, partitionScale, dtMax, maxIter, stateSpaceModel.getLipschitz());
      oldValue = newValue;
      newValue = resolution.divide(test.getDepthLimitExact());
      System.out.println(resolution);
      System.out.println("values:");
      System.out.println(N.of(oldValue));
      System.out.println(N.of(newValue));
      System.out.println(oldValue);
      System.out.println(newValue);
      assertTrue(Scalars.lessEquals(newValue, oldValue));
      if (Scalars.lessThan(newValue.abs(), RealScalar.of(0.001)))
        break;
    }
    System.out.println(iter);
    assertTrue(iter < 1000);
  }
}
