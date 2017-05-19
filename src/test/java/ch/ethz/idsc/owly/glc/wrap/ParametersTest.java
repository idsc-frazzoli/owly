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
import junit.framework.TestCase;

public class ParametersTest extends TestCase {
  public void testDepthtoZero() {
    Scalar timeScale = RealScalar.of(10);
    Scalar depthScale = RealScalar.of(5);
    Tensor partitionScale = Tensors.vector(3, 3, 15);
    Scalar dtMax = RationalScalar.of(1, 6);
    int maxIter = 2000;
    StateSpaceModel stateSpaceModel = new Se2StateSpaceModel();
    int resolution = 0; // resolution is bound by Integer.MAX_VALUE
    Scalar oldValue = RealScalar.POSITIVE_INFINITY;
    Scalar newValue = oldValue;
    long iter = 0;
    do {
      iter++;
      resolution = resolution * 2; // TODO after 31 iterations this will become 0 due to integer overflow!
      Parameters test = new Se2Parameters(//
          resolution, timeScale, depthScale, partitionScale, dtMax, maxIter, stateSpaceModel.getLipschitz());
      newValue = RealScalar.of(resolution).divide(RealScalar.of(test.getDepthLimit()));
    } while (iter < 1000 && Scalars.lessThan(RealScalar.of(0.001), (oldValue.subtract(newValue))));
    assertTrue(iter < 1000);
  }
}
