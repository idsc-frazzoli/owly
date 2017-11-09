// code by jph
package ch.ethz.idsc.owly.math;

import java.util.Arrays;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Array;
import ch.ethz.idsc.tensor.alg.Dimensions;
import ch.ethz.idsc.tensor.red.Norm2Squared;
import ch.ethz.idsc.tensor.sca.Chop;
import junit.framework.TestCase;

public class CirclePointsTest extends TestCase {
  public void testSimple() {
    int n = 5;
    Tensor tensor = CirclePoints.of(n);
    assertEquals(Dimensions.of(tensor), Arrays.asList(n, 2));
    Tensor nrm = Tensor.of(tensor.stream().map(Norm2Squared::ofVector));
    assertTrue(Chop._14.close(nrm, Array.of(l -> RealScalar.ONE, n)));
  }

  public void testScaled() {
    int n = 11;
    Tensor tensor = CirclePoints.elliptic(n, RealScalar.of(2), RealScalar.of(.5));
    // System.out.println(Pretty.of(tensor.map(Round._4)));
    assertEquals(Dimensions.of(tensor), Arrays.asList(n, 2));
  }

  public void testFirst() {
    int n = 5;
    Tensor tensor = CirclePoints.of(n);
    assertEquals(tensor.get(0), Tensors.vector(1, 0));
  }
}