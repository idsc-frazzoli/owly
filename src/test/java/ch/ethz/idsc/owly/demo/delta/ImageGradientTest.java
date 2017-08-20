// code by jph
package ch.ethz.idsc.owly.demo.delta;

import java.util.Arrays;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Dimensions;
import ch.ethz.idsc.tensor.io.ResourceData;
import junit.framework.TestCase;

public class ImageGradientTest extends TestCase {
  public void testSimple() throws Exception {
    Tensor range = Tensors.vector(9, 6.5);
    Tensor res;
    Scalar max;
    final Tensor image = ResourceData.of("/io/delta_uxy.png");
    assertEquals(Dimensions.of(image), Arrays.asList(128, 179));
    {
      ImageGradient ig = new ImageGradient(image, range, RealScalar.of(.5)); // -.25 .5
      res = ig.rotate(Tensors.vector(2, 3));
      max = ig.maxNorm();
    }
    {
      ImageGradient ig = new ImageGradient(image, range, RealScalar.ONE); // -.25 .5
      Tensor cmp = ig.rotate(Tensors.vector(2, 3));
      assertEquals(cmp, res.multiply(RealScalar.of(2)));
      assertEquals(ig.maxNorm(), max.multiply(RealScalar.of(2)));
    }
  }
}
