// code by jph
package ch.ethz.idsc.owly.demo.glc.delta;

import ch.ethz.idsc.owly.demo.util.Images;
import ch.ethz.idsc.owly.demo.util.Resources;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.io.Import;
import junit.framework.TestCase;

public class ImageGradientTest extends TestCase {
  public void testSimple() throws Exception {
    Tensor range = Tensors.vector(9, 6.5);
    Tensor res;
    Scalar max;
    {
      ImageGradient ig = new ImageGradient( //
          Images.displayOrientation(Import.of(Resources.fileFromRepository("/io/delta_uxy.png")).get(Tensor.ALL, Tensor.ALL, 0)), //
          range, RealScalar.of(.5)); // -.25 .5
      res = ig.rotate(Tensors.vector(2, 3));
      max = ig.maxNorm();
    }
    {
      ImageGradient ig = new ImageGradient( //
          Images.displayOrientation(Import.of(Resources.fileFromRepository("/io/delta_uxy.png")).get(Tensor.ALL, Tensor.ALL, 0)), //
          range, RealScalar.ONE); // -.25 .5
      Tensor cmp = ig.rotate(Tensors.vector(2, 3));
      assertEquals(cmp, res.multiply(RealScalar.of(2)));
      assertEquals(ig.maxNorm(), max.multiply(RealScalar.of(2)));
    }
  }
}
