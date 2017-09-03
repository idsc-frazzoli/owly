// code by jph
package ch.ethz.idsc.owly.math.sample;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.red.Mean;
import ch.ethz.idsc.tensor.red.Norm;
import junit.framework.TestCase;

public class BoxRandomSampleTest extends TestCase {
  public void testSimple3D() {
    Tensor offset = Tensors.vector(2, 2, 3);
    Tensor width = Tensors.vector(1, 1, 1);
    RandomSampleInterface rsi = new BoxRandomSample(offset.subtract(width), offset.add(width));
    Tensor rand = RandomSample.of(rsi, 100);
    Scalars.compare(Norm._2.ofVector(Mean.of(rand).subtract(offset)), RealScalar.of(.1));
  }
}
