// code by jph
package ch.ethz.idsc.owly.math.sample;

import ch.ethz.idsc.owly.demo.util.UserHome;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.io.Put;

enum CircleRandomSampleDemo {
  ;
  public static void main(String[] args) throws Exception {
    CircleRandomSample circleSampler = //
        new CircleRandomSample(Tensors.vector(1, 1), RealScalar.of(2));
    Tensor matrix = Tensors.empty();
    for (int count = 0; count < 10000; ++count) {
      Tensor vec = circleSampler.nextSample();
      matrix.append(vec);
    }
    Put.of(UserHome.file("samples.txt"), matrix);
  }
}
