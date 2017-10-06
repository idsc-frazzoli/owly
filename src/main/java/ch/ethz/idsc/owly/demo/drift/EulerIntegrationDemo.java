// code by edo
package ch.ethz.idsc.owly.demo.drift;

import java.io.IOException;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

public enum EulerIntegrationDemo {
  ;
  public static void main(String[] args) throws IOException {
    DriftStateSpaceModel driftStateSpaceModel = new DriftStateSpaceModel(new DriftParameters());
    int N = 10;
    Scalar dt = RealScalar.of(0.01);
    Tensor u = Tensors.vector(-10 * Math.PI / 180, 1500);
    Tensor x0 = Tensors.vector(0, 0, 4);
    Tensor xNext, xCurr;
    xCurr = x0;
    for (int i = 0; i < N; i++) {
      xNext = xCurr.add(driftStateSpaceModel.f(xCurr, u).multiply(dt));
      xCurr = xNext;
    }
    System.out.println(xCurr);
  }
}
