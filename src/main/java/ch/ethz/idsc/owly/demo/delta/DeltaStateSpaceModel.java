// code by jph
package ch.ethz.idsc.owly.demo.delta;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

public class DeltaStateSpaceModel implements StateSpaceModel {
  private final ImageGradient imageGradient;

  public DeltaStateSpaceModel(ImageGradient imageGradient) {
    this.imageGradient = imageGradient;
  }

  @Override
  public Tensor f(Tensor x, Tensor u) {
    return imageGradient.rotate(x).add(u);
  }

  @Override
  public Scalar getLipschitz() {
    // TODO not sure if |u| should appear in formula
    // TODO maxNorm is very big--> therefore eta with R^(1+LF) is huge? real lipschitz?
    return imageGradient.maxNorm(); // .add(maxInput);
  }
}
