// code by jph
package ch.ethz.idsc.owly.math.sample;

import ch.ethz.idsc.tensor.Tensor;

public class ConstantRandomSample implements RandomSample {
  private final Tensor tensor;

  public ConstantRandomSample(Tensor sample) {
    tensor = sample.unmodifiable();
  }

  @Override
  public Tensor nextSample() {
    return tensor;
  }
}
