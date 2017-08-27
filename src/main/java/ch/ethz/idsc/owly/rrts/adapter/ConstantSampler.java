// code by jph
package ch.ethz.idsc.owly.rrts.adapter;

import ch.ethz.idsc.owly.rrts.core.RandomSampleInterface;
import ch.ethz.idsc.tensor.Tensor;

public class ConstantSampler implements RandomSampleInterface {
  private final Tensor tensor;

  public ConstantSampler(Tensor sample) {
    tensor = sample.unmodifiable();
  }

  @Override
  public Tensor nextSample() {
    return tensor;
  }
}
