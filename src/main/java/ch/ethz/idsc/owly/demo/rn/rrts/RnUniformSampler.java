// code by jph
package ch.ethz.idsc.owly.demo.rn.rrts;

import java.util.LinkedList;
import java.util.List;

import ch.ethz.idsc.owly.data.GlobalAssert;
import ch.ethz.idsc.owly.rrts.core.SamplerInterface;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.pdf.Distribution;
import ch.ethz.idsc.tensor.pdf.RandomVariate;
import ch.ethz.idsc.tensor.pdf.UniformDistribution;

public class RnUniformSampler implements SamplerInterface {
  private final List<Distribution> distributions = new LinkedList<>();

  /** the parameters define the coordinate bounds of the cube
   * from which the samples are drawn
   * 
   * @param min lower-left
   * @param max upper-right */
  public RnUniformSampler(Tensor min, Tensor max) {
    GlobalAssert.that(min.length() == max.length());
    for (int index = 0; index < min.length(); ++index)
      distributions.add(UniformDistribution.of(min.Get(index), max.Get(index)));
  }

  @Override
  public Tensor nextSample() {
    return Tensor.of(distributions.stream().map(RandomVariate::of));
  }
}
