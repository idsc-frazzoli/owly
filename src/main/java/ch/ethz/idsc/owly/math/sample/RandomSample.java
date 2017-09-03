// code by jph
package ch.ethz.idsc.owly.math.sample;

import java.io.Serializable;

import ch.ethz.idsc.tensor.Tensor;

/** inspired by
 * <a href="https://reference.wolfram.com/language/ref/RandomSample.html">RandomSample</a> */
public interface RandomSample extends Serializable {
  /** @return random sample from continuous or discrete set */
  Tensor nextSample();
}
