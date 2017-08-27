// code by jph
package ch.ethz.idsc.owly.rrts.core;

import ch.ethz.idsc.tensor.Tensor;

/** inspired by
 * <a href="https://reference.wolfram.com/language/ref/RandomSample.html">RandomSample</a> */
public interface RandomSampleInterface {
  /** @return random sample from continuous or discrete set */
  Tensor nextSample();
}
