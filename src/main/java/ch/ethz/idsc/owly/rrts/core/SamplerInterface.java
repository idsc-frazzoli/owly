// code by jph
package ch.ethz.idsc.owly.rrts.core;

import ch.ethz.idsc.tensor.Tensor;

/**
 * 
 */
public interface SamplerInterface {
  Tensor nextSample();
}
