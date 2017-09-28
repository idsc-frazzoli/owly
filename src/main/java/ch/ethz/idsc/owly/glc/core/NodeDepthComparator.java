// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import java.util.Comparator;
import java.util.Map.Entry;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;

/** compare two nodes based on {@link GlcNode#merit()} */
/* package */ enum NodeDepthComparator implements Comparator<Entry<Tensor, GlcNode>> {
  INSTANCE;
  // ---
  @Override
  public int compare(Entry o1, Entry o2) {
    return Scalars.compare(RealScalar.of(((GlcNode) o1.getValue()).depth()), RealScalar.of(((GlcNode) o2.getValue()).depth()));
  }
}
