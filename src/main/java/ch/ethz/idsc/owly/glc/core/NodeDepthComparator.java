// code by jl
package ch.ethz.idsc.owly.glc.core;

import java.util.Comparator;
import java.util.Map.Entry;

import ch.ethz.idsc.tensor.Tensor;

/** compare two nodes based on {@link GlcNode#merit()} */
/* package */ enum NodeDepthComparator implements Comparator<Entry<Tensor, GlcNode>> {
  INSTANCE;
  // ---
  @Override
  public int compare(Entry<Tensor, GlcNode> o1, Entry<Tensor, GlcNode> o2) {
    return Integer.compare(o1.getValue().depth(), o2.getValue().depth());
  }
}
