// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import java.util.Comparator;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalars;

/** compare two nodes based on {@link GlcNode#merit()} */
/* package */ enum NodeDepthComparator implements Comparator<GlcNode> {
  INSTANCE;
  // ---
  @Override
  public int compare(GlcNode o1, GlcNode o2) {
    return Scalars.compare(RealScalar.of(o1.depth()), RealScalar.of(o2.depth()));
  }
}
