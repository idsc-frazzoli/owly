// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import java.util.Comparator;

import ch.ethz.idsc.tensor.Scalars;

/** compare two nodes based on {@link GlcNode#cost()} */
/* package */ enum NodeCostComparator implements Comparator<GlcNode> {
  instance;
  // ---
  @Override
  public int compare(GlcNode o1, GlcNode o2) {
    return Scalars.compare(o1.costFromRoot(), o2.costFromRoot());
  }
}
