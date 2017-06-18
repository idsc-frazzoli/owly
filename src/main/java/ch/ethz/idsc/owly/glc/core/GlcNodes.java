// code by jph
package ch.ethz.idsc.owly.glc.core;

import java.util.List;
import java.util.stream.Collectors;

import ch.ethz.idsc.owly.data.tree.Nodes;
import ch.ethz.idsc.owly.math.state.StateTime;

public enum GlcNodes {
  ;
  // TODO perhaps rename to coarse path ...
  /** @return path to goal if found, or path to current Node in queue
   * @throws Exception if best is null and queue is empty */
  public static List<StateTime> getPathFromRootTo(GlcNode node) {
    return Nodes.listFromRoot(node).stream() //
        .map(GlcNode::stateTime).collect(Collectors.toList());
  }
}
