// code by jph
package ch.ethz.idsc.owly.glc.core;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/** utility functions that operate on {@link Node}s */
/* package */ enum Nodes {
  ;
  // ---
  public static List<Node> nodesToRoot(Node best) {
    List<Node> list = new ArrayList<>();
    Node node = best;
    while (node != null) {
      list.add(node);
      node = node.parent();
    }
    return list;
  }

  public static List<Node> nodesFromRoot(Node best) {
    List<Node> list = nodesToRoot(best);
    Collections.reverse(list);
    return list;
  }
}
