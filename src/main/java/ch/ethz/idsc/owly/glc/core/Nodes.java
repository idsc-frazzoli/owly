// code by jph
package ch.ethz.idsc.owly.glc.core;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/** utility functions that operate on {@link Node}s */
/* package */ enum Nodes {
  ;
  // ---
  /** @param node
   * @return */
  public static List<Node> nodesToRoot(Node node) {
    List<Node> list = new ArrayList<>();
    Node next = node;
    while (next != null) {
      list.add(next);
      next = next.parent();
    }
    return list;
  }

  /** @param node
   * @return */
  public static List<Node> nodesFromRoot(Node node) {
    List<Node> list = nodesToRoot(node);
    Collections.reverse(list);
    return list;
  }
}
