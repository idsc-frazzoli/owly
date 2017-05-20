// code by jph
package ch.ethz.idsc.owly.data.tree;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

public enum Nodes {
  ;
  /** @param node
   * @return */
  @SuppressWarnings("unchecked")
  public static <T extends Node> List<T> toRoot(T node) {
    List<T> list = new ArrayList<>();
    T next = node;
    while (next != null) {
      list.add(next);
      next = (T) next.parent();
    }
    return list;
  }

  /** @param node
   * @return */
  public static <T extends Node> List<T> fromRoot(T node) {
    List<T> list = toRoot(node);
    Collections.reverse(list);
    return list;
  }

  @SuppressWarnings("unchecked")
  private static <T extends Node> void _ofSubtree(T node, Collection<T> collection) {
    collection.add(node);
    node.children().stream().forEach(child -> _ofSubtree((T) child, collection));
  }

  public static <T extends Node> Collection<T> ofSubtree(T node) {
    Collection<T> collection = new ArrayList<>();
    _ofSubtree(node, collection);
    return collection;
  }

  public static <T extends Node> T disjoinAt(T node) {
    @SuppressWarnings("unchecked")
    T parent = (T) node.parent();
    if (parent != null)
      parent.removeEdgeTo(node);
    return parent;
  }
}
