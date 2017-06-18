// code by jph
package ch.ethz.idsc.owly.data.tree;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

public enum Nodes {
  ;
  // ---
  /** @param node
   * @return root that is the result of visiting the parents from given start node */
  @SuppressWarnings("unchecked")
  public static <T extends Node> T rootFrom(T node) {
    T root = node;
    while (root.parent() != null)
      root = (T) root.parent();
    return root;
  }

  /** @param node
   * @return */
  @SuppressWarnings("unchecked")
  public static <T extends Node> List<T> listToRoot(T node) {
    if (node == null)
      throw new RuntimeException();
    List<T> list = new ArrayList<>();
    while (node != null) {
      list.add(node);
      node = (T) node.parent();
    }
    return list;
  }

  /** @param node
   * @return */
  public static <T extends Node> List<T> listFromRoot(T node) {
    List<T> list = listToRoot(node);
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
