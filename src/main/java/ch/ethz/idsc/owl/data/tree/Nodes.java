// code by jph
package ch.ethz.idsc.owl.data.tree;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

import ch.ethz.idsc.owly.glc.any.OptimalAnyTrajectoryPlanner;

/** utility functions */
public enum Nodes {
  ;
  // ---
  /** @param node
   * @return root that is the result of visiting the parents from given start node */
  @SuppressWarnings("unchecked")
  public static <T extends Node> T rootFrom(T node) {
    T root = node;
    while (Objects.nonNull(root.parent()))
      root = (T) root.parent();
    return root;
  }

  /** @param node
   * @return */
  @SuppressWarnings("unchecked")
  public static <T extends Node> List<T> listToRoot(T node) {
    if (Objects.isNull(node))
      throw new NullPointerException();
    List<T> list = new ArrayList<>();
    while (Objects.nonNull(node)) {
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

  public static <T extends Node> boolean areConnected(T node1, T node2) {
    return (Nodes.listToRoot(node1).contains(node2) || Nodes.listToRoot(node2).contains(node1));
  }

  @SuppressWarnings("unchecked")
  public static <T extends Node> void ofSubtree(T node, Collection<T> collection) {
    collection.add(node);
    node.children().stream().forEach(child -> ofSubtree((T) child, collection));
  }

  /** applications may sort the collection, for instance {@link OptimalAnyTrajectoryPlanner}
   * 
   * @param node
   * @return */
  public static <T extends Node> Collection<T> ofSubtree(T node) {
    Collection<T> collection = new ArrayList<>();
    ofSubtree(node, collection);
    return collection;
  }

  public static <T extends Node> T disjoinAt(T node) {
    @SuppressWarnings("unchecked")
    T parent = (T) node.parent();
    if (Objects.nonNull(parent))
      parent.removeEdgeTo(node);
    return parent;
  }
}
