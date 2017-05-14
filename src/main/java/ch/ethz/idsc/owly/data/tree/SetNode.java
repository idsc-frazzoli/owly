// code by jph
package ch.ethz.idsc.owly.data.tree;

import java.util.Collection;
import java.util.HashSet;
import java.util.Set;

public class SetNode<T extends Node> extends AbstractNode<T> {
  private final Set<T> next = new HashSet<>();

  @Override // from Node
  public final Collection<T> children() {
    return next;
  }

  @Override // from AbstractNode
  protected final boolean protected_registerChild(T node) {
    return next.add(node);
  }
}
