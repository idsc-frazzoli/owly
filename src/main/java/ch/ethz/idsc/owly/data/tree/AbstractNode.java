// code by jph
package ch.ethz.idsc.owly.data.tree;

public abstract class AbstractNode<T extends Node> implements Node {
  /** parent is null for root node */
  private T parent = null;

  protected abstract boolean protected_registerChild(T node);

  @Override // from Node
  public final T parent() {
    return parent;
  }

  @SuppressWarnings("unchecked")
  @Override // from Node
  public final void removeEdgeTo(Node child) {
    boolean modified = children().remove(child);
    if (!modified)
      throw new RuntimeException();
    if (child.isRoot())
      throw new RuntimeException();
    ((AbstractNode<T>) child).parent = null;
  }

  @SuppressWarnings("unchecked")
  @Override // from Node
  public final void insertEdgeTo(Node child) {
    boolean modified = protected_registerChild((T) child);
    if (!modified)
      throw new RuntimeException();
    if (!child.isRoot())
      throw new RuntimeException();
    ((AbstractNode<T>) child).parent = (T) this;
  }

  @Override // from Node
  public final boolean isRoot() {
    return parent() == null;
  }

  @Override // from Node
  public final boolean isLeaf() {
    return children().isEmpty();
  }
}
