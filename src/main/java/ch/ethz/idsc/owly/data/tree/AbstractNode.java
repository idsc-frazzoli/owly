// code by jph
package ch.ethz.idsc.owly.data.tree;

public abstract class AbstractNode<T extends Node> implements Node {
  /** parent is null for root node */
  private T parent = null;

  /** function has to be provided by deriving class that
   * holds the data structure to store the child nodes.
   * 
   * function adds/inserts given child to collection children()
   * 
   * @param child
   * @return true if child was added to children() as a result of calling the function,
   * false if child was already present, or could not be added to children() */
  protected abstract boolean protected_registerChild(T child);

  @Override // from Node
  public final T parent() {
    return parent;
  }
  
  public void setParent(T parent){
    this.parent = parent;
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
//    if (!child.isRoot()) // child has parent
//      throw new RuntimeException();
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
