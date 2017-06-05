package ch.ethz.idsc.owly.data.tree;

public interface TempNode<T extends TempNode<?>> {
  T getSome();

  void putSome(T asd);
}
