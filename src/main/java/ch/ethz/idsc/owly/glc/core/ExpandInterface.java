// code by jph
package ch.ethz.idsc.owly.glc.core;

public interface ExpandInterface {
  /** @return next node for expansion, or null if no such node exists */
  Node pollNext();

  /** performs expansion at given node
   * 
   * @param node */
  void expand(Node node);

  /** @return best node in goal region, or null if no such node has been identified yet */
  Node getBest();
}
