// code by jph
package ch.ethz.idsc.owly.glc.core;

public interface ExpandInterface {
  /** retrieves next node and removes the node from the queue
   * 
   * @return next node for expansion, or null if no such node exists */
  GlcNode pollNext();

  /** performs expansion at given node
   * 
   * @param node */
  void expand(GlcNode node);

  /** @return best node in goal region, or null if no such node has been identified yet */
  GlcNode getBest();
}
