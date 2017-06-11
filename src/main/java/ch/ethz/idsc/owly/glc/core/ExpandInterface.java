// code by jph
package ch.ethz.idsc.owly.glc.core;

import java.util.Optional;

public interface ExpandInterface {
  /** retrieves next node and removes the node from the queue
   * 
   * @return next node for expansion, or Optional.empty() if no such node exists */
  Optional<GlcNode> pollNext();

  /** performs expansion at given node
   * 
   * @param node */
  void expand(GlcNode node);

  /** @return best node in goal region, or Optional.empty() if no such node has been identified yet */
  Optional<GlcNode> getBest();
}
