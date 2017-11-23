// code by jl
package ch.ethz.idsc.owly.glc.any;

import ch.ethz.idsc.owl.data.tree.Nodes;
import ch.ethz.idsc.owly.glc.core.GlcNode;

// VISIBILITY IS PACKAGE
/* package */ enum StaticHelper {
  ;
  /** @param best getBestOrElsePeek()
   * @param node
   * @param size domainMap.size() */
  public static final void nodeAmountCheck(GlcNode best, GlcNode node, int size) {
    final GlcNode root = Nodes.rootFrom(best); //
    int treeSize = Nodes.ofSubtree(root).size();
    if (size != treeSize) {
      System.err.println("DomainMap  != TreeSize:");
      System.err.println(size + " =/= " + treeSize + " after expanding of Node: ");
      System.err.println("DEPTH: " + node.depth());
      System.err.println("State: " + node.state());
      if (node.isRoot())
        System.err.println("Node has no parents");
      if (node.isLeaf())
        System.err.println("Node is leaf");
      throw new RuntimeException();
    }
  }
}
