// code by jl
package ch.ethz.idsc.owly.glc.core;

import ch.ethz.idsc.owly.data.tree.Nodes;

public enum DebugUtils {
  ;
  // ---
  // function for convenience
  @Deprecated // TODO deprecated because root should be passed to function
  public static final void nodeAmountCompare(TrajectoryPlanner trajectoryPlanner) {
    nodeAmountCompare( //
        Nodes.rootFrom(trajectoryPlanner.getBestOrElsePeek().get()), trajectoryPlanner.domainMap().size());
  }

  // ---
  public static final void nodeAmountCompare(GlcNode node, int size) {
    if (size != Nodes.ofSubtree(node).size()) {
      System.out.println("****NODE CHECK****");
      System.out.println("Nodes in DomainMap: " + size);
      System.out.println("Nodes in SubTree from Node: " + Nodes.ofSubtree(node).size());
      throw new RuntimeException();
    }
  }

  /** @param best getBestOrElsePeek()
   * @param node
   * @param size domainMap.size() */
  static final void nodeAmountCheck(GlcNode best, GlcNode node, int size) {
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
