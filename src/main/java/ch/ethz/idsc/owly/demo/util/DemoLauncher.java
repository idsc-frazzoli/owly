// code by jph
package ch.ethz.idsc.owly.demo.util;

import java.awt.GridLayout;
import java.lang.reflect.Modifier;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.WindowConstants;

import lcm.util.ClassDiscovery;
import lcm.util.ClassPaths;
import lcm.util.ClassVisitor;

public enum DemoLauncher {
  ;
  public static void main(String[] args) {
    List<Class<?>> demos = new LinkedList<>();
    ClassVisitor classVisitor = new ClassVisitor() {
      @Override
      public void classFound(String jarfile, Class<?> cls) {
        if (DemoInterface.class.isAssignableFrom(cls))
          if (!Modifier.isAbstract(cls.getModifiers()))
            demos.add(cls);
      }
    };
    ClassDiscovery.execute(ClassPaths.getDefault(), classVisitor);
    // ---
    Collections.sort(demos, new Comparator<Class<?>>() {
      @Override
      public int compare(Class<?> o1, Class<?> o2) {
        return o1.getName().compareToIgnoreCase(o2.getName());
      }
    });
    JFrame jFrame = new JFrame();
    jFrame.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
    JPanel jPanel = new JPanel(new GridLayout(demos.size(), 1));
    for (Class<?> cls : demos) {
      JButton jButton = new JButton(cls.getSimpleName());
      jButton.addActionListener(event -> {
        try {
          DemoInterface demoInterface = (DemoInterface) cls.getConstructors()[0].newInstance();
          demoInterface.start();
        } catch (Exception exception) {
          exception.printStackTrace();
        }
      });
      jPanel.add(jButton);
    }
    jFrame.setContentPane(jPanel);
    jFrame.setBounds(1200, 100, 250, 40 + demos.size() * 20);
    jFrame.setVisible(true);
  }
}
