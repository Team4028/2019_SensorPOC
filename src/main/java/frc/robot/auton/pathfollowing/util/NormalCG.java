package frc.robot.auton.pathfollowing.util;
/*


import java.util.Enumeration;
import java.util.Vector;

import static java.util.Objects.requireNonNull;

import java.util.Enumeration;
import java.util.Vector;

import org.omg.PortableServer.ImplicitActivationPolicyOperations;

import edu.wpi.first.wpilibj.command.Command;

import static java.util.Objects.requireNonNull;



public class NormalCG extends Command {

  private final Vector<Entry> m_commands = new Vector<>();

  final Vector<Entry> m_children = new Vector<>();

  private int m_currentCommandIndex = -1;
  


  public NormalCG() {
  }

  public NormalCG(String name) {
    super(name);
  }

  public final synchronized void addSequential(Command command) {
    validate("Can not add new command to command group");
    if (command == null) {
      throw new IllegalArgumentException("Given null command");
    }

    command.setParent(this);

    m_commands.addElement(new Entry(command, Entry.IN_SEQUENCE));
    for (Enumeration e = command.getRequirements(); e.hasMoreElements(); ) {
      requires((Subsystem) e.nextElement());
    }
  }

  public final synchronized void addSequential(Command command, double timeout) {
    validate("Can not add new command to command group");
    if (command == null) {
      throw new IllegalArgumentException("Given null command");
    }
    if (timeout < 0) {
      throw new IllegalArgumentException("Can not be given a negative timeout");
    }

    command.setParent(this);

    m_commands.addElement(new Entry(command, Entry.IN_SEQUENCE, timeout));
    for (Enumeration e = command.getRequirements(); e.hasMoreElements(); ) {
      requires((Subsystem) e.nextElement());
    }
  }

  public final synchronized void addParallel(Command command) {
    requireNonNull(command, "Provided command was null");
    validate("Can not add new command to command group");

    command.setParent(this);

    m_commands.addElement(new Entry(command, Entry.BRANCH_CHILD));
    for (Enumeration e = command.getRequirements(); e.hasMoreElements(); ) {
      requires((Subsystem) e.nextElement());
    }
  }

  
  public final synchronized void addParallel(Command command, double timeout) {
    requireNonNull(command, "Provided command was null");
    if (timeout < 0) {
      throw new IllegalArgumentException("Can not be given a negative timeout");
    }
    validate("Can not add new command to command group");

    command.setParent(this);

    m_commands.addElement(new Entry(command, Entry.BRANCH_CHILD, timeout));
    for (Enumeration e = command.getRequirements(); e.hasMoreElements(); ) {
      requires((Subsystem) e.nextElement());
    }
  }
















  @Override
  void _initialize() {
    m_currentCommandIndex = -1;
  }

  @Override


  void _execute() {

    //deals with running commands
    Entry entry = null;
    Command cmd = null;
    boolean firstRun = false;
    if (m_currentCommandIndex == -1) {
      firstRun = true;
      m_currentCommandIndex = 0;
    }

    while (m_currentCommandIndex < m_commands.size()) {
      if (cmd != null) {
        if (entry.isTimedOut()) {
          cmd._cancel();
        }
        if (cmd.run()) {
          break;
        } else {
          cmd.removed();
          m_currentCommandIndex++;
          firstRun = true;
          cmd = null;
          continue;
        }
      }

      entry = m_commands.elementAt(m_currentCommandIndex);
      cmd = null;

      switch (entry.m_state) {
        case Entry.IN_SEQUENCE:
          cmd = entry.m_command;
          if (firstRun) {
            cmd.startRunning();
            cancelConflicts(cmd);
          }
          firstRun = false;
          break;
        case Entry.BRANCH_PEER:
          m_currentCommandIndex++;
          entry.m_command.start();
          break;
        case Entry.BRANCH_CHILD:
          m_currentCommandIndex++;
          cancelConflicts(entry.m_command);
          entry.m_command.startRunning();
          m_children.addElement(entry);
          break;
        default:
          break;
      }
    }

    // Run Children
    for (int i = 0; i < m_children.size(); i++) {
      entry = m_children.elementAt(i);
      Command child = entry.m_command;
      if (entry.isTimedOut()) {
        child._cancel();
      }
      if (!child.run()) {
        child.removed();
        m_children.removeElementAt(i--);
      }
    }
  }






  
  @Override
  void _end() {
    if (m_currentCommandIndex != -1 && m_currentCommandIndex < m_commands.size()) {
      Command cmd = m_commands.elementAt(m_currentCommandIndex).m_command;
      cmd._cancel();
      cmd.removed();
    }

    Enumeration children = m_children.elements();
    while (children.hasMoreElements()) {
      Command cmd = ((Entry) children.nextElement()).m_command;
      cmd._cancel();
      cmd.removed();
    }
    m_children.removeAllElements();
  }


  @Override
  void _interrupted() {
    _end();
  }

  @Override
  protected boolean isFinished() {
    return m_currentCommandIndex >= m_commands.size() && m_children.isEmpty();
  }

  @Override
  protected void initialize() {
  }


  @Override
  protected void execute() {
  }


  @Override
  protected void end() {
  }


  @Override
  protected void interrupted() {
  }


  @Override
  public synchronized boolean isInterruptible() {
    if (!super.isInterruptible()) {
      return false;
    }

    if (m_currentCommandIndex != -1 && m_currentCommandIndex < m_commands.size()) {
      Command cmd = m_commands.elementAt(m_currentCommandIndex).m_command;
      if (!cmd.isInterruptible()) {
        return false;
      }
    }

    for (int i = 0; i < m_children.size(); i++) {
      if (!m_children.elementAt(i).m_command.isInterruptible()) {
        return false;
      }
    }

    return true;
  }

  private void cancelConflicts(Command command) {
    for (int i = 0; i < m_children.size(); i++) {
      Command child = m_children.elementAt(i).m_command;

      Enumeration requirements = command.getRequirements();

      while (requirements.hasMoreElements()) {
        Object requirement = requirements.nextElement();
        if (child.doesRequire((Subsystem) requirement)) {
          child._cancel();
          child.removed();
          m_children.removeElementAt(i--);
          break;
        }
      }
    }
  }

  private static class Entry {
    private static final int IN_SEQUENCE = 0;
    private static final int BRANCH_PEER = 1;
    private static final int BRANCH_CHILD = 2;
    private final Command m_command;
    private final int m_state;
    private final double m_timeout;

    Entry(Command command, int state) {
      m_command = command;
      m_state = state;
      m_timeout = -1;
    }

    Entry(Command command, int state, double timeout) {
      m_command = command;
      m_state = state;
      m_timeout = timeout;
    }

    boolean isTimedOut() {
      if (m_timeout == -1) {
        return false;
      } else {
        double time = m_command.timeSinceInitialized();
        return time != 0 && time >= m_timeout;
      }
    }
  }
}

*/