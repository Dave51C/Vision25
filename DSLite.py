# $Source: /home/scrobotics/src/DriveStation/RCS/DSLite.py,v $
# $Revision: 1.3 $
# $Date: 2025/03/27 02:37:49 $
# $Author: scrobotics $

#                    OVERLAY
#                  Show    Hide
#                |   0   |   1
#           -----+-------+-------
#   SHOW Head 0  |  00   |  10
#        Tail 1  |  01   |  11
#
#   Cmd:  0)  Show overlay, Head (fore) camera
#         1)  Show overlay, Tail (aft) camera
#         2)  Hide overlay, show Head camera
#         3)  Hide overlay, show Tail camera
#

import ntcore
import curses
import sys

IP='10.42.0.1'
#IP='10.2.50.2'

if len(sys.argv) >= 2:
    IP = sys.argv[1]

stdscr = curses.initscr()

def show_choices():
  stdscr.addstr(0,0,"PICK ONE")
  stdscr.move(1,0)
  stdscr.clrtoeol()
  stdscr.addstr(1,0,"0 - Head Cam with Overlay")
  stdscr.addstr(2,0,"1 - Tail Cam with Overlay")
  stdscr.addstr(3,0,"2 - Head Cam without Overlay")
  stdscr.addstr(4,0,"3 - Tail Cam without Overlay")
  stdscr.addstr(5,0,"Q - end this program")
  stdscr.addstr(6,0,"Selection: ")

def main():
  curses.noecho()
  curses.cbreak()
  stdscr.keypad(True)
  show_choices()
  pub1.set('0')

  while True:
      key = stdscr.getch()
      if key == ord('0'):
          stdscr.addstr(6,11,'0')
          pub1.set('0')
          stdscr.move(6,12)
      elif key == ord('1'):
          stdscr.addstr(6,11,'1')
          pub1.set('1')
          stdscr.move(6,12)
      elif key == ord('2'):
          stdscr.addstr(6,11,'2')
          pub1.set('2')
          stdscr.move(6,12)
      elif key == ord('3'):
          stdscr.addstr(6,11,'3')
          pub1.set('3')
          stdscr.move(6,12)
      elif key == ord('Q') or key == ord('q'):
          stdscr.addstr(6,11,'Q')
          stdscr.move(6,12)
          break
  
  curses.nocbreak()
  stdscr.keypad(False)
  curses.echo()
  curses.endwin()

if __name__ == "__main__":
  # Initialize NT4 client
  inst = ntcore.NetworkTableInstance.getDefault()
  inst.startClient4("DSLite")
  inst.setServer(IP)
  # publish one values
  table = inst.getTable("DS")
  #pub1 = table.getIntegerTopic("View").publish()
  pub1 = table.getStringTopic("View").publish()

  main()
