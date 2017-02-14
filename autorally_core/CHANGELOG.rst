^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autorally_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* New camera_trigger Arduino and .cpp code
* removed GPGSV messages from being displayed in diagnostics
* SystemStatus: added disc usage, removed m4api stats that are causing a hang on startup, fixed broken fan speed parsing
* Xbee: rewrite to send GPS RTK corrections better, allow multiple coordinators, transmit/receive state estimates to/from other vehicles
* Improved error checking/handling across the board
* AutorallyChassis: streamlined non-critical error handling
* Fixed incorrect wheel speeds published by autorally_chassis by adding conversion from rotations persecond to m/s
* ChronyStatus diagnostic topic is now color coded to idicate whether it is synchonizing to a clock (green) or not (yellow)
* Added wheelOdometry node

0.2.3 (2016-09-20 11:06)
------------------------

0.2.2 (2016-09-20 11:05:39)
---------------------------

0.2.1 (2016-09-20 11:05:16)
---------------------------

0.2.0 (2016-09-20 11:04)
------------------------
