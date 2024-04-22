# systems_bringup

This package brings up the systems for the RoboCup Team, running all the nodes in the same process, allowing:
* Use intra-process communications
* Select which nodes are real-time and which ones aren't
* Unifying all the node parameter configurations in the same file

## Setup the system for launch real-time threads

Normal users are not allowed by default to set high-priority threads, so you have to add a file `/etc/security/limits.d/20-YOURUSERNAME-rtprio.conf` (for example, `/etc/security/limits.d/20-fmrico-rtprio.conf`) with the content:

```
YOURUSERNAME - rtprio 98
```
