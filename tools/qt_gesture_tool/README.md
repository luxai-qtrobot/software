# QTrobot gesture tools

QTrobot gesture tools is a simple python script for playing and recording new gestures. 


```bash 
QTrobot gesture utility - (C) 2018 LuxAI S.A
Usage:
      qt_gesture list                   list available gestures
      qt_gesture play   <name>          play a gesture given by its <name>
      qt_gesture record <name> <parts>  record a new gesture. <parts> is a list of 'head', 'right_arm', 'left_arm'
      qt_gesture home                   move all parts to home postion

Example:
      $ qt_gesture record mygesture "right_arm, left_arm"
      $ qt_gesture play mygesture
```
