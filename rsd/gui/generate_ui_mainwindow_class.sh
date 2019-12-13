#!/bin/bash

#Removes the Pointsize warning. 
#(https://stackoverflow.com/questions/18531266/qfontsetpointsize-point-size-0-1-must-be-greater-than-0)

cat ui_mainwindow.ui | sed '/<pointsize>-1<\/pointsize>/d' > ui_mainwindow_wo_warnings.ui 
pyside2-uic ui_mainwindow_wo_warnings.ui -o ui_mainwindow.py