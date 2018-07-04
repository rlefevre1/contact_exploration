#!/bin/bash

hpp-manipulation-server &
gepetto-gui &
ipython -i --no-confirm-exit $1

pkill -f "gepetto-gui"
pkill -f "hpp-manipulation-server"
