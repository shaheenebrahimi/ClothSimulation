#!/bin/sh
set -e
if test "$CONFIGURATION" = "Debug"; then :
  cd /Users/shaheenebrahimi/Developer/ComputerAnimation/A5/build
  make -f /Users/shaheenebrahimi/Developer/ComputerAnimation/A5/build/CMakeScripts/ReRunCMake.make
fi
if test "$CONFIGURATION" = "Release"; then :
  cd /Users/shaheenebrahimi/Developer/ComputerAnimation/A5/build
  make -f /Users/shaheenebrahimi/Developer/ComputerAnimation/A5/build/CMakeScripts/ReRunCMake.make
fi
if test "$CONFIGURATION" = "MinSizeRel"; then :
  cd /Users/shaheenebrahimi/Developer/ComputerAnimation/A5/build
  make -f /Users/shaheenebrahimi/Developer/ComputerAnimation/A5/build/CMakeScripts/ReRunCMake.make
fi
if test "$CONFIGURATION" = "RelWithDebInfo"; then :
  cd /Users/shaheenebrahimi/Developer/ComputerAnimation/A5/build
  make -f /Users/shaheenebrahimi/Developer/ComputerAnimation/A5/build/CMakeScripts/ReRunCMake.make
fi

