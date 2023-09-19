#!/bin/sh
set -e
if test "$CONFIGURATION" = "Debug"; then :
  cd /Users/jingouyang/Personal/Courseware/Surface_Registeration/zzzlib/Registeration/Xcode
  make -f /Users/jingouyang/Personal/Courseware/Surface_Registeration/zzzlib/Registeration/Xcode/CMakeScripts/ReRunCMake.make
fi
if test "$CONFIGURATION" = "Release"; then :
  cd /Users/jingouyang/Personal/Courseware/Surface_Registeration/zzzlib/Registeration/Xcode
  make -f /Users/jingouyang/Personal/Courseware/Surface_Registeration/zzzlib/Registeration/Xcode/CMakeScripts/ReRunCMake.make
fi
if test "$CONFIGURATION" = "MinSizeRel"; then :
  cd /Users/jingouyang/Personal/Courseware/Surface_Registeration/zzzlib/Registeration/Xcode
  make -f /Users/jingouyang/Personal/Courseware/Surface_Registeration/zzzlib/Registeration/Xcode/CMakeScripts/ReRunCMake.make
fi
if test "$CONFIGURATION" = "RelWithDebInfo"; then :
  cd /Users/jingouyang/Personal/Courseware/Surface_Registeration/zzzlib/Registeration/Xcode
  make -f /Users/jingouyang/Personal/Courseware/Surface_Registeration/zzzlib/Registeration/Xcode/CMakeScripts/ReRunCMake.make
fi

