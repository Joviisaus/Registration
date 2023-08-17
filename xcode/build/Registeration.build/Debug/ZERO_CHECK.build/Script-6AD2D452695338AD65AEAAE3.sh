#!/bin/sh
set -e
if test "$CONFIGURATION" = "Debug"; then :
  cd /Users/jingouyang/Personal/Courseware/曲面配准/zzzlib/Registeration/xcode
  make -f /Users/jingouyang/Personal/Courseware/曲面配准/zzzlib/Registeration/xcode/CMakeScripts/ReRunCMake.make
fi
if test "$CONFIGURATION" = "Release"; then :
  cd /Users/jingouyang/Personal/Courseware/曲面配准/zzzlib/Registeration/xcode
  make -f /Users/jingouyang/Personal/Courseware/曲面配准/zzzlib/Registeration/xcode/CMakeScripts/ReRunCMake.make
fi
if test "$CONFIGURATION" = "MinSizeRel"; then :
  cd /Users/jingouyang/Personal/Courseware/曲面配准/zzzlib/Registeration/xcode
  make -f /Users/jingouyang/Personal/Courseware/曲面配准/zzzlib/Registeration/xcode/CMakeScripts/ReRunCMake.make
fi
if test "$CONFIGURATION" = "RelWithDebInfo"; then :
  cd /Users/jingouyang/Personal/Courseware/曲面配准/zzzlib/Registeration/xcode
  make -f /Users/jingouyang/Personal/Courseware/曲面配准/zzzlib/Registeration/xcode/CMakeScripts/ReRunCMake.make
fi

