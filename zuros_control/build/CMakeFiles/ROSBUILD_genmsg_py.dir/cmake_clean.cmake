FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/zuros_control/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/zuros_control/msg/__init__.py"
  "../src/zuros_control/msg/_motorMSG.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
