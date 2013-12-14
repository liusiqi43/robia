FILE(REMOVE_RECURSE
  "CMakeFiles/packages"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/packages.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
