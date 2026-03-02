project('number-generator-app', 'cpp',
  version : '0.1',
  default_options : ['cpp_std=c++17'])

# 1. Find the libcamera dependency (uses pkg-config under the hood)
libcamera_dep = dependency('libcamera', required : true)

# 2. Identify your source files
sources = [
  'main.cpp'
  # for later
]

# 3. Define the executable
executable('rng_bin',
  sources,
  dependencies : libcamera_dep,
  install : true)