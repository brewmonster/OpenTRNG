project('number-generator-app', 'cpp',
  version : '0.1',
  default_options : ['cpp_std=c++20', 'optimization=1'])
  
# 1. Find the libcamera dependency (uses pkg-config under the hood)
libcamera_dep = dependency('libcamera', required : true)
openssl_dep = dependency('openssl', required : true)

# 2. Identify your source files
sources = [
  'src/main.cpp',
  'src/EntropySource.cpp',
  'src/HashDigester.cpp',
  'src/OutputPort.cpp'
]

inc_dir = include_directories('inc')
# 3. Define the executable
executable('truerng',
  sources,
  include_directories : inc_dir,
  dependencies : [libcamera_dep, openssl_dep],
  #dependencies : libcamera_dep,
  install : true)
