#!/bin/bash

base_path=$(cd `dirname $0`; pwd)
echo $base_path/build/devel/setup.bash
source $base_path/build/devel/setup.bash
echo "ABCDEFGHIJKLMNOPQRSTUVWXYZ$.+-*\\0123456789"
rosservice call write_word_server "ABCDEFGHIJKLMNOPQRSTUVWXYZ$+-*\\0123456789" "ABCDEFGHIJKLMNOPQRSTUVWXYZ$+-*\\0123456789"
