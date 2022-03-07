#!/bin/bash

# The MIT License (MIT)
# 
# Copyright (c) 2022 Rong "Mantle" Bao.
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy of
# this software and associated documentation files (the "Software"), to deal in
# the Software without restriction, including without limitation the rights to use,
# copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
# Software, and to permit persons to whom the Software is furnished to do so, subject
# to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all copies
# or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
# INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
# PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
# HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
# OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

# ARGPARSE PHASE
if [[ $# -lt 0 ]]
then
    echo "usage: build_protobuf.sh";
    exit -1;
fi

# WORKING PHASE
mkdir build

rm -r build/arbiter-message
mkdir build/arbiter-message
mkdir build/arbiter-message/cpp;
mkdir build/arbiter-message/nanopb;

cd arbiter-message;
for filename in `ls`
do
    if [[ $filename == *.proto ]]
    then
        echo "[INFO] Building $filename with protoc";
        protoc --cpp_out ./../build/arbiter-message/cpp/ $filename;
        echo "[INFO] Building $filename with nanopb_generator";
        nanopb_generator --output-dir ./../build/arbiter-message/nanopb/ --quiet $filename;
    fi;
done;
