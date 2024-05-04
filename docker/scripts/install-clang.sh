#!/bin/bash

set -e

# Install LLVM family: compiler, formatter, debugger, etc

apt-get update
apt-get install -y --no-install-recommends \
    clangd-15 \
    clang-15 \
    clang-format-15 \
    lld-15 \
    lldb-15
update-alternatives --install /usr/bin/clang clang /usr/bin/clang-15 100
update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-15 100
update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-15 100
update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-15 100
update-alternatives --install /usr/bin/lld lld /usr/bin/lld-15 100
update-alternatives --install /usr/bin/lldb lldb /usr/bin/lldb-15 100
update-alternatives --install /usr/bin/lldb-server lldb-server /usr/bin/lldb-server-15 100

# Fix https://github.com/llvm/llvm-project/issues/55575
ln -s /usr/lib/llvm-15/lib/python3.10/dist-packages/lldb/* /usr/lib/python3/dist-packages/lldb/