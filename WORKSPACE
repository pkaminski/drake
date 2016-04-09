# -*- python -*-
# Copyright 2016 Toyota Research Institute.  All rights reserved.
# This file marks a workspace root for the Bazel build system. see
# http://bazel.io/ .

new_http_archive(
    name = "gtest",
    url = "https://googletest.googlecode.com/files/gtest-1.7.0.zip",
    sha256 = "247ca18dd83f53deb1328be17e4b1be31514cedfc1e3424f672bf11fd7e0d60d",
    build_file = "gtest.BUILD",
    strip_prefix = "gtest-1.7.0",
)

new_git_repository(
    name = "drake",
    remote = "https://github.com/RobotLocomotion/drake.git",
    commit = "42590ee",
    build_file = "drake.BUILD",
)

new_git_repository(
    name = "eigen",
    remote = "https://github.com/RLovelett/eigen.git",
    tag = "3.3-beta1",
    build_file = "eigen.BUILD",
)
