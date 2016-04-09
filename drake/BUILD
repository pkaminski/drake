# -*- python -*-
# Copyright 2016 Toyota Research Institute.  All rights reserved.
genrule(
    name = "gen_path_h",
    srcs = ["core/Path.h.in"],
    outs = ["Path.h"],
    cmd = "sed 's#@PROJECT_SOURCE_DIR@#ZOMG#g' $< > $@",
    visibility = ["//visibility:public"],
)

cc_library(
    name = "core",
    # TODO rico: fix upstream drakeGradientUtil.h, but until then ...
    hdrs = glob(["core/*.h"]) + ["Path.h", "util/drakeGradientUtil.h"],
    linkopts = ["-lm"],
    visibility = ["//visibility:public"],
    deps = ["@eigen//:headers"],
)

cc_library(
    name = "systems",
    hdrs = glob(["systems/*.h"]),
    visibility = ["//visibility:public"],
    deps = [":core"],
)

cc_test(
    name = "vector_test",
    srcs = ["core/test/vector_test.cc",
            "core/test/pendulum.h",
            "util/eigen_matrix_compare.h",
            "util/testUtil.h"],
    deps = [":core", "@gtest//:main"],
    visibility = ["//visibility:public"],
    timeout = "short",
)
