# -*- python -*-
# Copyright 2016 Toyota Research Institute.  All rights reserved.
genrule(
    name = "gen_path_h",
    srcs = ["drake/core/Path.h.in"],
    outs = ["drake/Path.h"],
    cmd = "sed 's#@PROJECT_SOURCE_DIR@#ZOMG#g' $< > $@",
    visibility = ["//visibility:public"],
)

cc_library(
    name = "core",
    # TODO rico: fix upstream drakeGradientUtil.h, but until then ...
    hdrs = glob(["drake/core/*.h"]) + ["drake/Path.h",
                                       "drake/util/drakeGradientUtil.h"],
    visibility = ["//visibility:public"],
    deps = ["@eigen//:headers"],
)

cc_library(
    name = "systems",
    hdrs = glob(["drake/systems/*.h"]),
    visibility = ["//visibility:public"],
    deps = [":core"],
)

cc_test(
    name = "vector_test",
    srcs = ["drake/core/test/vector_test.cc"],
    deps = [":core"],
    visibility = ["//visibility:public"],
    timeout = "short",
)
