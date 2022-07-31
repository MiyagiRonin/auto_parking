load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "vec2d",
    srcs = ["vec2d.cc"],
    hdrs = ["vec2d.h"],
    linkopts = ["-lm"],
    deps = [
    ],
)

cc_library(
    name = "math_utils",
    srcs = ["math_utils.cc"],
    hdrs = ["math_utils.h"],
    linkopts = ["-lm"],
    deps = [
        ":vec2d",
    ],
)

cc_library(
    name = "line_segment2d",
    srcs = ["line_segment2d.cc"],
    hdrs = ["line_segment2d.h"],
    linkopts = ["-lm"],
    deps = [
        ":vec2d",
        ":math_utils",
    ],
)

cc_library(
    name = "aabox2d",
    srcs = ["aabox2d.cc"],
    hdrs = ["aabox2d.h"],
    linkopts = ["-lm"],
    deps = [
        ":vec2d",
        ":math_utils",
    ],
)

cc_library(
    name = "box2d",
    srcs = ["box2d.cc"],
    hdrs = ["box2d.h"],
    linkopts = ["-lm"],
    deps = [
        ":aabox2d",
        ":line_segment2d",
    ],
)
