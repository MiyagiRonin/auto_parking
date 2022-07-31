load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "geometry_parking_utils",
    srcs = ["geometry_parking_utils.cc"],
    hdrs = ["geometry_parking_utils.h",
    "vehicle_conf.h",],
    deps = [
        "//apollo_math:box2d",
    ],
)

cc_library(
    name = "line_circle_connection",
    srcs = ["line_circle_connection.cc"],
    hdrs = ["line_circle_connection.h",],
    deps = [
        ":geometry_parking_utils",
    ],
)

cc_library(
    name = "reverse_verticle_parking",
    srcs = ["reverse_verticle_parking.cc"],
    hdrs = ["reverse_verticle_parking.h",],
    deps = [
        ":line_circle_connection",
        ":geometry_parking_utils",
    ],
)

# https://docs.bazel.build/versions/master/be/c-cpp.html#cc_binary
cc_binary(
    name = "main",
    srcs = ["main.cc"],
    copts = [],
    deps = [":reverse_verticle_parking",
    "@com_google_absl//absl/strings",
    ],
)