load("@rules_cc//cc:defs.bzl", "cc_library")
load("@AvrToolchain//platforms/cpu_frequency:cpu_frequency.bzl", "cpu_frequency_flag")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "mcu_firmware_base",
    srcs = [
        "analog_reader.cc",
        "binary_commander.cc",
        "main.cc",
    ],
    hdrs = [
        "analog_reader.h",
        "binary_commander.h",
        "debug_led.h",
        "temperature.h",
    ],
    copts = cpu_frequency_flag() + [
        "-Os",
        "-ffunction-sections",
        "-fdata-sections",
        "-flto",
        "-ffat-lto-objects",
        "-fuse-linker-plugin",
        "-nodevicelib",
        "-Wl,--gc-sections",
        "-Wl,--strip-all",
    ],
    strip_include_prefix = "//src/mcu_firmware",
    deps = [
        "//src/libs/communication:binary_communication",
        "@arduino//:arduino_core",
        "@arduino//:arduino_eeprom_lib",
        "@arduino//:arduino_wire_lib",
        "//src/mcu_firmware/base/simple_foc:simplefoc_custom",
        #"@simplefoc//:simplefoc_lib",
    ],
)
