# SPDX-FileCopyrightText: Copyright (c) 2025 lbuque
#
# SPDX-License-Identifier: MIT

import os
import sys
import time
from os.path import join

Import("env")

try:
    import esptool
except:
    try:
        sys.path.append(join(env['UPLOADER'], '..'))
        import esptool
    except:
        env.Execute("$PYTHONEXE -m pip install esptool")
        import esptool

verbose = False

if verbose:
    for item in env.Dictionary():
        print('%s : %s' % (item, env[item]))
        print()

def merge_bin_files(env):
    upload_cmd = env['UPLOADERFLAGS']
    if verbose:
        print(upload_cmd)
    chip_type = env['BOARD_MCU']

    board_config = env.BoardConfig()
    flash_size = board_config.get("upload.flash_size", "4MB")
    flash_mode = env['BOARD_FLASH_MODE']

    # 默认输出到 .pio/bin/ 目录
    OUTPUT_DIR = join(env['PROJECT_BUILD_DIR'], '..', 'bin')

    output_dir = env.GetProjectOption('merge_bin_output_dir', default=OUTPUT_DIR)
    output_file = env.GetProjectOption('merge_bin_output_file', default="{0}_{1}_{2}.bin".format(
                                os.path.basename(env['PROJECT_DIR']),
                                env['PIOENV'],
                                time.strftime("%Y%m%d_%H%M%S", time.localtime())
                            ))

    if not os.path.exists(output_dir):
        os.mkdir(output_dir)

    outputFilename = join(output_dir, output_file)

    version_tuple = tuple(map(int, esptool.__version__.split('.')[:2]))
    if verbose:
        print("esptool version:", esptool.__version__)

    commands = []
    commands.append('--chip')
    commands.append(chip_type)
    if version_tuple >= (5, 0):
        commands.append('merge-bin')
    else:
        commands.append('merge_bin')
    commands.append('-o')
    commands.append(outputFilename)
    if version_tuple >= (5, 0):
        commands.append('--flash-size')
    else:
        commands.append('--flash_size')
    commands.append(flash_size)
    # commands.append('--flash-mode')
    # commands.append(flash_mode)

    if verbose:
        print("extra images:")
        for item in env['FLASH_EXTRA_IMAGES']:
            print("%s: %s" % (item[0], item[1]))

    if verbose:
        print("%s: %s" % (env['ESP32_APP_OFFSET'], join(env['PROJECT_BUILD_DIR'], env['PIOENV'], '{}.bin'.format(env['PROGNAME']))))

    for item in env['FLASH_EXTRA_IMAGES']:
        commands.append(item[0])
        commands.append(upload_cmd[upload_cmd.index(item[0]) + 1])
    commands.append(env['ESP32_APP_OFFSET'])
    commands.append(join(env['PROJECT_BUILD_DIR'], env['PIOENV'], '{}.bin'.format(env['PROGNAME'])))

    if verbose:
        print("commands:", commands)

    esptool.main(commands)


def before_upload(source, target, env):
    merge_bin_files(env)

def after_buildprog(source, target, env):
    merge_bin_files(env)

env.AddPostAction("buildprog", after_buildprog)
# env.AddPostAction("checkprogsize", after_buildprog)
# env.AddPreAction("upload", before_upload)
# env.AddPostAction("$PROGPATH", after_buildprog)
# env.AddPostAction("$LINK", after_buildprog)