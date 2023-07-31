# encoding: utf-8

"""
Waf tool for ESP32 build
"""

from waflib import Build, ConfigSet, Configure, Context, Task, Utils
from waflib import Errors, Logs
from waflib.TaskGen import before, after_method, before_method, feature
from waflib.Configure import conf
from collections import OrderedDict

import os
import shutil
import sys
import re
import pickle
import subprocess

def configure(cfg):
    mcu_esp32s3 = cfg.variant[:7] == "esp32s3"
    target = "esp32s3" if mcu_esp32s3 else "esp32"
    bldnode = cfg.bldnode.make_node(cfg.variant)
    def srcpath(path):
        return cfg.srcnode.make_node(path).abspath()

    def bldpath(path):
        return bldnode.make_node(path).abspath()

    #Load cmake builder and make
    cfg.load('cmake')

    #define env and location for the cmake esp32 file
    env = cfg.env
    env.AP_HAL_ESP32 = srcpath(f'libraries/AP_HAL_ESP32/targets/{target}/esp-idf')
    env.AP_PROGRAM_FEATURES += ['esp32_ap_program']

    env.ESP_IDF_PREFIX_REL = 'esp-idf'

    prefix_node = bldnode.make_node(env.ESP_IDF_PREFIX_REL)
    env.ESP32_TARGET = target
    env.BUILDROOT = bldpath('')
    env.SRCROOT = srcpath('')
    env.APJ_TOOL = srcpath('Tools/scripts/apj_tool.py')

    #Check if esp-idf env are loaded, or load it
    try:
        env.IDF = os.environ['IDF_PATH']
    except:
        env.IDF = f"{cfg.srcnode.abspath()}/modules/esp_idf"
    print(f"USING EXPRESSIF IDF:{str(env.IDF)}")

    try:
        env.DEFAULT_PARAMETERS = os.environ['DEFAULT_PARAMETERS']
    except:
        env.DEFAULT_PARAMETERS = f"{cfg.srcnode.abspath()}/libraries/AP_HAL_ESP32/boards/defaults.parm"
    print(f"USING DEFAULT_PARAMETERS:{str(env.DEFAULT_PARAMETERS)}")

    #env.append_value('GIT_SUBMODULES', 'esp_idf')


def pre_build(self):
    """Configure esp-idf as lib target"""
    lib_vars = OrderedDict()
    lib_vars['ARDUPILOT_CMD'] = self.cmd
    lib_vars['ARDUPILOT_LIB'] = self.bldnode.find_or_declare('lib/').abspath()
    lib_vars['ARDUPILOT_BIN'] = self.bldnode.find_or_declare('lib/bin').abspath()
    target = self.env.ESP32_TARGET
    esp_idf = self.cmake(
        name='esp-idf',
        cmake_vars=lib_vars,
        cmake_src=f'libraries/AP_HAL_ESP32/targets/{target}/esp-idf',
        cmake_bld='esp-idf_build',
    )

    esp_idf_showinc = esp_idf.build('showinc', target='esp-idf_build/includes.list')
    esp_idf_showinc.post()

    from waflib import Task


    class load_generated_includes(Task.Task):
        """After includes.list generated include it in env"""
        always_run = True
        def run(self):
            bld = self.generator.bld
            includes = bld.bldnode.find_or_declare('esp-idf_build/includes.list').read().split()
            #print(includes)
            bld.env.prepend_value('INCLUDES', includes)


    tsk = load_generated_includes(env=self.env)
    tsk.set_inputs(self.path.find_resource('esp-idf_build/includes.list'))
    self.add_to_group(tsk)


@feature('esp32_ap_program')
@after_method('process_source')
def esp32_firmware(self):
    self.link_task.always_run = True
    esp_idf = self.bld.cmake('esp-idf')

    build = esp_idf.build('all', target='esp-idf_build/ardupilot.bin')
    build.post()

    build.cmake_build_task.set_run_after(self.link_task)

    # tool that can update the default params in a .bin or .apj
    #self.default_params_task = self.create_task('set_default_parameters',
    #                                          src='esp-idf_build/ardupilot.bin')
    #self.default_params_task.set_run_after(self.generate_bin_task)

    # optional upload is last
    if self.bld.options.upload:
        flasher = esp_idf.build('flash')
        flasher.post()


class set_default_parameters(Task.Task):
    color='CYAN'
    always_run = True
    def keyword(self):
        return "setting default params"
    def run(self):

        # TODO: disabled this task outright as apjtool appears to destroy checksums and/or the esp32 partition table
        # TIP:  if u do try this, afterwards, be sure to 'rm -rf build/esp32buzz/idf-plane/*.bin' and re-run waf
        return




