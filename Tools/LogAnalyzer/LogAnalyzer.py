#!/usr/bin/env python
#
# A module to analyze and identify any common problems which can be determined from log files
#
# Initial code by Andrew Chapman (amchapman@gmail.com), 16th Jan 2014
#

# AP_FLAKE8_CLEAN

# some logging oddities noticed while doing this, to be followed up on:
#   - tradheli MOT labels Mot1,Mot2,Mot3,Mot4,GGain
#   - Pixhawk doesn't output one of the FMT labels... forget which one
#   - MAG offsets seem to be constant (only seen data on Pixhawk)
#   - MAG offsets seem to be cast to int before being output? (param is -84.67, logged as -84)
#   - copter+plane use 'V' in their vehicle type/version/build line, rover uses lower case 'v'.
#     Copter+Rover give a build number, plane does not
#   - CTUN.ThrOut on copter is 0-1000, on plane+rover it is 0-100

# TODO: add test for noisy baro values
# TODO: support loading binary log files (use Tridge's mavlogdump?)

from __future__ import print_function

import argparse
import datetime
import glob
import os
import sys
import time
from xml.sax.saxutils import escape

import DataflashLog
from VehicleType import VehicleType


class TestResult(object):
    '''all tests return a standardized result type'''

    class StatusType:
        # NA means not applicable for this log (e.g. copter tests against a plane log)
        # UNKNOWN means it is missing data required for the test
        GOOD, FAIL, WARN, UNKNOWN, NA = range(5)

    status = None
    statusMessage = ""  # can be multi-line


class Test(object):
    """
    Base class to be inherited by log tests. Each test should be quite granular so we have lots of small tests with
    clear results
    """

    def __init__(self):
        self.name = ""
        self.result = None  # will be an instance of TestResult after being run
        self.execTime = None
        self.enable = True

    def run(self, logdata, verbose=False):
        pass


class TestSuite(object):
    '''registers test classes, loading using a basic plugin architecture, and can run them all in one run() operation'''

    def __init__(self):
        self.tests = []
        self.logfile = None
        self.logdata = None
        # dynamically load in Test subclasses from the 'tests' folder
        # to prevent one being loaded, move it out of that folder, or set that test's .enable attribute to False
        dirName = os.path.dirname(os.path.abspath(__file__))
        testScripts = glob.glob(f'{dirName}/tests/*.py')

        def getTests(module_path):
            import inspect

            tests = []
            module_name = os.path.basename(module_path)[:-3]
            if sys.version_info >= (3, 5):
                from importlib.util import spec_from_file_location, module_from_spec

                spec = spec_from_file_location(module_name, module_path)
                m = module_from_spec(spec)
                spec.loader.exec_module(m)
            else:
                from imp import load_source

                m = load_source(module_name, module_path)
            for _, _class in inspect.getmembers(m, inspect.isclass):
                if _class.__module__ == m.__name__:
                    tests.append(_class())
            return tests

        for script in testScripts:
            self.tests.extend(getTests(script))

        # and here's an example of explicitly loading a Test class if you wanted to do that
        # spec = importlib.util.spec_from_file_location("m", dirName + "/tests/TestBadParams.py")
        # m = importlib.util.module_from_spec(spec)
        # spec.loader.exec_module(m)
        # self.tests.append(m.TestBadParams())

    def run(self, logdata, verbose):
        '''run all registered tests in a single call, gathering execution timing info'''
        self.logdata = logdata
        if 'GPS' not in self.logdata.channels and 'GPS2' in self.logdata.channels:
            # *cough*
            self.logdata.channels['GPS'] = self.logdata.channels['GPS2']

        self.logfile = logdata.filename
        for test in self.tests:
            # run each test in turn, gathering timing info
            if test.enable:
                startTime = time.time()
                test.run(self.logdata, verbose)  # RUN THE TEST
                endTime = time.time()
                test.execTime = 1000 * (endTime - startTime)

    def outputPlainText(self, outputStats):
        '''output test results in plain text'''
        print(f'Dataflash log analysis report for file: {self.logfile}')
        print('Log size: %.2fmb (%d lines)' % (self.logdata.filesizeKB / 1024.0, self.logdata.lineCount))
        print('Log duration: %s' % str(datetime.timedelta(seconds=self.logdata.durationSecs)) + '\n')

        if self.logdata.vehicleType == VehicleType.Copter and self.logdata.getCopterType():
            print(
                f'Vehicle Type: {self.logdata.vehicleTypeString} ({self.logdata.getCopterType()})'
            )
        else:
            print(f'Vehicle Type: {self.logdata.vehicleTypeString}')
        print(
            f'Firmware Version: {self.logdata.firmwareVersion} ({self.logdata.firmwareHash})'
        )
        print(f'Hardware: {self.logdata.hardwareType}')
        print(f'Free RAM: {self.logdata.freeRAM}')
        if self.logdata.skippedLines:
            print("\nWARNING: %d malformed log lines skipped during read" % self.logdata.skippedLines)
        print('\n')

        print("Test Results:")
        for test in self.tests:
            if not test.enable:
                continue
            statusMessageFirstLine = test.result.statusMessage.strip('\n\r').split('\n')[0]
            statusMessageExtra = test.result.statusMessage.strip('\n\r').split('\n')[1:]
            execTime = ""
            if outputStats:
                execTime = "  (%6.2fms)" % (test.execTime)
            if test.result.status == TestResult.StatusType.GOOD:
                print("  %20s:  GOOD       %-55s%s" % (test.name, statusMessageFirstLine, execTime))
            elif test.result.status == TestResult.StatusType.FAIL:
                print("  %20s:  FAIL       %-55s%s    [GRAPH]" % (test.name, statusMessageFirstLine, execTime))
            elif test.result.status == TestResult.StatusType.WARN:
                print("  %20s:  WARN       %-55s%s    [GRAPH]" % (test.name, statusMessageFirstLine, execTime))
            elif test.result.status == TestResult.StatusType.NA:
                # skip any that aren't relevant for this vehicle/hardware/etc
                continue
            else:
                print("  %20s:  UNKNOWN    %-55s%s" % (test.name, statusMessageFirstLine, execTime))
            # if statusMessageExtra:
            for line in statusMessageExtra:
                print("  %29s     %s" % ("", line))

        print('\n')
        print(
            "The Log Analyzer is currently BETA code."
            "\nFor any support or feedback on the log analyzer please email Andrew Chapman (amchapman@gmail.com)"
        )
        print('\n')

    def outputXML(self, xmlFile):
        '''output test results to an XML file'''

        # open the file for writing
        xml = None
        try:
            xml = sys.stdout if xmlFile == '-' else open(xmlFile, 'w')
        except IOError:
            sys.stderr.write(f"Error opening output xml file: {xmlFile}")
            sys.exit(1)

        # output header info
        xml.write("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n")
        xml.write("<loganalysis>\n")
        xml.write("<header>\n")
        xml.write(f"  <logfile>{escape(self.logfile)}" + "</logfile>\n")
        xml.write(f"  <sizekb>{escape(repr(self.logdata.filesizeKB))}" + "</sizekb>\n")
        xml.write(
            f"  <sizelines>{escape(repr(self.logdata.lineCount))}"
            + "</sizelines>\n"
        )
        xml.write(
            f"  <duration>{escape(str(datetime.timedelta(seconds=self.logdata.durationSecs)))}"
            + "</duration>\n"
        )
        xml.write(
            f"  <vehicletype>{escape(self.logdata.vehicleTypeString)}"
            + "</vehicletype>\n"
        )
        if self.logdata.vehicleType == VehicleType.Copter and self.logdata.getCopterType():
            xml.write(
                f"  <coptertype>{escape(self.logdata.getCopterType())}"
                + "</coptertype>\n"
            )
        xml.write(
            f"  <firmwareversion>{escape(self.logdata.firmwareVersion)}"
            + "</firmwareversion>\n"
        )
        xml.write(
            f"  <firmwarehash>{escape(self.logdata.firmwareHash)}"
            + "</firmwarehash>\n"
        )
        xml.write(
            f"  <hardwaretype>{escape(self.logdata.hardwareType)}"
            + "</hardwaretype>\n"
        )
        xml.write(f"  <freemem>{escape(repr(self.logdata.freeRAM))}" + "</freemem>\n")
        xml.write(
            f"  <skippedlines>{escape(repr(self.logdata.skippedLines))}"
            + "</skippedlines>\n"
        )
        xml.write("</header>\n")

        # output parameters
        xml.write("<params>\n")
        for param, value in self.logdata.parameters.items():
            xml.write("  <param name=\"%s\" value=\"%s\" />\n" % (param, escape(repr(value))))
        xml.write("</params>\n")

        # output test results
        xml.write("<results>\n")
        for test in self.tests:
            if not test.enable:
                continue
            xml.write("  <result>\n")
            if test.result.status == TestResult.StatusType.GOOD:
                xml.write(f"    <name>{escape(test.name)}" + "</name>\n")
                xml.write("    <status>GOOD</status>\n")
                xml.write(f"    <message>{escape(test.result.statusMessage)}" + "</message>\n")
            elif test.result.status == TestResult.StatusType.FAIL:
                xml.write(f"    <name>{escape(test.name)}" + "</name>\n")
                xml.write("    <status>FAIL</status>\n")
                xml.write(f"    <message>{escape(test.result.statusMessage)}" + "</message>\n")
                xml.write("    <data>(test data will be embedded here at some point)</data>\n")
            elif test.result.status == TestResult.StatusType.WARN:
                xml.write(f"    <name>{escape(test.name)}" + "</name>\n")
                xml.write("    <status>WARN</status>\n")
                xml.write(f"    <message>{escape(test.result.statusMessage)}" + "</message>\n")
                xml.write("    <data>(test data will be embedded here at some point)</data>\n")
            elif test.result.status == TestResult.StatusType.NA:
                xml.write(f"    <name>{escape(test.name)}" + "</name>\n")
                xml.write("    <status>NA</status>\n")
            else:
                xml.write(f"    <name>{escape(test.name)}" + "</name>\n")
                xml.write("    <status>UNKNOWN</status>\n")
                xml.write(f"    <message>{escape(test.result.statusMessage)}" + "</message>\n")
            xml.write("  </result>\n")
        xml.write("</results>\n")

        xml.write("</loganalysis>\n")

        xml.close()


def main():
    # deal with command line arguments
    parser = argparse.ArgumentParser(description='Analyze an APM Dataflash log for known issues')
    parser.add_argument('logfile', type=argparse.FileType('r'), help='path to Dataflash log file (or - for stdin)')
    parser.add_argument(
        '-f',
        '--format',
        metavar='',
        type=str,
        action='store',
        choices=['bin', 'log', 'auto'],
        default='auto',
        help='log file format: \'bin\',\'log\' or \'auto\'',
    )
    parser.add_argument(
        '-q', '--quiet', metavar='', action='store_const', const=True, help='quiet mode, do not print results'
    )
    parser.add_argument(
        '-p', '--profile', metavar='', action='store_const', const=True, help='output performance profiling data'
    )
    parser.add_argument(
        '-s', '--skip_bad', metavar='', action='store_const', const=True, help='skip over corrupt dataflash lines'
    )
    parser.add_argument(
        '-e', '--empty', metavar='', action='store_const', const=True, help='run an initial check for an empty log'
    )
    parser.add_argument(
        '-x',
        '--xml',
        type=str,
        metavar='XML file',
        nargs='?',
        const='',
        default='',
        help='write output to specified XML file (or - for stdout)',
    )
    parser.add_argument('-v', '--verbose', metavar='', action='store_const', const=True, help='verbose output')
    args = parser.parse_args()

    # load the log
    startTime = time.time()
    logdata = DataflashLog.DataflashLog(args.logfile.name, format=args.format, ignoreBadlines=args.skip_bad)  # read log
    endTime = time.time()
    if args.profile:
        print("Log file read time: %.2f seconds" % (endTime - startTime))

    # check for empty log if requested
    if args.empty:
        if emptyErr := DataflashLog.DataflashLogHelper.isLogEmpty(logdata):
            sys.stderr.write(f"Empty log file: {logdata.filename}, {emptyErr}")
            sys.exit(1)

    # run the tests, and gather timings
    testSuite = TestSuite()
    startTime = time.time()
    testSuite.run(logdata, args.verbose)  # run tests
    endTime = time.time()
    if args.profile:
        print("Test suite run time: %.2f seconds" % (endTime - startTime))

    # deal with output
    if not args.quiet:
        testSuite.outputPlainText(args.profile)
    if args.xml:
        testSuite.outputXML(args.xml)
        if not args.quiet:
            print("XML output written to file: %s\n" % args.xml)


if __name__ == "__main__":
    main()
