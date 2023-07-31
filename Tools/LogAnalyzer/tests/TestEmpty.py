# AP_FLAKE8_CLEAN


import DataflashLog
from LogAnalyzer import Test, TestResult


class TestEmpty(Test):
    '''test for empty or near-empty logs'''

    def __init__(self):
        Test.__init__(self)
        self.name = "Empty"

    def run(self, logdata, verbose):
        self.result = TestResult()
        self.result.status = TestResult.StatusType.GOOD

        if emptyErr := DataflashLog.DataflashLogHelper.isLogEmpty(logdata):
            self.result.status = TestResult.StatusType.FAIL
            self.result.statusMessage = f"Empty log? {emptyErr}"
