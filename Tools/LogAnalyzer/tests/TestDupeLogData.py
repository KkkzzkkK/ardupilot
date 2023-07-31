# AP_FLAKE8_CLEAN


from __future__ import print_function

from LogAnalyzer import Test, TestResult


class TestDupeLogData(Test):
    '''test for duplicated data in log, which has been happening on PX4/Pixhawk'''

    def __init__(self):
        Test.__init__(self)
        self.name = "Dupe Log Data"

    def __matchSample(self, sample, sampleStartIndex, logdata):
        '''return the line number where a match is found, otherwise return False'''

        nSame = sum(1 for s in sample if s[1] == sample[0][1])
        if nSame == 20:
            return False

        # c
        data = logdata.channels["ATT"]["Pitch"].listData
        for i in range(sampleStartIndex, len(data)):
            if i == sampleStartIndex:
                continue  # skip matching against ourselves
            j = 0
            while j < 20 and (i + j) < len(data) and data[i + j][1] == sample[j][1]:
                j += 1
            if j == 20:  # all samples match
                return data[i][0]

        return False

    def run(self, logdata, verbose):
        self.result = TestResult()
        self.result.status = TestResult.StatusType.GOOD

        # this could be made more flexible by not hard-coding to use ATT data, could make it dynamic based on whatever
        # is available as long as it is highly variable
        if "ATT" not in logdata.channels:
            self.result.status = TestResult.StatusType.UNKNOWN
            self.result.statusMessage = "No ATT log data"
            return

        attEndIndex = len(logdata.channels["ATT"]["Pitch"].listData) - 1
        step = int(attEndIndex / 11)
        sampleStartIndices = list(range(step, attEndIndex - step, step))
        # get 20 datapoints of pitch from each sample location and check for a match elsewhere
        sampleIndex = 0
        for i in range(sampleStartIndices[0], len(logdata.channels["ATT"]["Pitch"].listData)):
            if i == sampleStartIndices[sampleIndex]:
                sample = logdata.channels["ATT"]["Pitch"].listData[i : i + 20]
                if matchedLine := self.__matchSample(sample, i, logdata):
                    self.result.status = TestResult.StatusType.FAIL
                    self.result.statusMessage = "Duplicate data chunks found in log (%d and %d)" % (
                        sample[0][0],
                        matchedLine,
                    )
                    return
                sampleIndex += 1
                if sampleIndex >= len(sampleStartIndices):
                    break
