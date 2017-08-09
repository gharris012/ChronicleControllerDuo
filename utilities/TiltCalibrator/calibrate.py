
import sys
import datetime
import time
import os

import threading
import thread

import numpy

from scipy.interpolate import interp1d
from scipy import arange, array, exp
import csv
import functools
import ConfigParser

#Simple offset calibration if only one point available.
def offsetCalibration(offset, value):
    return value + offset

#More complex interpolation calibration if more than one calibration point available
def extrapolationCalibration(extrapolationFunction, value):
    inputValue = [value]
    returnValue = extrapolationFunction(inputValue)
    return returnValue[0]

def noCalibration(value):
    return value

#Median utility function
def median(values):
        return numpy.median(numpy.array(values))

    #Load the calibration settings from file and create the calibration functions
def tiltHydrometerCalibrationFunction(type, colour):
    returnFunction = noCalibration

    originalValues = []
    actualValues = []
    csvFile = None
    filename = type.upper() + "." + colour.lower()

    print "loading calibration data from " + filename
    try:
        if (os.path.isfile(filename)):
            #print "opening file"
            csvFile = open(filename, "rb")
            csvFileReader = csv.reader(csvFile, skipinitialspace=True)

            lineNumber = 1
            for row in csvFileReader:
                #print "TiltHydrometer (" + colour + "): File - " + filename  + ", Line " + str(lineNumber) + " processing [" + str(row) + "]"
                #Skip any comment rows and rows with no configuration data

                if ( len(row) > 0 and row[0][:1] == "#" ):
                    # ignore comments
                    pass
                elif ( len(row) != 2 ):
                    if ( len(row) > 0 ):
                        print "WARNING: TiltHydrometer (" + colour + "): File - " + filename  + ", Line " + str(lineNumber) + " was ignored as does not contain valid configuration data [" + str(row) + "]"
                else:
                    print "TiltHydrometer (" + colour + "): File - " + filename  + ", Line " + str(lineNumber) + " processed successfully"
                    originalValues.append(float(row[0]))
                    actualValues.append(float(row[1]))

                lineNumber += 1
            #Close file
            csvFile.close()
    except IOError:
        print "TiltHydrometer (" + colour + "):  " + type.capitalize() + ": No calibration data (" + filename  + ")"
    except Exception, e:
        print "ERROR: TiltHydrometer (" + colour + "): Unable to initialise " + type.capitalize() + " Calibration data (" + filename  + ") - " + e.message
        #Attempt to close the file
        if (csvFile is not None):
            #Close file
            csvFile.close()

    #If more than two values, use interpolation
    if (len(actualValues) >= 2):
        returnFunction = interp1d(originalValues, actualValues, fill_value='extrapolate')
        print "TiltHydrometer (" + colour + "): Initialised " + type.capitalize() + " Calibration: Interpolation"
    #Not enough values. Likely just an offset calculation
    elif (len(actualValues) == 1):
        offset = actualValues[0] - originalValues[0]
        returnFunction = functools.partial(offsetCalibration, offset)
        print "TiltHydrometer (" + colour + "): Initialised " + type.capitalize() + " Calibration: Offset (" + str(offset) + ")"

    print "done. found " + str(len(actualValues)) + " calibration points"
    return returnFunction

argColor = sys.argv[1]
print "Generating calibration data for " + argColor
temperatureCalibrationFunction = tiltHydrometerCalibrationFunction('temperature', argColor)
gravityCalibrationFunction = tiltHydrometerCalibrationFunction('gravity', argColor)

gravityRange = (1000,1070)
temperatureRange = (350,700)
temperatureStep = 5

print
print "// === Gravity Calibration === "
print gravityRange[0], ", // start"
print gravityRange[1], ", // end"
print "// int table[", (gravityRange[1]-gravityRange[0])+1,"];"
print "{"
for row in range(gravityRange[0], gravityRange[1]+1, 1):
    if row % 10 == 0:
        if row > gravityRange[0]:
            print ""
        print " //", range(row, min(row+10, gravityRange[1]+1))
        print "    ",
    print str(int(gravityCalibrationFunction(row))) + (" ",",")[row < gravityRange[1]],
print ""
print "},"

print
print "// === Temperature Calibration === "
print temperatureRange[0], ", // start"
print temperatureRange[1], ", // end"
print "// int table[", ( ( temperatureRange[1] - temperatureRange[0] ) / temperatureStep ) + 1,"];"
print "{"
for row in range(temperatureRange[0], temperatureRange[1] + ( 1 * temperatureStep ), 5):
    if row % ( 10 * temperatureStep) == 0 or row == temperatureRange[0]:
        if row > temperatureRange[0]:
            print ""
        print " //", range(row, min(row + ( 10 * temperatureStep ), temperatureRange[1] + ( 1 * temperatureStep ) ), temperatureStep)
        print "    ",
    print str(int(temperatureCalibrationFunction(row))) + (" ",",")[row < temperatureRange[1]],
print ""
print "},"
