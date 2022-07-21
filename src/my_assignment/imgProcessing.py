#!/usr/bin/env python

import numpy as np
import cv2

def getCoordiantes(image, lineParams):
    # slope, intercept = lineParams
    slope = lineParams[0]
    intercept = lineParams[1]
    y1 = image.shape[0]
    y2 = int(y1 * 0.6)
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    return np.array([x1, y1, x2, y2])

def averageLines(image, lines):
    leftLines = []
    rightLines = []

    if lines is None:
        return lines

    for line in lines:
        x1, y1, x2, y2 = line.reshape(4)
        params = np.polyfit((x1, x2), (y1, y2), 1)
        slope = params[0]
        intercept = params[1]

        if slope < 0:
            leftLines.append((slope, intercept))
        else:
            rightLines.append((slope, intercept))

    finalLines = np.empty((0, 4), int)

    if len(leftLines) > 0:
        leftFitAverage = np.average(leftLines, axis=0)
        leftArr = getCoordiantes(image, leftFitAverage)
        finalLines = np.append(finalLines, [leftArr], axis=0)

    if len(rightLines) > 0:
        rightFitAverage = np.average(rightLines, axis=0)
        rightArr = getCoordiantes(image, rightFitAverage)
        finalLines = np.append(finalLines, [rightArr], axis=0)

    return finalLines


def drawLinesInImg(rgb_image, lines, weighted):
    copiedImg = np.copy(rgb_image)

    if lines is None:
        return copiedImg

    if weighted:
        blankImg = np.zeros_like(rgb_image)
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            cv2.line(blankImg, (x1, y1), (x2, y2), (0, 255, 0), 10)

        return cv2.addWeighted(copiedImg, 0.8, blankImg, 1, 0.0)
    else:
        for line in lines:
            x1, x2, y1, y2 = line.reshape(4)
            cv2.line(copiedImg, (x1, x2), (y1, y2), (0, 255, 0), 10)

        return copiedImg


def cannyEdgeDetection(rgb_image, minVal, maxVal, reduceNoise):
    grayImg = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
    if reduceNoise:
        #Reduce noise in grayscale img
        blur = cv2.GaussianBlur(grayImg, (5, 5), 0)
        edgeDetecImg = cv2.Canny(blur, minVal, maxVal)
    else:
        edgeDetecImg = cv2.Canny(grayImg, minVal, maxVal)

    return edgeDetecImg



def applyMask(rgb_image, vertices):
    # Create with the same size of the image
    mask = np.zeros_like(rgb_image)
    # Color for the mask. Must be an array with 3 elements for RGB ((255, 255, 255)) or single element for grayscale images
    maskColor = 255
    # Draw a polygon into the mask given the vertices
    cv2.fillPoly(mask, pts=[vertices], color=maskColor)
    # Applied it to the image
    maskedImg = cv2.bitwise_and(rgb_image, mask)
    return maskedImg


def detectLine(frame):

    height = frame.shape[0]
    width = frame.shape[1]

    # CANNY EDGE DETECTOR
    applyFilter = True
    cannyMinVal = 50
    cannyMaxVal = 100
    cannyImg = cannyEdgeDetection(frame, cannyMinVal, cannyMaxVal, applyFilter)

    # APPLY MASK TO THE IMAGE
    maskImg = True
    if maskImg:
        regionOfInterestVertices = np.array([[0, height], [width / 2, height / 2], [width, height]])
        # regionOfInterestVertices = np.array([[200, height], [550, 250], [1100, height]])
        croppedImg = applyMask(cannyImg, regionOfInterestVertices)
    else:
        croppedImg = cannyImg

    # HOUGH TRANSFORM
    linesImg = cv2.HoughLinesP(croppedImg,
        rho = 2, # 1
        theta = np.pi / 180, # np.pi / 180
        threshold = 100, # Min number of votes
        lines = np.array([]),
        minLineLength = 40, # 80
        maxLineGap = 25) # 10

    # AVERAGE LINES AND DRAW THEM
    applyAvg = True
    addWeighted = True
    if applyAvg:
        avgLin = averageLines(frame, linesImg)
        endImg = drawLinesInImg(frame, avgLin, addWeighted)
    else:
        endImg = drawLinesInImg(frame, linesImg, addWeighted)

    return endImg


