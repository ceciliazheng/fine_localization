'''
Direct Linear Transformation for homography.
'''
import math

import cv2
import numpy as np
from numpy.linalg import inv, norm


def average(org, corrs):
    averageOrg = np.zeros(2)
    averageCorr = np.zeros(2)
    for point in org:
        averageOrg[0] += point[0]
        averageOrg[1] += point[1]
    for point in corrs:
        averageCorr[0] += point[0]
        averageCorr[1] += point[1]
    averageOrg /= len(org)
    averageCorr /= len(corrs)

    return averageOrg, averageCorr


def scale(points, average):
    sumOfDist = 0.0
    for point in points:
        sumOfDist += norm(point - average)
    s = math.sqrt(2) * len(points) / (sumOfDist)
    return s


def matrixT(points, average):
    s = scale(points, average)
    tX = s * (-average[0])
    tY = s * (-average[1])
    T = np.float64([[s, 0, tX], [0, s, tY], [0, 0, 1]])
    return T


def normalize(points, T):
    normalizedPoints = np.zeros((len(points), 3))
    i = 0
    for point in points:
        normalizedPoints[i] = np.transpose(T @ point)
        i += 1
    return normalizedPoints


def matrixA(normalizedOrg, normalizedCorr):
    A = np.zeros((len(normalizedOrg) * 2, 9))
    i = 0
    for index in range(0, len(A), 2):
        A[index] = np.float64([0, 0, 0, -normalizedOrg[i][0], -normalizedOrg[i][1], -1, normalizedCorr[i]
                               [1] * normalizedOrg[i][0], normalizedCorr[i][1] * normalizedOrg[i][1], normalizedCorr[i][1]])
        A[index + 1] = np.float64([normalizedOrg[i][0], normalizedOrg[i][1], 1, 0, 0, 0, -normalizedCorr[i]
                                   [0] * normalizedOrg[i][0], -normalizedCorr[i][0] * normalizedOrg[i][1], -normalizedCorr[i][0]])
        i += 1
    return A


def computeH(src, dst):
    averageOrg, averageCorr = average(src, dst)
    orgT = matrixT(src, averageOrg)
    corrT = matrixT(dst, averageCorr)
    normalizedOrg = normalize(src, orgT)
    normalizedCorr = normalize(dst, corrT)
    A = matrixA(normalizedOrg, normalizedCorr)
    U, S, vT = cv2.SVDecomp(A)

    normalizedH = np.zeros((3, 3))
    i = 0
    for index in range(0, len(normalizedH)):
        normalizedH[index][0] = vT[-1][i]
        normalizedH[index][1] = vT[-1][i + 1]
        normalizedH[index][2] = vT[-1][i + 2]
        i += 3
    H = inv(corrT) @ normalizedH @ orgT
    return H


def main():
    print("Main!")
    src = [[0, 1], [200, 13]]
    dst = [[2, 23], [34, 90]]
    print(computeH((src, dst)))


if __name__ == '__main__':
    main()