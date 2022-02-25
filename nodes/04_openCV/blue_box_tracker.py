#!/usr/bin/env python
# blue_box_tracker.py
# V2 ohne TCP-Goal und IK
# ------------------------------------
# edited WHS, OJ , 25.2.2022
# -------------------------------------
# Pick and Place
# in Python mit der move_group_api
# und Kollsionsverhütung
# Erkennt gruene Rechtecke und zeichnet
# ein umschliessendes Rechteck und einen Ausrichtungspfeil
# Mittelpunkt und Winkel in Rad
# werden ausgegeben

from collections import deque
import numpy as np
import argparse
import imutils
import cv2

# Blau-Such-Parameter
# mit color_picker ermitteln
# python color_picker.py -i left0000.jpg
blueLower = (112, 31, 163)
blueUpper = (147, 163, 244)

# Bild aus Datei lesen
# '~' funktioniert hier nicht
frame = cv2.imread('/home/oj/Bilder/left0000.jpg')
cv2.imshow("Kinect Bild", frame)
#cv2.waitKey(0)

# Masken anfertigen
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
mask = cv2.inRange(hsv, blueLower, blueUpper)
mask = cv2.erode(mask, None, iterations=2)
mask = cv2.dilate(mask, None, iterations=2)
cv2.imshow("Mask", mask)
#cv2.waitKey(0)

# Konturen finden
cnts = cv2.findContours(mask.copy(),
                        cv2.RETR_EXTERNAL,
                        cv2.CHAIN_APPROX_SIMPLE)[-2]
center = None
# print("gefundene Konturen", cnts)

if len(cnts) > 0:
    c = max(cnts, key=cv2.contourArea)
    # Erstellen umschiessendes Rechteck
    rect = cv2.minAreaRect(c)

    # Erstellen einer Punktwolke aus dem Rechteck
    #  und umwandeln der Liste in intp
    box = cv2.boxPoints(rect)
    box = np.intp(box)

    # Ermitteln des Mittelpunktes in x,y
    ((x, y), radius) = cv2.minEnclosingCircle(c)
    print("Radius", radius)
    if radius > 10:
        ax = int(x)
        ay = int(y)

        # Lesen des Winkels aus dem Rechteck, umwandeln in Rad
        phi = rect[2]*np.pi/180

        # Erstellen der Zielpunkte des Pfeils
        x2 = int(ax + 100 * np.cos(phi))
        y2 = int(ay + 100 * np.sin(phi))

        # Text, Rechteck und Richtungspfeil ins Bild einfuegen
        cv2.putText(frame, "Koordinate x:" + str(ax)
                    + " y:" + str(ay), (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))
        cv2.putText(frame, "Rotation: " + str(phi),
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))
        frame = cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)
        frame = cv2.arrowedLine(frame, (ax, ay), (x2, y2), (255, 255, 255))

    print(" Blue Box gefunden an Position ", x, y)
    #cv2.imshow("Mask", mask)
    cv2.imshow("Frame", frame)
    cv2.waitKey(0)

