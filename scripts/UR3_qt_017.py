#!/usr/bin/env python

#Damit Programm auf einem anderen PC funktionier, muessen die Pfade geaendert werden: Z.50, Z.218, Z.223, Z.246 und Z.354
#Fuer die Einbindung an den UR3, muessen die kommentierten Teile hierfuer in den Code eingefuegt werden (markiert durch die Rauten)

#Import aller benoetigten Bibliotheken
import sys
import math
from PyQt5.QtCore import Qt
from PyQt5 import QtGui
from PyQt5.QtWidgets import (QApplication, QCheckBox, QComboBox, QDateTimeEdit, QDial, 
QDialog, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit, QSlider, QSpinBox, 
QStyleFactory, QProgressBar, QPushButton, QRadioButton, QScrollBar, QSizePolicy, QTableWidget, 
QTabWidget, QTextEdit, QVBoxLayout, QWidget, QListWidget)
from PyQt5.QtGui import QIcon

import time

import urx  # urx is a python library to control the robots from Universal Robots. 
# if mising, try $ pip install urx
from urx.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper

#Setup fuer Roboter erstellen

rob = urx.Robot("192.168.0.9") ## IP anpassen ##############################################################################################################################################
rob.set_tcp((0, 0, 0.1, 0, 0, 0))
rob.set_payload(2, (0, 0, 0.1))
#robotiqgrip = Robotiq_Two_Finger_Gripper()
a = 0.2
v = 0.2
#l = 0.05

#Allgemeine Variablen die das System braucht
Gelenkzahl = 6
posen = {}
posen_zahlen = {}
posPublish = []
posUebergabe = []

Gelenk_Winkel = rob.getj(True)
print(Gelenk_Winkel)
Gelenk_Winkel_0 = int(Gelenk_Winkel[0]*180/math.pi)
Gelenk_Winkel_1 = int(Gelenk_Winkel[1]*180/math.pi)
Gelenk_Winkel_2 = int(Gelenk_Winkel[2]*180/math.pi)
Gelenk_Winkel_3 = int(Gelenk_Winkel[3]*180/math.pi)
Gelenk_Winkel_4 = int(Gelenk_Winkel[4]*180/math.pi)
Gelenk_Winkel_5 = int(Gelenk_Winkel[5]*180/math.pi)

#Oberste Struktur fuer die Tabs
class TabWidget(QDialog):
    def __init__(self, parent = None):
        super(TabWidget, self).__init__(parent)

        self.setWindowTitle('UR3 Kontollprogramm')
        self.setWindowIcon(QIcon("ur3schema.png")) #Hier wird unser Logo eingefuegt

        #Tabs instanzieren
        tabwidget = QTabWidget()
        tabwidget.addTab(PosenTab(), "Posen")
        tabwidget.addTab(PfadTab(), "Pfadplanung")

        #Layout instanzieren
        vbox = QVBoxLayout()
        vbox.addWidget(tabwidget)

        #Groesse des Fensters festlegen 
        self.resize(600,600)
        self.setLayout(vbox)

#Erster Tab zum Posen einstellen
class PosenTab(QWidget):
    def __init__(self, parent = None):
        super(PosenTab, self).__init__(parent)

        #Viergeteiltes Fester erstellen
        grid = QGridLayout()
        grid.addWidget(self.Groupol(), 0, 0)
        grid.addWidget(self.Groupul(), 1, 0)
        grid.addWidget(self.Groupor(), 0, 1)
        grid.addWidget(self.Groupur(), 1, 1)
        self.setLayout(grid)

    #Fenster mit Boxen fuer die Gelenkwinkel
    def Groupol(self):
        groupBoxol = QGroupBox("Motorwinkel")
        vbox = QVBoxLayout()

        #Fuer jedes Gelenk eine Anzeige erstellen und entsprechende Aktionen verbinden
        self.spinbox0 = QSpinBox(self)
        self.spinbox0.setRange(-360, 360)
        self.spinbox0.setValue(Gelenk_Winkel_0)
        self.spinbox0.valueChanged.connect(self.spin_changed)
        vbox.addWidget(self.spinbox0)
        self.spinbox1 = QSpinBox(self)
        self.spinbox1.setRange(-360, 360)
        self.spinbox1.setValue(Gelenk_Winkel_1)
        self.spinbox1.valueChanged.connect(self.spin_changed)
        vbox.addWidget(self.spinbox1)
        self.spinbox2 = QSpinBox(self)
        self.spinbox2.setRange(-360, 360)
        self.spinbox2.setValue(Gelenk_Winkel_2)
        self.spinbox2.valueChanged.connect(self.spin_changed)
        vbox.addWidget(self.spinbox2)
        self.spinbox3 = QSpinBox(self)
        self.spinbox3.setRange(-360, 360)
        self.spinbox3.setValue(Gelenk_Winkel_3)
        self.spinbox3.valueChanged.connect(self.spin_changed)
        vbox.addWidget(self.spinbox3)
        self.spinbox4 = QSpinBox(self)
        self.spinbox4.setRange(-360, 360)
        self.spinbox4.setValue(Gelenk_Winkel_4)
        self.spinbox4.valueChanged.connect(self.spin_changed)
        vbox.addWidget(self.spinbox4)
        self.spinbox5 = QSpinBox(self)
        self.spinbox5.setRange(-360, 360)
        self.spinbox5.setValue(Gelenk_Winkel_5)
        self.spinbox5.valueChanged.connect(self.spin_changed)
        vbox.addWidget(self.spinbox5)

        send = QPushButton('Senden', self)
        send.setToolTip("Aktuelle Winkel senden")
        send.clicked.connect(self.senden_winkel)
        vbox.addWidget(send)

        self.label = QLabel()
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setFont(QtGui.QFont("Sanserif", 15))
        vbox.addWidget(self.label)
        groupBoxol.setLayout(vbox)

        return groupBoxol

    def senden_winkel(self):
        global Gelenk_Winkel_0
        global Gelenk_Winkel_1
        global Gelenk_Winkel_2
        global Gelenk_Winkel_3
        global Gelenk_Winkel_4
        global Gelenk_Winkel_5
        rob.movej((Gelenk_Winkel_0*(math.pi/180), Gelenk_Winkel_1*(math.pi/180), Gelenk_Winkel_2*(math.pi/180), Gelenk_Winkel_3*(math.pi/180), Gelenk_Winkel_4*(math.pi/180), Gelenk_Winkel_5*(math.pi/180)),a,v,False) # Roboter ansteuern  ########################################################
        print("Winkel werden angefahren:", Gelenk_Winkel_0, Gelenk_Winkel_1, Gelenk_Winkel_2, Gelenk_Winkel_3, Gelenk_Winkel_4, Gelenk_Winkel_5)

    def spin_changed(self):
        #Gelenkwinkel muessen dem gesamten system zur verfuegung gestellt werden
        global Gelenk_Winkel_0
        global Gelenk_Winkel_1
        global Gelenk_Winkel_2
        global Gelenk_Winkel_3
        global Gelenk_Winkel_4
        global Gelenk_Winkel_5

        #Abfrage des Inhaltes der einzelnen Boxen
        spinValue0 = self.spinbox0.value()
        Gelenk_Winkel_0 = spinValue0
        print("Value0: ", Gelenk_Winkel_0)

        spinValue1 = self.spinbox1.value()
        Gelenk_Winkel_1 = spinValue1
        print("Value1: ", Gelenk_Winkel_1)

        spinValue2 = self.spinbox2.value()
        Gelenk_Winkel_2 = spinValue2
        print("Value2: ", Gelenk_Winkel_2)

        spinValue3 = self.spinbox3.value()
        Gelenk_Winkel_3 = spinValue3
        print("Value3: ", Gelenk_Winkel_3)

        spinValue4 = self.spinbox4.value()
        Gelenk_Winkel_4 = spinValue4
        print("Value4: ", Gelenk_Winkel_4)

        spinValue5 = self.spinbox5.value()
        Gelenk_Winkel_5 = spinValue5
        print("Value5: ", Gelenk_Winkel_5)

    # Fenster mit Box fuer die Bedienung des Greifers
    def Groupul(self):
        groupBoxul = QGroupBox("Greifer")

        #Zwei Buttons erstellen
        vbox = QVBoxLayout()
        gauf = QPushButton('Auf', self)
        gauf.setToolTip("Greifer oeffnen")
        gauf.clicked.connect(self.auf_click)
        vbox.addWidget(gauf)

        gzu = QPushButton('Zu', self)
        gzu.setToolTip("Greifer schliessen")
        gzu.clicked.connect(self.zu_click)
        vbox.addWidget(gzu)

        groupBoxul.setLayout(vbox)

        return groupBoxul


    #Funktionen die Ausgefuehrt werden wenn Button gedrueckt wird
    def auf_click(self):
        #robotiqgrip.open_gripper() # Roboter ansteuern  ####################################################################################################################################
        print("Greifer auf") 

    def zu_click(self):
        #robotiqgrip.close_gripper() # Roboter ansteuern  ###################################################################################################################################

        print("Greifer zu")
    #Fenster zum Posennamen angeben und abspeichern
    def Groupor(self):
        groupBoxor = QGroupBox("Posenbenennung und Speicherung")
        vbox = QVBoxLayout()

        #Textfeldeingabe
        eingabe = QLineEdit(self)
        eingabe.textChanged[str].connect(self.erfolgte_Eingabe)
        vbox.addWidget(eingabe)
        #Speicherbutton
        Speichern = QPushButton('Speichern', self)
        Speichern.setToolTip("Aktuelle Pose speichern")
        Speichern.clicked.connect(self.Abspeichern_senden)
        vbox.addWidget(Speichern)

        groupBoxor.setLayout(vbox)
        return groupBoxor
        pass
    
    #Eingegebene Buchstaben an globale Variable weiterleiten
    def erfolgte_Eingabe(self, text):
        global Speicher_Name
        Speicher_Name = text
        pass
    
    #Gelenkwinkel mit Namen in die .txt Datei schrieben
    def Abspeichern_senden(self):
        #Daten schreiben
        print("Speichere ab...")
        fileObject = open("Positions_Posen.txt", "a")
        fileObject.write("\n{}, {}, {}, {}, {}, {}, {}".format(Speicher_Name, Gelenk_Winkel_0, Gelenk_Winkel_1, Gelenk_Winkel_2, Gelenk_Winkel_3, Gelenk_Winkel_4, Gelenk_Winkel_5))
        fileObject.close()  

        #Inhalt der .txt erneut laden 
        fileObject02 = open("Positions_Posen.txt", 'r')
        for i in fileObject02:
            i = i.strip()
            zuordnung = i.split(", ")
            posen[zuordnung[0]] = zuordnung[1:] ### Zuordnung der Winkelwerte zu den Posennamen als Bibliothek 0: Posenname, 1-Ende: Winkelpositionen
        fileObject02.close()

        key_list = []  #### leere Liste fuer die Schluessel erstellen
        for key in posen:  ### Durchzaehlen des Bibliothek
            gelenkWinkel = []
            var = posen[key]  
            for i in range(0, Gelenkzahl):### durchzaehlen der Winkelwerte der zugehoerigen Pose
                Zahl = int(var[i]) ### Winkelwerte als Integer umschreiben
                gelenkWinkel.append(Zahl) 
                pass
            posen_zahlen[key] = gelenkWinkel[0:]
            key_list.append(key) ### Einschreiben der Winkelwerte zu dem dazugehoerigen Posennamen in oben erstellte Liste, dadurch Daten fuer das Programm lesbar
            pass
        pass

#Fenster mit Lister der gespeicherten Posen
    def Groupur(self):
        #Daten einlesen
        fileObject = open("Positions_Posen.txt", 'r')

        for i in fileObject: 
            i = i.strip()
            zuordnung = i.split(", ")
            posen[zuordnung[0]] = zuordnung[1:]
        fileObject.close()

        key_list = []
        for key in posen:
            gelenkWinkel = []
            var = posen[key]
            for i in range(0, Gelenkzahl):
                Zahl = int(var[i])
                gelenkWinkel.append(Zahl)
                pass
            posen_zahlen[key] = gelenkWinkel[0:]
            key_list.append(key)
            pass
        #Boxlayout
        groupBoxur = QGroupBox("Gespeicherte Roboterposen")
        vbox = QVBoxLayout()
        dropDown = QComboBox(self)
        dropDown.setStyleSheet("combobox-popup: 0;")
        for a in range (0, len(key_list)):   ## Liste wird in Dropdown geschrieben
            dropDown.addItem(key_list[a])
            pass
        
        dropDown.activated[str].connect(self.dropDown_Action) #Zahlen zur Kontrolle ausgeben
        vbox.addWidget(dropDown)

        senden = QPushButton('Senden', self)
        senden.setToolTip("Pose dem Roboter senden")
        senden.clicked.connect(lambda: self.Sende_an_Robo(dropDown.currentText()))
        vbox.addWidget(senden)
        
        groupBoxur.setLayout(vbox)

        return groupBoxur
        pass

    def Sende_an_Robo(self, pose):
        posDD = posen_zahlen[pose]
        print(posDD[0])
        rob.movej((posDD[0]*(math.pi/180), posDD[1]*(math.pi/180), posDD[2]*(math.pi/180), posDD[3]*(math.pi/180), posDD[4]*(math.pi/180), posDD[5]*(math.pi/180)), a, v, False) # Roboter ansteuern  #################################################################################
        print("Winkel werden angefahren:", posDD)
        pass

    def dropDown_Action(self, text):
        print("Kontrolle das die Auswahl im DropDown funktioniert:") 
        print(posen_zahlen[text])

#Zweiter Tab zum Choreo Erstellen
class PfadTab(QWidget):
    def __init__(self, parent = None):
        super(PfadTab, self).__init__(parent)

        #Posenspeicher- und Erklaerungstextfeld erstelle
        grid = QGridLayout()
        grid.addWidget(self.Pfadplan(), 0, 0)
        grid.addWidget(self.Posenspeicher(), 0, 1)
        self.setLayout(grid)

    #Abschicken der Choreo
    def Pfadplan(self):
        groupBoxPfadplan = QGroupBox("Pfadplanung")
        vbox = QVBoxLayout()

        self.label = QLabel("Die doppelt angeklickten Posen werden\ndem System uebergeben und gespeichert,\nklicken Sie 'Senden', um Choreografie\nabzufahren")
        vbox.addWidget(self.label)

        senden = QPushButton('Senden', self)
        senden.setToolTip("Choreografie dem Roboter senden")
        senden.clicked.connect(lambda: self.Senden_Choreo(posUebergabe))
        vbox.addWidget(senden)
        
        groupBoxPfadplan.setLayout(vbox)
        return groupBoxPfadplan
    
    #Choreo senden
    def Senden_Choreo(self, posUebergabe):
        global posPublish
        global posen
        a = 0
        for key in posUebergabe: 
            testvar = posen[key]
            posPublish.append(testvar)
            a = a+1
        for i in range (0, len(posUebergabe)):
            posG0 = posPublish[i][0]
            posG1 = posPublish[i][1]
            posG2 = posPublish[i][2]
            posG3 = posPublish[i][3]
            posG4 = posPublish[i][4]
            posG5 = posPublish[i][5]
            #rob.movej((posG0, posG1, posG2, posG3, posG4, posG5), a, v) # Roboter ansteuren ################################################################################################
            print(posUebergabe[i], "wird angefahren:", posG0, posG1, posG2, posG3, posG4, posG5)
            time.sleep(3)


    #Posenspeicher anzeigen
    def Posenspeicher(self):
        groupBoxPosenspeicher = QGroupBox("Zugriff Posenspeicher")
        vbox = QVBoxLayout()

        self.label = QLabel("Hier der Speicher der einzelnen Posen")

        #Daten aus der .txt lesen
        fileObject = open("Positions_Posen.txt", 'r')

        for i in fileObject:
            i = i.strip()
            zuordnung = i.split(", ")
            posen[zuordnung[0]] = zuordnung[1:]
        fileObject.close()

        key_list = []
        for key in posen:
            gelenkWinkel = []
            var = posen[key]
            for i in range(0, Gelenkzahl):
                Zahl = int(var[i])
                gelenkWinkel.append(Zahl)
                pass
            posen_zahlen[key] = gelenkWinkel[0:]
            key_list.append(key)
            pass
        ListePosen = QListWidget()
        ListePosen.insertItems(0, key_list)
        vbox.addWidget(self.label)
        vbox.addWidget(ListePosen)

        #Um Pose zu Uebergeben doppelt draufklicken
        ListePosen.itemDoubleClicked.connect(self.PoseAusgeben)
        
        groupBoxPosenspeicher.setLayout(vbox)

        return groupBoxPosenspeicher

    #Hinzugefuegte Posen speichern
    def PoseAusgeben(self, item):
        global posUebergabe
        zwischenVariable = item.text()
        zwischenVariable = str(zwischenVariable)
        #print(zwischenVariable)
        posUebergabe.append(zwischenVariable)
        print("Pose hinzugefuegt")
        #print(posUebergabe)
        self.Pfadplan()

if __name__ == "__main__":
    #Programm starten
    app = QApplication(sys.argv)
    tabwidget = TabWidget()
    tabwidget.show()
    app.exec_()
