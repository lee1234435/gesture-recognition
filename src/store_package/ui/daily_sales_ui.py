# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '/home/addinedu/amr_ws/aris_team5/Aris_Team5/store/ui/daily_sales.ui'
#
# Created by: PyQt5 UI code generator 5.15.9
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(703, 478)
        self.stackedWidget = QtWidgets.QStackedWidget(Dialog)
        self.stackedWidget.setGeometry(QtCore.QRect(30, 40, 531, 381))
        self.stackedWidget.setObjectName("stackedWidget")
        self.page = QtWidgets.QWidget()
        self.page.setObjectName("page")
        self.dailySalesTable = QtWidgets.QTableWidget(self.page)
        self.dailySalesTable.setGeometry(QtCore.QRect(10, 40, 511, 331))
        self.dailySalesTable.setObjectName("dailySalesTable")
        self.dailySalesTable.setColumnCount(6)
        self.dailySalesTable.setRowCount(0)
        item = QtWidgets.QTableWidgetItem()
        self.dailySalesTable.setHorizontalHeaderItem(0, item)
        item = QtWidgets.QTableWidgetItem()
        self.dailySalesTable.setHorizontalHeaderItem(1, item)
        item = QtWidgets.QTableWidgetItem()
        self.dailySalesTable.setHorizontalHeaderItem(2, item)
        item = QtWidgets.QTableWidgetItem()
        self.dailySalesTable.setHorizontalHeaderItem(3, item)
        item = QtWidgets.QTableWidgetItem()
        self.dailySalesTable.setHorizontalHeaderItem(4, item)
        item = QtWidgets.QTableWidgetItem()
        self.dailySalesTable.setHorizontalHeaderItem(5, item)
        self.totalSalesLine = QtWidgets.QLineEdit(self.page)
        self.totalSalesLine.setGeometry(QtCore.QRect(330, 10, 191, 25))
        self.totalSalesLine.setReadOnly(True)
        self.totalSalesLine.setObjectName("totalSalesLine")
        self.totalSalesLabel = QtWidgets.QLabel(self.page)
        self.totalSalesLabel.setGeometry(QtCore.QRect(260, 10, 67, 17))
        self.totalSalesLabel.setObjectName("totalSalesLabel")
        self.stackedWidget.addWidget(self.page)
        self.page_2 = QtWidgets.QWidget()
        self.page_2.setObjectName("page_2")
        self.menuSalesGraph = QtWidgets.QLabel(self.page_2)
        self.menuSalesGraph.setGeometry(QtCore.QRect(20, 10, 481, 331))
        self.menuSalesGraph.setText("")
        self.menuSalesGraph.setObjectName("menuSalesGraph")
        self.stackedWidget.addWidget(self.page_2)
        self.page_3 = QtWidgets.QWidget()
        self.page_3.setObjectName("page_3")
        self.timeSalesGraph = QtWidgets.QLabel(self.page_3)
        self.timeSalesGraph.setGeometry(QtCore.QRect(50, 50, 431, 281))
        self.timeSalesGraph.setText("")
        self.timeSalesGraph.setObjectName("timeSalesGraph")
        self.stackedWidget.addWidget(self.page_3)
        self.dateLabel = QtWidgets.QLabel(Dialog)
        self.dateLabel.setGeometry(QtCore.QRect(40, 20, 171, 17))
        self.dateLabel.setObjectName("dateLabel")
        self.layoutWidget = QtWidgets.QWidget(Dialog)
        self.layoutWidget.setGeometry(QtCore.QRect(270, 10, 246, 31))
        self.layoutWidget.setObjectName("layoutWidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.layoutWidget)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.dailySalesRBtn = QtWidgets.QRadioButton(self.layoutWidget)
        self.dailySalesRBtn.setObjectName("dailySalesRBtn")
        self.horizontalLayout.addWidget(self.dailySalesRBtn)
        self.menuSalesRBtn = QtWidgets.QRadioButton(self.layoutWidget)
        self.menuSalesRBtn.setObjectName("menuSalesRBtn")
        self.horizontalLayout.addWidget(self.menuSalesRBtn)
        self.timeSalesRBtn = QtWidgets.QRadioButton(self.layoutWidget)
        self.timeSalesRBtn.setObjectName("timeSalesRBtn")
        self.horizontalLayout.addWidget(self.timeSalesRBtn)

        self.retranslateUi(Dialog)
        self.stackedWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))
        item = self.dailySalesTable.horizontalHeaderItem(0)
        item.setText(_translate("Dialog", "주문 번호"))
        item = self.dailySalesTable.horizontalHeaderItem(1)
        item.setText(_translate("Dialog", "주문 시간"))
        item = self.dailySalesTable.horizontalHeaderItem(2)
        item.setText(_translate("Dialog", "메뉴"))
        item = self.dailySalesTable.horizontalHeaderItem(3)
        item.setText(_translate("Dialog", "토핑"))
        item = self.dailySalesTable.horizontalHeaderItem(4)
        item.setText(_translate("Dialog", "수량"))
        item = self.dailySalesTable.horizontalHeaderItem(5)
        item.setText(_translate("Dialog", "가격"))
        self.totalSalesLabel.setText(_translate("Dialog", "총 매출 : "))
        self.dateLabel.setText(_translate("Dialog", "날짜"))
        self.dailySalesRBtn.setText(_translate("Dialog", "판매 기록"))
        self.menuSalesRBtn.setText(_translate("Dialog", "메뉴별"))
        self.timeSalesRBtn.setText(_translate("Dialog", "시간대별"))
