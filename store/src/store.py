import sys
import os
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import QDateTime, QDate, Qt, QRect,  pyqtSignal
from PyQt5.QtGui import QTextCharFormat, QColor, QFont, QPainter, QPixmap, QMouseEvent
import time

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../db")))

from db import DatabaseManager

# Load the UI files
loginClass = uic.loadUiType("../ui/login.ui")[0]
mainClass = uic.loadUiType("../ui/store.ui")[0]
robotClass = uic.loadUiType("../ui/robot_manage.ui")[0]
dailySalesClass = uic.loadUiType("../ui/daily_sales.ui")[0]

class Calendar(QCalendarWidget):
    dateClicked = pyqtSignal(QDate)

    def __init__(self, parent=None):
        super(Calendar, self).__init__(parent)
        self.salesRecords = {}
        
        self.clicked.connect(self.handleClick)

    def setDailyTotalSales(self, salesRecords):
        self.salesRecords = salesRecords
        self.updateCells()

    def paintCell(self, painter, rect, date):
        super(Calendar, self).paintCell(painter, rect, date)
        
        if date in self.salesRecords:
            painter.save()
            painter.setPen(QColor(0, 255, 0))
            painter.setFont(QFont("Arial", 8, QFont.Bold))
            textRect = QRect(rect.left(), rect.top() + 25, rect.width(), rect.height() - 20)
            painter.drawText(textRect, Qt.AlignCenter, self.salesRecords[date])
            painter.restore()
    
    def handleClick(self):
        selectedDate = self.selectedDate()
        self.dateClicked.emit(selectedDate) 

class LoginPage(QDialog, loginClass):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Login Page")
        
        self.loginBtn.clicked.connect(self.loginBtnClicked)


    def loginBtnClicked(self):
        self.dbManager = DatabaseManager(
            host="localhost",
            user="root",
            password="amr231218!",
            database="ArisTeam5"
        )
        self.dbManager.connect()

        self.mainWindow = MainPage(self.dbManager)
        self.mainWindow.show()
        self.close()
    
class MainPage(QMainWindow, mainClass):
    def __init__(self, dbManager):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Main Page")

        self.dbManager = dbManager 
        self.initWindow()

    def initWindow(self):
        self.robotManageBtn.clicked.connect(self.showRobotManagePage)

    
        self.stockTable.verticalHeader().setVisible(False) #일별 판매 기록 테이블의 왼쪽의 번호 비활성화
        self.stockTable.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

        self.robotManageWindow = RobotManagePage()
        

        self.calendarContainer = self.findChild(QWidget, "calendarContainer")  # 디자이너에서 설정한 이름 사용
        self.calendarLayout = QVBoxLayout(self.calendarContainer)
        self.calendarWidget = Calendar(self.calendarContainer)
        self.calendarLayout.addWidget(self.calendarWidget)
       
        self.calendarWidget.dateClicked.connect(self.showDailySalesPage)
        self.updateDailyTotalSales()
       

    def closeEvent(self, event):
        # 창이 닫힐 때 데이터베이스 연결 종료
        self.dbManager.disconnect()
        event.accept()  # 창을 닫음

    def onDateTimeChanged(self):
        self.calendarWidget.show()
        self.updateDailyTotalSales()

    def showRobotManagePage(self):
        self.robotManageWindow.show()

    def getStock(self):
        # self.stockTable.setRowCount(len(data))
        pass

    def getDailyTotalSales(self):
        year = str(self.calendarWidget.yearShown())
        month = str(self.calendarWidget.monthShown())
        results = self.dbManager.getDailyTotalSales(year, month)
        salesRecords = {}
        # print(results)
        for result in results:
            date = result[0]  # datetime.date 객체
            totalSales = result[1]  # Decimal 객체
            qdate = QDate(date.year, date.month, date.day)
            salesRecords[qdate] = f"{totalSales}원"
        return salesRecords


    def updateDailyTotalSales(self):
        salesRecords = self.getDailyTotalSales()
        # print(salesRecords)
        self.calendarWidget.setDailyTotalSales(salesRecords)

    def showDailySalesPage(self, date):
        self.dailySalesWindow = DailySalesPage(self.dbManager, date)
        self.dailySalesWindow.show()

class DailySalesPage(QDialog, dailySalesClass):
    def __init__(self, dbManager, date):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Daily Sales")
        self.date = date
        self.dbManager = dbManager
        self.initWindow()
    
    def initWindow(self):
        self.dailySalesRBtn.toggled.connect(self.showDailySalesPage) # 일별 판매 기록 버튼 연결
        self.menuSalesRBtn.toggled.connect(self.showMenuSales) # 메뉴별 판매 기록 버튼 연결
        self.dailySalesRBtn.setChecked(True) # 일별 판매 기록 버튼 활성화
        self.dailySalesTable.verticalHeader().setVisible(False) #일별 판매 기록 테이블의 왼쪽의 번호 비활성화
        self.showDailySalesPage() # 처음에 일별 판매 기록 테이블이 보이게 설정

    def showDailySalesPage(self):
        if self.dailySalesRBtn.isChecked():
            self.stackedWidget.setCurrentIndex(0)
            self.dailySalesTable.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
            self.updateDailySales()

    def updateTable(self, data):
        self.dailySalesTable.setRowCount(len(data))
        # self.dailySalesTable.setColumnCount(5)
        
        for row_num, row_data in enumerate(data):
            for col_num, item in enumerate(row_data):
                self.dailySalesTable.setItem(row_num, col_num, QTableWidgetItem(str(item)))

    def updateDailySales(self):
        date = self.date.toPyDate()  # QDateTime을 date 객체로 변환
        dailySales = self.dbManager.getDailySales(date)
        # Update the table with daily sales data
        self.updateTable(dailySales)

    def showMenuSales(self):
        if self.menuSalesRBtn.isChecked():
            self.stackedWidget.setCurrentIndex(1)


class RobotManagePage(QDialog, robotClass):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Robot Manage")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    loginWindow = LoginPage()
    loginWindow.show()
    sys.exit(app.exec_())