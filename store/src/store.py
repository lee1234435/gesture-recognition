import sys
import os
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import QDateTime, QDate
from PyQt5.QtGui import QTextCharFormat, QColor, QFont

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../db")))

from db import DatabaseManager

# Load the UI files
login_class = uic.loadUiType("../ui/login.ui")[0]
main_class = uic.loadUiType("../ui/store.ui")[0]
robot_class = uic.loadUiType("../ui/robot_manage.ui")[0]

class CustomCalendarWidget(QCalendarWidget):
    def __init__(self, parent=None):
        super(CustomCalendarWidget, self).__init__(parent)
        self.sales_data = {}

    def setSalesData(self, sales_data):
        self.sales_data = sales_data
        self.updateCells()

    def updateCells(self):
        for date, sales in self.sales_data.items():
            print(date, sales)
            date_format = QTextCharFormat()
            date_format.setFontWeight(QFont.Bold)
            date_format.setForeground(QColor("blue"))
            date_format.setToolTip(f"Sales: {sales}")
            
            # Add sales text to the date cell
            self.setDateTextFormat(QDate.fromString(date, "yyyy-MM-dd"), date_format)

class LoginPage(QDialog, login_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Login Page")
        
        
        self.loginBtn.clicked.connect(self.loginBtn_clicked)


    def loginBtn_clicked(self):
        self.db_manager = DatabaseManager(
            host="localhost",
            user="root",
            password="amr231218!",
            database="ArisTeam5"
        )
        self.db_manager.connect()

        self.second_window = MainPage(self.db_manager)
        self.second_window.show()
        self.close()

class MainPage(QMainWindow, main_class):
    def __init__(self, db_manager):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Main Page")

        self.db_manager = db_manager 
        self.initUI()

    def initUI(self):
        self.dateTimeEdit.setDateTime(QDateTime.currentDateTime()) #dateTimeEdit 날짜를 현재 날짜로 설정
        self.dateTimeEdit.dateTimeChanged.connect(self.onDateTimeChanged) #dateTimeEdit의 날짜가 변경될 때 self.onDateTimeChnaged 함수 호출 

        self.dailySalesRBtn.toggled.connect(self.showDailySalesPage) # 일별 판매 기록 버튼 연결
        self.menuSalesRBtn.toggled.connect(self.showMenuSalesPage) # 메뉴별 판매 기록 버튼 연결
        self.salesGraphRBtn.toggled.connect(self.showSalesGraphPage) # 재고 버튼 연결
        self.dailySalesRBtn.setChecked(True) # 일별 판매 기록 버튼 활성화
        self.showDailySalesPage() # 처음에 일별 판매 기록 테이블이 보이게 설정

        self.robotManageBtn.clicked.connect(self.showRobotManagePage)

        self.dailySalesTable.verticalHeader().setVisible(False) #일별 판매 기록 테이블의 왼쪽의 번호 비활성화
        self.stockTable.verticalHeader().setVisible(False) #일별 판매 기록 테이블의 왼쪽의 번호 비활성화
        
        self.stockTable.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

        self.customCalendarWidget = CustomCalendarWidget(self.calendarWidget)
        self.third_window = RobotManagePage()

        self.customCalendarWidget.currentPageChanged.connect(self.onCalendarPageChanged)

    def closeEvent(self, event):
        # 창이 닫힐 때 데이터베이스 연결 종료
        self.db_manager.disconnect()
        event.accept()  # 창을 닫음

    def onDateTimeChanged(self):
        self.updateDailySales()

    def showDailySalesPage(self):
        if self.dailySalesRBtn.isChecked():
            self.stackedWidget.setCurrentIndex(0)
            self.dailySalesTable.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
            self.updateDailySales()

    def showMenuSalesPage(self):
        if self.menuSalesRBtn.isChecked():
            self.stackedWidget.setCurrentIndex(1)

    def showSalesGraphPage(self):
        if self.salesGraphRBtn.isChecked():
            self.stackedWidget.setCurrentIndex(2)
            self.update_calendar_sales()
    def onCalendarPageChanged(self, year, month):
        self.update_calendar_sales()

    def showRobotManagePage(self):
        self.third_window.show()

    def updateDailySales(self):
        date = self.dateTimeEdit.date().toPyDate()  # QDateTime을 date 객체로 변환
        daily_sales = self.db_manager.getDailySales(date)
        # Update the table with daily_sales data
        self.updateTable(daily_sales)

    def updateTable(self, data):
        self.dailySalesTable.setRowCount(len(data))
        # self.dailySalesTable.setColumnCount(5)
        
        for row_num, row_data in enumerate(data):
            for col_num, item in enumerate(row_data):
                self.dailySalesTable.setItem(row_num, col_num, QTableWidgetItem(str(item)))

    def getStock(self):
        # self.stockTable.setRowCount(len(data))
        pass

    def update_calendar_sales(self):
        year = self.customCalendarWidget.yearShown()
        month = self.customCalendarWidget.monthShown()
        sales_data = self.db_manager.getDaillySales(year, month)
        sales_dict = {record[0].strftime("%Y-%m-%d"): record[1] for record in sales_data}
        print(sales_dict)
        self.customCalendarWidget.setSalesData(sales_dict)

class RobotManagePage(QDialog, robot_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Robot Manage")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    login_window = LoginPage()
    login_window.show()
    sys.exit(app.exec_())