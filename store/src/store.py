import sys
import os
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import QDateTime

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../db")))

from db import DatabaseManager

# Load the UI files
login_class = uic.loadUiType("../ui/login.ui")[0]
main_class = uic.loadUiType("../ui/store.ui")[0]


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

        self.dateTimeEdit.setDateTime(QDateTime.currentDateTime()) #dateTimeEdit 날짜를 현재 날짜로 설정
        self.dateTimeEdit.dateTimeChanged.connect(self.onDateTimeChanged) #dateTimeEdit의 날짜가 변경될 때 self.onDateTimeChnaged 함수 호출 

        self.dailySalesRBtn.toggled.connect(self.showDailySalesPage) # 일별 판매 기록 버튼 연결
        self.menuSalesRBtn.toggled.connect(self.showMenuSalesPage) # 메뉴별 판매 기록 버튼 연결
        self.stockRBtn.toggled.connect(self.showStockPage) # 재고 버튼 연결
        self.dailySalesRBtn.setChecked(True) # 일별 판매 기록 버튼 활성화
        self.showDailySalesPage() # 처음에 일별 판매 기록 테이블이 보이게 설정

        self.dailySalesTable.verticalHeader().setVisible(False) #일별 판매 기록 테이블의 왼쪽의 번호 비활성화
        self.stockTable.verticalHeader().setVisible(False) #일별 판매 기록 테이블의 왼쪽의 번호 비활성화


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

    def showStockPage(self):
        if self.stockRBtn.isChecked():
            self.stackedWidget.setCurrentIndex(2)
            self.stockTable.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

    def updateDailySales(self):
        date = self.dateTimeEdit.date().toPyDate()  # QDateTime을 date 객체로 변환
        daily_sales = self.db_manager.getDailySales(date)
        # Update the table with daily_sales data
        self.update_table(daily_sales)

    def update_table(self, data):
        self.dailySalesTable.setRowCount(len(data))
        self.dailySalesTable.setColumnCount(5)
        
        for row_num, row_data in enumerate(data):
            for col_num, item in enumerate(row_data):
                self.dailySalesTable.setItem(row_num, col_num, QTableWidgetItem(str(item)))


if __name__ == "__main__":
    app = QApplication(sys.argv)
    login_window = LoginPage()
    login_window.show()
    sys.exit(app.exec_())