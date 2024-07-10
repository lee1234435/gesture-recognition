import sys
import os
import matplotlib.pyplot as plt
import koreanize_matplotlib
from matplotlib.ticker import MaxNLocator
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import QDateTime, QDate, Qt, QRect,  pyqtSignal
from PyQt5.QtGui import QTextCharFormat, QColor, QFont, QPainter, QPixmap, QImage
import time
import socket
import threading
import io

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
        self.initWindow()


    def initWindow(self):
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
        self.initWindow()        
        self.sock = None

    def initWindow(self):
        self.setupUi(self)
        self.setWindowTitle("Login")
        self.loginBtn.clicked.connect(self.loginBtnClicked)

    def loginBtnClicked(self):

        host = self.hostLine.text()
        port = int(self.portLine.text())
        
        if self.connectToServer(host, port):
            self.dbManager = DatabaseManager(
                host="localhost",
                user="root",
                password="amr231218!",
                database="ArisTeam5"
            )
            self.dbManager.connect()

            self.mainWindow = MainPage(self.dbManager, self.sock)
            self.mainWindow.show()
            self.close()
        else:
            QMessageBox.critical(self, "Connection Failed", "Could not connect to the server. Please check the host and port.")
        
    def connectToServer(self, host, port):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((host, port))
            print(f"Connected to server at {host}:{port}")
            return True
        except socket.error as e:
            print(f"Connection to server failed : {e}")
            return False


class MainPage(QMainWindow, mainClass):
    def __init__(self, dbManager, sock):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Main")

        self.dbManager = dbManager
        self.sock = sock
        self.initWindow()

    def initWindow(self):
        self.robotManageWindow = RobotManagePage(self.sock)

        self.robotManageBtn.clicked.connect(self.showRobotManagePage)
        self.stockTable.verticalHeader().setVisible(False) #일별 판매 기록 테이블의 왼쪽의 번호 비활성화
        self.stockTable.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.calendarContainer = self.findChild(QWidget, "calendarContainer")  # 디자이너에서 설정한 이름 사용
        self.calendarLayout = QVBoxLayout(self.calendarContainer)
        self.calendarWidget = Calendar(self.calendarContainer)
        self.calendarLayout.addWidget(self.calendarWidget)
        self.calendarWidget.dateClicked.connect(self.showDailySalesPage)
        self.updateData()

        self.receiveThread = threading.Thread(target= self.recieveMessages)
        self.receiveThread.daemon = True
        self.receiveThread.start()

    def sendMessage(self):
        message = "test"
        if self.sock:
            self.sock.sendall(message.endcode())

    def recieveMessages(self):
        while True:
            data = self.sock.recv(1024)
            if not data:
                break
            print(f"Received : {data.decode()}")

    def updateData(self):
        self.updateDailyTotalSales()
        self.updateStock()
        self.updateMonthTotalSales()
    
    def closeEvent(self, event):
        # 창이 닫힐 때 데이터베이스 연결 종료
        self.dbManager.disconnect()
        event.accept()  # 창을 닫음

    def updateMonthTotalSales(self):
        year = str(self.calendarWidget.yearShown())
        month = str(self.calendarWidget.monthShown())
        result = self.dbManager.getMonthTotalSales(year, month)
        monthTotalSales = str(result[0][0])
        self.totalSalesLine.setText(f"{monthTotalSales}원")
        self.totalSalesLine.setAlignment(Qt.AlignRight)

    def updateDailyTotalSales(self):
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
        self.calendarWidget.setDailyTotalSales(salesRecords)

    def updateStock(self):
        data = self.dbManager.getStock()
        self.stockTable.setRowCount(len(data))
        for row_num, row_data in enumerate(data):
            for col_num, item in enumerate(row_data):
                if col_num == 1:
                    formatted_item = f"{int(item)}개"
                    table_item = QTableWidgetItem(formatted_item)
                else:
                    table_item = QTableWidgetItem(str(item))
                table_item.setTextAlignment(Qt.AlignCenter)  # 가운데 정렬
                self.stockTable.setItem(row_num, col_num, table_item)

    def showDailySalesPage(self, date):
        self.dailySalesWindow = DailySalesPage(self.dbManager, date)
        self.dailySalesWindow.show()

    def showRobotManagePage(self):
        self.robotManageWindow.show()

class DailySalesPage(QDialog, dailySalesClass):
    def __init__(self, dbManager, date):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Daily Sales")
        self.date = date
        self.dbManager = dbManager
        self.initWindow()
    
    def initWindow(self):
        self.dailySalesRBtn.toggled.connect(self.showDailySales) # 일별 판매 기록 버튼 연결
        self.menuSalesRBtn.toggled.connect(self.showMenuSales) # 메뉴별 판매 기록 버튼 연결
        self.dailySalesRBtn.setChecked(True) # 일별 판매 기록 버튼 활성화
        self.dailySalesTable.verticalHeader().setVisible(False) #일별 판매 기록 테이블의 왼쪽의 번호 비활성화
        self.showDailySales() # 처음에 일별 판매 기록 테이블이 보이게 설정
        self.updateSelectedDateLabel(self.date)

    def updateSelectedDateLabel(self, date):
        self.dateLabel.setText(date.toString('yyyy년 MM월 dd일'))

    def updateTable(self, data):
        self.dailySalesTable.setRowCount(len(data))
        totalSales = 0
        for row_num, row_data in enumerate(data):
            for col_num, item in enumerate(row_data):
                table_item = QTableWidgetItem(str(item))
                table_item.setTextAlignment(Qt.AlignCenter)  # 가운데 정렬
                self.dailySalesTable.setItem(row_num, col_num, table_item)
                if col_num == 4:
                    totalSales += int(item)
        
        self.updateTotalSales(totalSales)

    def updateTotalSales(self, totalSales):
        self.totalSalesLine.setText(f"{str(totalSales)}원")
        self.totalSalesLine.setAlignment(Qt.AlignRight)

    def updateDailySales(self):
        date = self.date.toPyDate()  # QDateTime을 date 객체로 변환
        dailySales = self.dbManager.getDailySales(date)
        # Update the table with daily sales data
        self.updateTable(dailySales)

    def showMenuSales(self):
        if self.menuSalesRBtn.isChecked():
            self.stackedWidget.setCurrentIndex(1)
            self.drawBarGraph()

    def showDailySales(self):
        if self.dailySalesRBtn.isChecked():
            self.stackedWidget.setCurrentIndex(0)
            self.dailySalesTable.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
            self.updateDailySales()
    
    def drawBarGraph(self):
        date = self.date.toString('yyyy-MM-dd')  # QDate를 문자열로 변환
        menuSales = self.dbManager.getMenuSales(date)

        # 전체 메뉴 목록 (예시)
        all_menu_items = ['딸기', '바나나', '초코', '아포카토']

        # 모든 메뉴에 대해 판매량을 0으로 초기화
        sales_dict = {item: 0 for item in all_menu_items}

        # 판매된 메뉴의 판매량을 업데이트
        for item, quantity in menuSales:
            sales_dict[item] = quantity

        menuItems = list(sales_dict.keys())
        sales = list(sales_dict.values())


        width, height = self.menuSalesGraph.width() / 100, self.menuSalesGraph.height() / 100
        # Matplotlib을 사용하여 그래프 그리기
        fig, ax = plt.subplots(figsize=(width, height))
        fig.subplots_adjust(top=0.85, bottom=0.15)  # 그래프 여백 조정
        ax.bar(menuItems, sales, color='skyblue')
        ax.set_xlabel('메뉴')
        ax.set_ylabel('판매량')
        ax.set_title('메뉴별 일일 판매량')

        ax.yaxis.set_major_locator(MaxNLocator(integer=True))

        # Matplotlib Figure를 QPixmap으로 변환
        buf = io.BytesIO()
        fig.savefig(buf, format='png', bbox_inches='tight', pad_inches=0.1)
        buf.seek(0)
        img = QImage.fromData(buf.getvalue())
        pixmap = QPixmap.fromImage(img)

        # QLabel에 QPixmap 설정
        self.menuSalesGraph.setPixmap(pixmap)
        plt.close(fig)  # Matplotlib figure 닫기





class RobotManagePage(QDialog, robotClass):
    def __init__(self, sock):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Robot Manage")

        self.sock = sock

if __name__ == "__main__":
    app = QApplication(sys.argv)
    loginWindow = LoginPage()
    loginWindow.show()
    sys.exit(app.exec_())