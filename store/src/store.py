import sys
import os
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import QDateTime, QDate, Qt, QRect,  pyqtSignal
from PyQt5.QtGui import QTextCharFormat, QColor, QFont, QPainter, QPixmap, QImage, QPen
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
            painter.setPen(QColor(0, 100, 0))
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
        self.setSpinBoxesEnabled(False)
        self.initializeCalendar()

        self.modifyBtn.clicked.connect(self.modifyBtnClicked)

        self.robotManageBtn.clicked.connect(self.showRobotManagePage)
        self.updateData()
        self.startReceiveThread()


    def startReceiveThread(self):
        self.receiveThread = threading.Thread(target= self.recieveMessages)
        self.receiveThread.daemon = True
        self.receiveThread.start()


    def initializeCalendar(self):
        self.calendarContainer = self.findChild(QWidget, "calendarContainer")  # 디자이너에서 설정한 이름 사용
        self.calendarLayout = QVBoxLayout(self.calendarContainer)
        self.calendarWidget = Calendar(self.calendarContainer)
        self.calendarLayout.addWidget(self.calendarWidget)
        self.calendarWidget.dateClicked.connect(self.showDailySalesPage)

    def setSpinBoxesEnabled(self, enabled):
        spinBoxes = [
            self.strawberrySpinBox,
            self.bananaSpinBox,
            self.chocolateSpinBox,
            self.affogatoSpinBox,
            self.toppingASpinBox,
            self.toppingBSpinBox,
            self.toppingCSpinBox
        ]
        for spinBox in spinBoxes:
            spinBox.setEnabled(enabled)

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
        monthTotalSales = result[0][0]
        self.totalSalesLine.setText(f"{monthTotalSales:,}원")
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
            salesRecords[qdate] = f"{totalSales:,}원"
        self.calendarWidget.setDailyTotalSales(salesRecords)

    def updateStock(self):
        data = self.dbManager.getStock()

        spinBoxMapping = {
            "딸기" : self.strawberrySpinBox,
            "바나나" : self.bananaSpinBox,
            "초코" : self.chocolateSpinBox,
            "아포가토" : self.affogatoSpinBox,
            "토핑A" : self.toppingASpinBox,
            "토핑B" : self.toppingBSpinBox,
            "토핑C" : self.toppingCSpinBox,
        }

        for itemName, itemStock in data:
            if itemName in spinBoxMapping:
                spinBoxMapping[itemName].setValue(itemStock)

        # for row_data in data:
        #     item_name = row_data[0]
        #     item_stock = row_data[1]
        #     if item_name == "딸기":
        #         self.strawberrySpinBox.setValue(item_stock)
        #     elif item_name == "바나나":
        #         self.bananaSpinBox.setValue(item_stock)
        #     elif item_name == "초코":
        #         self.chocolateSpinBox.setValue(item_stock)
        #     elif item_name == "아포가토":
        #         self.affogatoSpinBox.setValue(item_stock)
        #     elif item_name == "토핑A":
        #         self.toppingASpinBox.setValue(item_stock)
        #     elif item_name == "토핑B":
        #         self.toppingBSpinBox.setValue(item_stock)
        #     elif item_name == "토핑C":
        #         self.toppingCSpinBox.setValue(item_stock)

    def modifyBtnClicked(self):
        text = self.modifyBtn.text()
        if text == "수정":
            self.setSpinBoxesEnabled(True)
            self.modifyBtn.setText("저장")
        elif text == "저장":
            self.modifyStock()
            self.setSpinBoxesEnabled(True)
            self.modifyBtn.setText("수정")
    
    def modifyStock(self):
        strawberryStock = self.strawberrySpinBox.value()
        bananaStock = self.bananaSpinBox.value()
        chocolateStock = self.chocolateSpinBox.value()
        affogatoStock = self.affogatoSpinBox.value()
        menuData = [
            ("딸기", strawberryStock),
            ("바나나", bananaStock),
            ("초코", chocolateStock),
            ("아포가토", affogatoStock)
        ]
        toppingAStock = self.toppingASpinBox.value()
        toppingBStock = self.toppingBSpinBox.value()
        toppingCStock = self.toppingCSpinBox.value()
        toppingData = [
            ("토핑A", toppingAStock),
            ("토핑B", toppingBStock),
            ("토핑C", toppingCStock)
        ]
        self.dbManager.updateStock(menuData,toppingData)

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
        self.timeSalesRBtn.toggled.connect(self.showTimeSales)
        self.dailySalesRBtn.setChecked(True) # 일별 판매 기록 버튼 활성화
        self.dailySalesTable.verticalHeader().setVisible(False) #일별 판매 기록 테이블의 왼쪽의 번호 비활성화
        self.showDailySales() # 처음에 일별 판매 기록 테이블이 보이게 설정
        self.updateSelectedDateLabel(self.date)

    def updateSelectedDateLabel(self, date):
        self.dateLabel.setText(date.toString('yyyy년 MM월 dd일'))

    def updateTable(self, data):
        self.dailySalesTable.setRowCount(len(data))
        totalSales = sum(row_data[5] for row_data in data)
        for row_num, row_data in enumerate(data):
            for col_num, item in enumerate(row_data):
                table_item = QTableWidgetItem(str(item))
                table_item.setTextAlignment(Qt.AlignCenter)  # 가운데 정렬
                self.dailySalesTable.setItem(row_num, col_num, table_item)
        self.updateTotalSales(totalSales)

    def updateTotalSales(self, totalSales):
        self.totalSalesLine.setText(f"{totalSales:,}원")
        self.totalSalesLine.setAlignment(Qt.AlignRight)

    def updateDailySales(self):
        date = self.date.toPyDate()  # QDateTime을 date 객체로 변환
        dailySales = self.dbManager.getDailySales(date)
        # Update the table with daily sales data
        self.updateTable(dailySales)

    def showDailySales(self):
        if self.dailySalesRBtn.isChecked():
            self.stackedWidget.setCurrentIndex(0)
            self.dailySalesTable.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
            self.updateDailySales()

    def showMenuSales(self):
        if self.menuSalesRBtn.isChecked():
            self.stackedWidget.setCurrentIndex(1)
            self.drawBarGraph()

    def showTimeSales(self):
        if self.timeSalesRBtn.isChecked():
            self.stackedWidget.setCurrentIndex(2)
    
    def drawBarGraph(self):
        date = self.date.toString('yyyy-MM-dd')  # QDate를 문자열로 변환
        menuSales = self.dbManager.getMenuSales(date)

        # 전체 메뉴 목록 (예시)
        all_menu_items = ['딸기', '바나나', '초코', '아포가토']

        # 모든 메뉴에 대해 판매량을 0으로 초기화
        sales_dict = {item: 0 for item in all_menu_items}

        # 판매된 메뉴의 판매량을 업데이트
        for item, quantity in menuSales:
            sales_dict[item] = quantity

        menuItems = list(sales_dict.keys())
        sales = list(sales_dict.values())

        # QImage 생성
        width, height = self.menuSalesGraph.width(), self.menuSalesGraph.height()
        image = QImage(width, height, QImage.Format_ARGB32)
        image.fill(Qt.white)

        # QPainter로 QImage에 그리기
        painter = QPainter(image)
        painter.setRenderHint(QPainter.Antialiasing)

        # 그래프 영역 정의
        margin = 50
        graph_width = width - 2 * margin
        graph_height = height - 2 * margin

        # 최대 판매량 구하기
        max_sales = max(sales, default=1)

        # 각 메뉴에 대한 색상 정의
        colors = {
            '딸기': QColor('#FF6347'),  # 토마토 레드
            '바나나': QColor('#FFD700'),  # 골드
            '초코': QColor('#8B4513'),  # 새들 브라운
            '아포가토': QColor('#6F4E37')  # 커피 색
        }

        # 막대 그리기
        bar_width = graph_width / len(menuItems)
        for i, (menu, sale) in enumerate(zip(menuItems, sales)):
            x = margin + int(i * bar_width)
            y = height - margin - int(sale / max_sales * graph_height)
            painter.setBrush(colors[menu])
            painter.drawRect(int(x), int(y), int(bar_width * 0.8), int(height - margin - y))

            # 막대 위에 텍스트 추가
            painter.setPen(Qt.black)
            painter.setFont(QFont('Arial', 10))
            painter.drawText(int(x + bar_width * 0.4), int(y - 10), str(sale))

            # 메뉴 라벨 추가
            painter.setFont(QFont('Arial', 10, QFont.Bold))
            painter.drawText(int(x + bar_width * 0.4) - 10, int(height - margin + 20), menu)

        # 축 그리기
        pen = QPen(Qt.black, 2)
        painter.setPen(pen)
        painter.drawLine(margin, height - margin, width - margin, height - margin)  # X 축
        painter.drawLine(margin, margin, margin, height - margin)  # Y 축

        # 제목 그리기
        painter.setFont(QFont('Arial', 14, QFont.Bold))
        painter.drawText(int(width / 2 - 50), margin - 25, '메뉴별 일일 판매량')

        painter.end()

        # QImage를 QPixmap으로 변환하여 QLabel에 설정
        pixmap = QPixmap.fromImage(image)
        self.menuSalesGraph.setPixmap(pixmap)

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