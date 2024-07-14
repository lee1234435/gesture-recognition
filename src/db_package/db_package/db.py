import mysql.connector
from mysql.connector import Error
from datetime import datetime, timedelta
import random
import rclpy as rp
from rclpy.node import Node
from interface_package.msg import StockInfo, StocksArray, OrderInfo
from interface_package.srv import DailyTotalSales, MonthTotalSales, Stocks, ModifyStocks, DailySales

class DataBaseNode(Node):
    def __init__(self):
        super().__init__('database_node')
        self.initDataBase()
        self.initService()

    def initDataBase(self):
        self.dbManager = DatabaseManager(
                host="localhost",
                user="root",
                password="amr231218!",
                database="ArisTeam5"
            )
        self.dbManager.connect()

    def initService(self):
        self.dailyTotalSalesService = self.create_service(DailyTotalSales, "dailyTotalSales", self.dailyTotalSalesCallback)
        self.monthTotalSalesService = self.create_service(MonthTotalSales, "monthTotalSales", self.monthTotalSalesCallback)
        self.stocksService = self.create_service(Stocks, "stocks", self.stocksCallback)
        self.modifyStocksService = self.create_service(ModifyStocks, "modifyStocks", self.modifyStocksCallback)
        self.dailySalesService = self.create_service(DailySales, "dailySales", self.dailySalesCallback)

    def dailyTotalSalesCallback(self, request, response):
        year = request.year
        month = request.month
        self.get_logger().info(f"Get request from dailyTotalSalesClient year : {year}, month : {month}")
        results = self.dbManager.getDailyTotalSales(year, month)
        response.data = [f"{record[0]},{record[1]}" for record in results]

        return response
    
    def monthTotalSalesCallback(self, request, response):
        year = request.year
        month = request.month
        self.get_logger().info(f"Get request from monthTotalSalesClient year : {year}, month : {month}")
        results = self.dbManager.getMonthTotalSales(year, month)
        response.total_sales = int(results[0][0])

        return response

    def stocksCallback(self, request, response):
        self.get_logger().info(f"Get request from stocksClient")
        menu_result, topping_result = self.dbManager.getStocks()
        if menu_result is not None and topping_result is not None:
            menu_items = [StockInfo(name=row[0], stock=row[1]) for row in menu_result]
            toppings = [StockInfo(name=row[0], stock=row[1]) for row in topping_result]
            response.stocks = StocksArray(menu=menu_items, topping=toppings)
        else:
            response.stocks = StocksArray(menu=[], topping=[])

        self.get_logger().info(f"Sending response with {len(response.stocks.menu)} menu items and {len(response.stocks.topping)} toppings")
        return response

    def modifyStocksCallback(self, request, response):
        self.get_logger().info(f"Get request from modifyStocksClient")
        
        menuData = [(item.name, item.stock) for item in request.stocks.menu]
        toppingData = [(item.name, item.stock) for item in request.stocks.topping]

        try:
            self.dbManager.updateStock(menuData, toppingData)
            response.success = True
            self.get_logger().info("Stocks successfully modified in database")
        except Exception as e:
            response.success = False
            self.get_logger().error(f"Failed to modify stocks in database: {e}")

        return response

    def dailySalesCallback(self, request, response):
        year = request.year
        month = request.month
        day = request.day
        
        self.get_logger().info(f"Get request from dailySalesClient - year: {year}, month: {month}, day: {day}")
        date_str = f"{year:04d}-{month:02d}-{day:02d}"
        date_obj = datetime.strptime(date_str, "%Y-%m-%d").date()
        results = self.dbManager.getDailySales(date_obj)

        # response.data = [
        #     OrderInfo(
        #         order_id = row[0],
        #         order_time = row[1],
        #         menu = row[2],
        #         topping = row[3],
        #         quantity = row[4],
        #         price = row[5],
        #     ) for row in results
        #     ] 
    
        orderInfoList = []
        for row in results:
            orderInfo = OrderInfo(
                order_id = row[0],
                order_time =  str(row[1]),
                menu = row[2],
                topping = row[3],
                quantity = row[4],
                price = row[5],
            ) 
            orderInfoList.append(orderInfo)
        
        response.data = orderInfoList

        return response

class DatabaseManager:
    def __init__(self, host, user, password, database):
        self.host = host
        self.user = user
        self.password = password
        self.database = database
        self.connection = None

    def connect(self):
        try:
            if self.connection is None:
                self.connection = mysql.connector.connect(
                    host=self.host,
                    user=self.user,
                    password=self.password,
                    database=self.database
                )
                print("Connection to MYSQL DB successful")
        except Error as e:
            print(f"Error: '{e}' occurred")

    def disconnect(self):
        if self.connection is not None and self.connection.is_connected():
            self.connection.close()
            self.connection = None
            print("Disconnected")

    def _executeQuery(self, query, params=None, fetch=False, many=False):
        if self.connection is None or not self.connection.is_connected():
            print("Not connected to the database")
            return None

        try:
            cursor = self.connection.cursor()
            if many:
                cursor.executemany(query, params)
            else:
                cursor.execute(query, params)
            if fetch:
                result = cursor.fetchall()
                cursor.close()
                return result
            self.connection.commit()
            cursor.close()
        except Error as e:
            print(f"Error: '{e}' occurred")
            return None

    def insertSalesData(self, data):
        query = """
        INSERT INTO sales (order_id, order_datetime, menu_id, topping_id, quantity, price, gender, age)
        VALUES (%s, %s, %s, %s, %s, %s, %s, %s)
        """
        self._executeQuery(query, data, fetch=False, many=True)
        print("Data inserted successfully")

    def updateStock(self, menuData, toppingData):
        self.updateMenuStock(menuData)
        self.updateToppingStock(toppingData)

    def updateMenuStock(self, data):
        query = "UPDATE menu SET stock = %s WHERE name = %s"
        for itemName, itemStock in data:
            # print(f"Updating menu item: {itemName} with stock: {itemStock}")
            self._executeQuery(query, (itemStock, itemName), fetch=False)
        print("Menu stock updated successfully")

    def updateToppingStock(self, data):
        query = "UPDATE topping SET stock = %s WHERE name = %s"
        for itemName, itemStock in data:
            # print(f"Updating topping item: {itemName} with stock: {itemStock}")
            self._executeQuery(query, (itemStock, itemName), fetch=False)
        print("Topping stock updated successfully")

    def getDailySales(self, date):
        query = """
        SELECT s.order_id, TIME(s.order_datetime) AS order_time, m.name, t.name, s.quantity, s.price
        FROM sales s
        JOIN menu m ON s.menu_id = m.id
        LEFT JOIN topping t ON s.topping_id = t.id
        WHERE DATE(s.order_datetime) = %s
        ORDER BY s.order_id
        """
        return self._executeQuery(query, (date,), fetch=True)

    def getDailyTotalSales(self, year, month):
        query = """
        SELECT DATE(order_datetime) as date, SUM(price * quantity) as total_sales
        FROM sales
        WHERE Year(order_datetime) = %s and MONTH(order_datetime) = %s
        GROUP BY date
        ORDER BY date
        """
        return self._executeQuery(query, (year, month), fetch=True)

    def getMonthTotalSales(self, year, month):
        query = """
        SELECT SUM(price * quantity) as total_sales
        FROM sales
        WHERE Year(order_datetime) = %s AND MONTH(order_datetime) = %s
        """
        return self._executeQuery(query, (year, month), fetch=True)

    def getStocks(self):
        menuQuery = "SELECT name, stock FROM menu"
        toppingQuery = "SELECT name, stock FROM topping"

        menuResult = self._executeQuery(menuQuery, fetch=True)
        toppingResult = self._executeQuery(toppingQuery, fetch=True)

        return menuResult, toppingResult

    def getMenuSales(self, date):
        query = """
        SELECT m.name, SUM(s.quantity) as total_quantity
        FROM sales as s
        JOIN menu m ON s.menu_id = m.id
        WHERE DATE(order_datetime) = %s
        GROUP BY m.name
        """
        return self._executeQuery(query, (date,), fetch=True)

    def insertDummyData(self, numRecords, startDate, endDate):
        genders = ['M', 'F']
        menu_ids = [1, 2, 3, 4]  # menu 테이블의 id 값
        topping_ids = [1, 2, 3]  # topping 테이블의 id 값 (임시로 3개의 토핑이 있다고 가정)
        menu_prices = {1: 3000, 2: 3000, 3: 3000, 4: 4000}

        total_days = (endDate - startDate).days + 1
        records_per_day = [random.randint(1, numRecords // total_days * 2) for _ in range(total_days)]
        records_per_day[-1] = numRecords - sum(records_per_day[:-1])  # 나머지 레코드를 마지막 날에 할당

        record_count = 0
        currentDate = startDate
        data = []

        for day, num_records in enumerate(records_per_day):
            daily_order_id = 1  # 매일 order_id를 1로 시작
            for _ in range(num_records):
                if record_count >= numRecords or currentDate > endDate:
                    break

                order_datetime = currentDate + timedelta(
                    minutes=random.randint(0, 59),
                    seconds=random.randint(0, 59)
                )
                menu_id = random.choice(menu_ids)
                topping_id = random.choice(topping_ids)
                quantity = 1
                price = menu_prices[menu_id]
                gender = random.choice(genders)
                age = random.randint(10, 60)

                data.append((daily_order_id, order_datetime, menu_id, topping_id, quantity, price, gender, age))

                daily_order_id += 1
                record_count += 1
            currentDate = startDate + timedelta(days=day + 1)

        self.insertSalesData(data)
        print(f"{record_count}개의 덤프 데이터가 삽입되었습니다.")

    def truncateTable(self):
        query = "TRUNCATE TABLE sales"
        self._executeQuery(query, fetch=False)
        print("sales 테이블이 초기화되었습니다.")

def main():
    rp.init(args=None)
    dbNode = DataBaseNode()
    rp.spin(dbNode)
    dbNode.destroy_node()
    rp.shutdown()


if __name__ == "__main__":
    # main()
    
    # # 테스트용
    dbManager = DatabaseManager(
        host="localhost",
        user="root",
        password="amr231218!",
        database="ArisTeam5"
    )

    try:
        dbManager.connect()

        # 덤프 데이터 넣기
        dbManager.truncateTable()
        startDate = datetime.strptime('2024-07-04 00:00:00', '%Y-%m-%d %H:%M:%S')
        endDate = datetime.strptime('2024-07-14 23:59:59', '%Y-%m-%d %H:%M:%S')
        dbManager.insertDummyData(1000, startDate, endDate)

    finally:
        dbManager.disconnect()
