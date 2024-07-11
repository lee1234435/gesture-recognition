import mysql.connector
from mysql.connector import Error
from datetime import datetime, timedelta
import random


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
                host = self.host,
                user = self.user,
                password = self.password,
                database = self.database
                )
                print("Connection to MYSQL DB successful")
        except Error as e:
            print(f"Error : '{e}' occurred")

    def disconnect(self):
        if self.connection is not None and self.connection.is_connected():
            self.connection.close()
            self.connection = None
            print("Disconnect")

    def insertSalesTable(self, data):
        if self.connection is None:
            print("Not connected yet!")
            return
        
        query = """
        INSERT INTO sales (order_datetime, menu_id, quantity, price, gender, age)
        VALUES (%s, %s, %s, %s, %s, %s)
        """
        try:
            cursor = self.connection.cursor()
            cursor.execute(query, data)
            self.connection.commit()
            cursor.close()
            print("Data inserted successfully")
        except Error as e:
            print(f"Error : '{e}' occurred")

    def getDailySales(self, date):
        if self.connection is None:
            print("Not connected yet!")
        else:
            cursor = self.connection.cursor()
            query = """
            SELECT s.order_id, TIME(s.order_datetime) AS order_time, m.menu, s.quantity, s.price
            FROM sales s
            JOIN menu m ON s.menu_id = m.id
            WHERE DATE(s.order_datetime) = %s
            ORDER BY s.order_id
            """
            cursor.execute(query, (date,))
            result = cursor.fetchall()
            cursor.close()

            return result
        
    def getDailyTotalSales(self, year, month):
        if self.connection is None or not self.connection.is_connected():
            print("Not connected to the database")
            return

        query = """
        SELECT DATE(order_datetime) as date, SUM(price * quantity) as total_sales
        FROM sales
        WHERE Year(order_datetime) = %s and MONTH(order_datetime) = %s
        GROUP BY date
        ORDER BY date
        """
        cursor = self.connection.cursor()
        cursor.execute(query, (year, month))
        result = cursor.fetchall()
        cursor.close()
        return result
    
    def getMonthTotalSales(self, year, month):
        if self.connection is None or not self.connection.is_connected():
            print("Not connected to the database")
            return

        query = """
        SELECT SUM(price * quantity) as total_sales
        FROM sales
        WHERE Year(order_datetime) = %s AND MONTH(order_datetime) = %s
        """
        cursor = self.connection.cursor()
        cursor.execute(query, (year, month))
        result = cursor.fetchall()
        cursor.close()
        return result
    
    def getStock(self):
        if self.connection is None or not self.connection.is_connected():
            print("Not connected to the database")
            return

        query = """
        SELECT menu, stock
        FROM menu
        """

        cursor = self.connection.cursor()
        cursor.execute(query)
        result = cursor.fetchall()
        cursor.close()
        return result
    
    def getMenuSales(self, date):
        if self.connection is None or not self.connection.is_connected():
            print("Not connected to the database")
            return

        query = """
        SELECT m.menu, SUM(s.quantity) as total_quantity
        FROM sales as s
        JOIN menu m ON s.menu_id = m.id
        WHERE DATE(order_datetime) = %s
        GROUP BY m.menu
        """
        cursor = self.connection.cursor()
        cursor.execute(query, (date,))
        result = cursor.fetchall()
        cursor.close()
        return result

    def insertDummyData(self, numRecords, startDate, endDate):
        if self.connection is None or not self.connection.is_connected():
            print("Not connected to the database")
            return
        cursor = self.connection.cursor()
        genders = ['M', 'F']
        menu_ids = [1, 2, 3, 4]  # menu 테이블의 id 값
        menu_prices = {1: 3000, 2: 3000, 3: 3000, 4: 4000}

        total_days = (endDate - startDate).days + 1
        records_per_day = [random.randint(1, numRecords // total_days * 2) for _ in range(total_days)]
        records_per_day[-1] = numRecords - sum(records_per_day[:-1])  # 나머지 레코드를 마지막 날에 할당

        record_count = 0
        currentDate = startDate

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
                quantity = 1
                price = menu_prices[menu_id]
                gender = random.choice(genders)
                age = random.randint(10, 60)

                query = """
                INSERT INTO sales (order_id, order_datetime, menu_id, quantity, price, gender, age)
                VALUES (%s, %s, %s, %s, %s, %s, %s)
                """
                values = (daily_order_id, order_datetime, menu_id, quantity, price, gender, age)
                cursor.execute(query, values)

                daily_order_id += 1
                record_count += 1
            currentDate = startDate + timedelta(days=day+1)

        self.connection.commit()
        cursor.close()
        print(f"{record_count}개의 덤프 데이터가 삽입되었습니다.")

    def truncateTable(self):
        cursor = self.connection.cursor()
        query = "TRUNCATE TABLE sales"
        cursor.execute(query)
        self.connection.commit()
        cursor.close()
        print("sales 테이블이 초기화되었습니다.")

    # def deleteAllData(self):
    #     cursor = self.connection.cursor()
    #     query = "DELETE FROM sales"
    #     cursor.execute(query)
    #     self.connection.commit()
    #     cursor.close()
    #     print("sales 테이블의 모든 데이터가 삭제되었습니다.")
    
    def truncateTable(self):
        cursor = self.connection.cursor()
        query = "TRUNCATE TABLE sales"
        cursor.execute(query)
        self.connection.commit()
        cursor.close()
        print("sales 테이블이 초기화되었습니다.")

#테스트용
if __name__ == "__main__":
    dbManager = DatabaseManager(
        host="localhost",
        user="root",
        password="amr231218!",
        database="ArisTeam5"
    )

    try:
        dbManager.connect()
        
        # daily_sales = dbManager.getDailySales()
        # for sale in daily_sales:
        #     print(sale)

        # date = datetime.now()
        # data = [date, 4, 1, 4000, 'M', 10]
        # dbManager.insertSalesTable(data)

        # data = dbManager.getDailySales("2024", "07")
        # print(data)

        #덤프 데이터 넣기
        dbManager.truncateTable()
        startDate = datetime.strptime('2024-07-04 00:00:00', '%Y-%m-%d %H:%M:%S')
        endDate = datetime.strptime('2024-07-11 23:59:59', '%Y-%m-%d %H:%M:%S')
        dbManager.insertDummyData(1000, startDate, endDate)


    finally:
        dbManager.disconnect()
    