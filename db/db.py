import mysql.connector
from mysql.connector import Error
from datetime import datetime


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
            """
            cursor.execute(query, (date,))
            result = cursor.fetchall()
            cursor.close()

            return result
        
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

    def getDailyTotalSales(self, year, month):
        if self.connection is None or not self.connection.is_connected():
            print("Not connected to the database")
            return []

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


#테스트용
if __name__ == "__main__":
    db_manager = DatabaseManager(
        host="localhost",
        user="root",
        password="amr231218!",
        database="ArisTeam5"
    )

    try:
        db_manager.connect()
        
        # daily_sales = db_manager.getDailySales()
        # for sale in daily_sales:
        #     print(sale)

        # date = datetime.now()
        # data = [date, 2, 1, 3000, 'F', 10]
        # db_manager.insertSalesTable(data)

        data = db_manager.getDailySales("2024", "07")
        print(data)

    finally:
        db_manager.disconnect()
    