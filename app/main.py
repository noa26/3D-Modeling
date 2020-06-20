from PyQt5 import QtCore, QtGui, QtWidgets, uic


class MainWindow(QtWidgets.QMainWindow):
	def __init__(self):
		super(MainWindow, self).__init__()
		uic.loadUi('APP.ui', self)
		
if __name__ == '__main__':
	import sys
	app = QtWidgets.QApplication(sys.argv)
	win = MainWindow()
	win.show()
	sys.exit(app.exec_())