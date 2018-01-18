import sys
from PyQt5 import QtWidgets

def window():
    app = QtWidgets.QApplication(sys.argv)
    w = QtWidgets.QWidget()
    l1 = QtWidgets.QLabel(w)
    b = QtWidgets.QPushButton(w)
    b.setText("press me")
    l1.setText('Look here')
    w.setWindowTitle('Test one')
    w.setGeometry(100, 100, 300, 200)
    l1.move(130, 20)
    w.show()
    sys.exit(app.exec_())

window()